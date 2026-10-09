/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "usbd_cdc_if.h"   // CDC_Transmit_FS: invio sulla seriale USB
#include "actor.h"         // inferenza della rete (actor SAC, policy F)
#include "test_vectors.h"  // 100 osservazioni + azioni di PyTorch: autotest all'accensione

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---------- Costanti copiate da controller/env_fan.py e controllo_reale.py ----------
 * NON cambiarle senza riaddestrare: la rete ha imparato con QUESTI valori.          */
#define N_OBS        7          // ingressi della rete
#define DT_MS        20         // [ms] periodo di controllo (env: dt = 0.02 -> 50 Hz)
#define DT_S         0.02f      // [s]  lo stesso, in secondi (serve all'integrale)
#define F_REF        1.15f      // [N]  forza desiderata (env: force_ref). La rete e' addestrata a 1.15 N:
                                //      altri valori peggiorano (vedi docs/08, controller/prova_riferimento.py)
#define F_SCALE      2.5f       // [N]  scala di forza e derivata  (env: f_scale)
#define I_MAX        0.5f       // [N*s] saturazione integrale     (env: i_max)
#define BETA_DF      0.2f       // [-]  filtro della derivata      (env: beta_df)
#define C_MIN        0.05f      // [-]  comando minimo normalizzato (env: c_min)
#define U_MAX        20.0f      // [-]  comando massimo, scala banco 0..40 (env: u_max)

/* ---------- Mappatura ESC: identica a firmware/banco_sysid_v2.ino ---------- */
#define US_NEUTRO    1472       // [us] motore fermo (= Servo.write(90) dell'Arduino)
#define US_PIENO     544        // [us] piena potenza nel verso che spinge la cella
#define U_SCALA      40.0f      // u = 40 -> US_PIENO
#define U_LIMITE     25.0f      // limite di sicurezza (alimentatore in limitazione oltre)

/* ---------- Cella di carico ---------- */
#define K_DEFAULT    (-199948.5f) // [conteggi/N] calibrazione sysid v2, HX711 a 5 V
#define MASSA_CAL_KG 0.317f       // [kg] peso noto per la calibrazione (comando K)
#define G_CAL        9.80665f     // [m/s^2]

/* ---------- Sicurezza (come controllo_reale.py) ---------- */
#define F_ALLARME    8.0f       // [N]  forza oltre questo valore -> stop
#define ETA_MAX_MS   100        // [ms] campione valido piu' vecchio di cosi' -> stop
#define DURATA_MAX_MS 180000    // [ms] la prova si ferma da sola dopo 3 minuti
#define T_ARMO_MS    3000       // [ms] neutro all'accensione: l'ESC si arma

/* ---------- Pin ----------
 * ESC  segnale -> PB6  (TIM4 canale 1, funzione alternativa AF2)
 * HX711 SCK    -> PB12 (uscita)
 * HX711 DT     -> PB13 (ingresso)                                        */
#define HX_SCK_PIN   12
#define HX_DT_PIN    13

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;   // definito in usb_device.c
static char txbuf[200];        // buffer di uscita USB: deve restare valido finche' la USB
                               // non ha finito di spedirlo, per questo e' globale

/* Ultimo comando arrivato dal PC. "volatile": viene scritto dentro l'interrupt USB
 * e letto dal ciclo principale, il compilatore non deve tenerlo "in cache" in un registro. */
static volatile char cmd_ricevuto = 0;

/* HX711: ultimo campione VALIDO e istante in cui e' arrivato */
static int32_t  hx_raw = 0;
static uint32_t hx_t_ms = 0;
static uint8_t  hx_almeno_uno = 0;   // 1 dopo il primo campione valido
static uint32_t hx_invalidi = 0;     // campioni scartati (diagnostica)

/* Calibrazione: F = (raw - raw_zero) / k */
static float   raw_zero = 0.0f;
static float   k_cnt_per_n = K_DEFAULT;
static uint8_t zero_fatto = 0;

/* Stato del controllore: identico a env.reset() */
static float df_filt, integrale, F_prec;
static float storia_c[3];            // [0] = ultimo comando, [1] penultimo, [2] terzultimo
static uint32_t t_start, t_prossimo, n_passi, righe_perse;
static uint8_t in_controllo = 0;     // 0 = fermo (IDLE), 1 = la rete comanda l'ESC

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* =========================================================================
 *  Tempi brevi: contatore di cicli DWT (conta a 168 MHz, 1 us = 168 cicli)
 * ========================================================================= */
static void delay_us(uint32_t us)
{
  uint32_t c0 = DWT->CYCCNT;
  uint32_t n = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - c0) < n) { }      // la sottrazione tra unsigned funziona anche
                                          // quando il contatore riparte da zero
}

/* =========================================================================
 *  Watchdog indipendente (IWDG): se il programma si blocca per ~0.5 s,
 *  il chip si resetta da solo. Al riavvio l'ESC riceve di nuovo il neutro.
 *  Gira con l'oscillatore interno LSI (~32 kHz), indipendente dal clock principale.
 *  ATTENZIONE: una volta avviato non si puo' piu' fermare fino al reset.
 * ========================================================================= */
static void iwdg_avvia(void)
{
  IWDG->KR  = 0xCCCC;          // avvia il watchdog
  IWDG->KR  = 0x5555;          // sblocca la scrittura di PR e RLR
  IWDG->PR  = 3;               // prescaler /32: 32 kHz / 32 = ~1 kHz -> 1 conteggio ~ 1 ms
  IWDG->RLR = 500;             // ~500 ms (l'LSI e' impreciso: in pratica 0.35..0.95 s)
  while (IWDG->SR != 0) { }    // attendo che i valori siano stati presi
  IWDG->KR  = 0xAAAA;          // prima "carica"
}
static inline void iwdg_carica(void) { IWDG->KR = 0xAAAA; }   // "sono vivo"

/* =========================================================================
 *  USB: due modi di stampare
 * ========================================================================= */
/* Aspetta (max 50 ms) che l'invio precedente sia finito: per i messaggi importanti. */
static void usb_print(const char *s)
{
  USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  uint32_t t0 = HAL_GetTick();
  while (h != NULL && h->TxState != 0) {
    if (HAL_GetTick() - t0 > 50) return;        // PC non in ascolto: rinuncio
  }
  if (s != txbuf) strncpy(txbuf, s, sizeof(txbuf) - 1);   // copio solo se serve
  CDC_Transmit_FS((uint8_t *)txbuf, strlen(txbuf));
}
/* Non aspetta MAI: se la USB e' occupata la riga si perde (e la conto).
 * Si usa dentro il ciclo di controllo, che non deve mai essere rallentato dalla USB. */
static uint8_t usb_print_se_libero(const char *s)
{
  USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (h == NULL || h->TxState != 0) return 0;
  if (s != txbuf) strncpy(txbuf, s, sizeof(txbuf) - 1);   // copio solo se serve
  CDC_Transmit_FS((uint8_t *)txbuf, strlen(txbuf));
  return 1;
}

/* Chiamata DENTRO l'interrupt USB quando arrivano byte dal PC (vedi usbd_cdc_if.c).
 * Tengo solo l'ultima lettera ricevuta, in maiuscolo: il lavoro lo fa il ciclo principale. */
void cdc_ricevuto(const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++) {
    char ch = (char)buf[i];
    if (ch >= 'a' && ch <= 'z') ch -= 32;   // minuscola -> maiuscola
    if (ch > ' ') cmd_ricevuto = ch;        // ignoro spazi e a capo
  }
}

/* =========================================================================
 *  ESC: PWM a 50 Hz su PB6 con il TIMER 4, programmato direttamente nei registri
 * ========================================================================= */
/* u (scala del banco 0..40) -> larghezza d'impulso. COPIA di u_to_us() dello sketch Arduino:
 * u = 0 -> 1472 us, u = 40 -> 544 us. L'impulso DIMINUISCE quando la spinta aumenta. */
static uint32_t u_to_us(float u)
{
  if (u < 0.0f)     u = 0.0f;
  if (u > U_LIMITE) u = U_LIMITE;
  return (uint32_t)(US_NEUTRO - (int32_t)(u / U_SCALA * (US_NEUTRO - US_PIENO) + 0.5f));
}

static void esc_scrivi_us(uint32_t us)
{
  /* doppia sicurezza: mai fuori dall'intervallo [impulso a U_LIMITE, neutro] */
  uint32_t us_min = u_to_us(U_LIMITE);
  if (us < us_min)    us = us_min;
  if (us > US_NEUTRO) us = US_NEUTRO;
  TIM4->CCR1 = us;          // con 1 tick = 1 us, il registro vale direttamente i microsecondi
}

/* Restituisce la frequenza (Hz) che entra nel timer, calcolata dai registri del clock:
 * se il prescaler di APB1 e' diverso da 1, i timer ricevono il DOPPIO di PCLK1. */
static uint32_t tim4_clock_hz(void)
{
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;  // 0xx = /1
  return (ppre1 < 4) ? pclk1 : 2U * pclk1;
}

static void esc_avvia(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // accendo il clock della porta B
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;     // accendo il clock del timer 4
  (void)RCC->APB1ENR;                     // lettura di attesa: il clock deve stabilizzarsi

  /* PB6 in modalita' "funzione alternativa" (MODER = 10), funzione AF2 = TIM4_CH1 */
  GPIOB->MODER  = (GPIOB->MODER & ~(3U << (6 * 2))) | (2U << (6 * 2));
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFU << (6 * 4))) | (2U << (6 * 4));

  /* Timer: clock / (PSC+1) = 1 MHz -> 1 tick = 1 us; periodo = ARR+1 = 20000 us = 50 Hz,
   * lo stesso periodo che usava la libreria Servo dell'Arduino. */
  TIM4->PSC  = tim4_clock_hz() / 1000000U - 1U;   // con 84 MHz -> 83
  TIM4->ARR  = 20000U - 1U;
  TIM4->CCR1 = US_NEUTRO;                          // si parte dal neutro
  TIM4->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos)         // PWM modo 1: uscita alta finche' CNT < CCR1
              | TIM_CCMR1_OC1PE;                   // il nuovo CCR1 vale dal prossimo periodo
  TIM4->CCER = TIM_CCER_CC1E;                      // abilito l'uscita del canale 1
  TIM4->CR1  = TIM_CR1_ARPE;
  TIM4->EGR  = TIM_EGR_UG;                         // carico subito PSC/ARR/CCR1
  TIM4->CR1 |= TIM_CR1_CEN;                        // via
}

/* =========================================================================
 *  HX711 letto "a mano" (stesso protocollo dello sketch banco_sysid_v2.ino)
 * ========================================================================= */
static void hx_avvia(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  /* PB12 = SCK: uscita push-pull (MODER = 01), parte BASSA (basso = HX711 acceso) */
  GPIOB->BSRR  = 1U << (HX_SCK_PIN + 16);                 // scrivo 0
  GPIOB->MODER = (GPIOB->MODER & ~(3U << (HX_SCK_PIN * 2))) | (1U << (HX_SCK_PIN * 2));
  /* PB13 = DT: ingresso (MODER = 00) con PULL-UP interno (PUPDR = 01).
   * L'HX711 pilota DT attivamente (alto e basso), quindi il pull-up non lo disturba.
   * Serve se il filo si stacca: DT resta ALTO = "nessun campione pronto" -> dopo
   * ETA_MAX_MS il controllo si ferma, invece di leggere rumore da un pin scollegato. */
  GPIOB->MODER &= ~(3U << (HX_DT_PIN * 2));
  GPIOB->PUPDR  = (GPIOB->PUPDR & ~(3U << (HX_DT_PIN * 2))) | (1U << (HX_DT_PIN * 2));
}

static inline uint8_t hx_pronto(void)       // DT basso = campione pronto
{
  return (GPIOB->IDR & (1U << HX_DT_PIN)) == 0;
}

/* Un impulso su SCK e lettura di DT. Vincolo del chip: SCK alto per NON piu' di 50 us,
 * altrimenti va in power-down. Per questo blocco gli interrupt SOLO mentre SCK e' alto
 * (~1 us): un interrupt USB in quel momento allungherebbe l'impulso. */
static inline uint32_t hx_impulso(void)
{
  uint32_t bit;
  __disable_irq();
  GPIOB->BSRR = 1U << HX_SCK_PIN;             // SCK alto: il chip mette il bit su DT
  delay_us(1);
  bit = (GPIOB->IDR >> HX_DT_PIN) & 1U;
  GPIOB->BSRR = 1U << (HX_SCK_PIN + 16);      // SCK basso
  __enable_irq();
  delay_us(1);
  return bit;
}

static int32_t hx_leggi(void)
{
  uint32_t v = 0;
  for (int i = 0; i < 24; i++) v = (v << 1) | hx_impulso();   // 24 bit, MSB per primo
  (void)hx_impulso();                       // 25-esimo impulso: canale A, guadagno 128
  if (v & 0x800000U) v |= 0xFF000000U;      // complemento a 2 su 24 bit -> 32 bit
  return (int32_t)v;
}

/* -1 = DT sempre alto (chip spento o filo staccato); estremi = ingresso saturato */
static inline uint8_t hx_valido(int32_t raw)
{
  return raw != -1 && raw != 8388607 && raw != -8388608;
}

/* Da chiamare il piu' spesso possibile: se c'e' un campione pronto lo legge. */
static void hx_aggiorna(void)
{
  if (!hx_pronto()) return;
  int32_t raw = hx_leggi();
  if (hx_valido(raw)) { hx_raw = raw; hx_t_ms = HAL_GetTick(); hx_almeno_uno = 1; }
  else                { hx_invalidi++; }
}

/* Mediana dei campioni validi raccolti in 'ms' millisecondi (motore al neutro).
 * La mediana, come in controllo_reale.py, ignora qualche campione storto. */
#define N_MED 400
static int32_t med_buf[N_MED];
static uint8_t hx_mediana(uint32_t ms, float *risultato)
{
  int n = 0;
  uint32_t t0 = HAL_GetTick(), t_ultimo = hx_t_ms;
  while (HAL_GetTick() - t0 < ms) {
    iwdg_carica();
    hx_aggiorna();
    if (hx_t_ms != t_ultimo && n < N_MED) { med_buf[n++] = hx_raw; t_ultimo = hx_t_ms; }
  }
  if (n < 10) return 0;                        // troppo pochi campioni: qualcosa non va
  for (int i = 1; i < n; i++) {                // insertion sort (n ~ 240: bastano pochi ms)
    int32_t x = med_buf[i]; int j = i - 1;
    while (j >= 0 && med_buf[j] > x) { med_buf[j + 1] = med_buf[j]; j--; }
    med_buf[j + 1] = x;
  }
  *risultato = (n % 2) ? (float)med_buf[n / 2]
                       : 0.5f * ((float)med_buf[n / 2 - 1] + (float)med_buf[n / 2]);
  return 1;
}

/* =========================================================================
 *  Controllore: le stesse operazioni di controllo_reale.py, righe 271-293
 * ========================================================================= */
static inline float limita(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void ferma(const char *motivo)
{
  esc_scrivi_us(US_NEUTRO);                  // PRIMA il motore, poi tutto il resto
  in_controllo = 0;
  snprintf(txbuf, sizeof(txbuf), "# STOP: %s (passi %lu, righe perse %lu, HX711 non validi %lu)\r\n",
           motivo, (unsigned long)n_passi, (unsigned long)righe_perse, (unsigned long)hx_invalidi);
  usb_print(txbuf);
}

static void avvia_controllo(void)
{
  if (!zero_fatto)   { usb_print("# Prima fai lo zero: comando Z\r\n"); return; }
  if (!hx_almeno_uno || HAL_GetTick() - hx_t_ms > ETA_MAX_MS) {
    usb_print("# Nessun campione recente dall'HX711: controlla i fili\r\n"); return;
  }
  /* stato identico a env.reset(): integrale, filtro e memoria dei comandi azzerati */
  df_filt = 0.0f; integrale = 0.0f;
  storia_c[0] = storia_c[1] = storia_c[2] = 0.0f;
  F_prec = ((float)hx_raw - raw_zero) / k_cnt_per_n;
  n_passi = 0; righe_perse = 0;
  t_start = HAL_GetTick();
  t_prossimo = t_start;
  in_controllo = 1;
  usb_print("# CONTROLLO AVVIATO. Colonne: t_ms,F_mN,c_x1000,u_us,inferenza_us,eta_ms\r\n");
}

/* Un passo di controllo. Ordine IDENTICO a controllo_reale.py. */
static void passo_controllo(void)
{
  uint32_t ora = HAL_GetTick();
  uint32_t eta = ora - hx_t_ms;               // eta' dell'ultimo campione valido
  if (eta > ETA_MAX_MS)               { ferma("nessun campione valido dall'HX711"); return; }
  if (ora - t_start > DURATA_MAX_MS)  { ferma("durata massima raggiunta");          return; }

  /* --- misura --- */
  float F = ((float)hx_raw - raw_zero) / k_cnt_per_n;               // [N]
  if (fabsf(F) > F_ALLARME)           { ferma("forza oltre F_ALLARME");             return; }

  /* --- osservazione (env_fan.py: step() e _get_obs()) --- */
  df_filt += BETA_DF * ((F - F_prec) - df_filt);                     // derivata filtrata
  float e = F_REF - F;                                               // errore
  integrale = limita(integrale + e * DT_S, -I_MAX, I_MAX);           // integrale anti-windup
  float obs[N_OBS] = {
    limita(F / F_SCALE,       -10.0f, 10.0f),
    limita(e / F_REF,         -10.0f, 10.0f),
    limita(df_filt / F_SCALE, -10.0f, 10.0f),
    limita(storia_c[0],         0.0f,  1.0f),   // ultimo comando
    limita(storia_c[1],         0.0f,  1.0f),   // penultimo
    limita(storia_c[2],         0.0f,  1.0f),   // terzultimo
    limita(integrale / I_MAX,  -1.0f,  1.0f),
  };

  /* --- rete --- */
  uint32_t c0 = DWT->CYCCNT;
  float a = limita(actor_forward(obs), -1.0f, 1.0f);
  uint32_t inf_us = (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000U);

  /* --- azione -> comando -> impulso (env: step(), righe 133-136) --- */
  float c = C_MIN + (1.0f - C_MIN) * 0.5f * (a + 1.0f);              // [c_min, 1]
  float u = c * U_MAX;                                               // scala del banco
  uint32_t us = u_to_us(u);
  esc_scrivi_us(us);

  /* --- memoria dei comandi, aggiornata DOPO aver calcolato il nuovo (come registra_comando) --- */
  storia_c[2] = storia_c[1];
  storia_c[1] = storia_c[0];
  storia_c[0] = c;
  F_prec = F;
  n_passi++;

  /* --- telemetria: interi scalati (il printf dei float non e' abilitato) --- */
  char riga[80];
  snprintf(riga, sizeof(riga), "%lu,%ld,%ld,%lu,%lu,%lu\r\n",
           (unsigned long)(ora - t_start), (long)lroundf(F * 1000.0f), (long)lroundf(c * 1000.0f),
           (unsigned long)us, (unsigned long)inf_us, (unsigned long)eta);
  if (!usb_print_se_libero(riga)) righe_perse++;
  if (n_passi % 25 == 0) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);       // LED D2: 1 Hz in controllo
}

static void stampa_aiuto(void)
{
  usb_print("# Comandi: Z = zero (cella scarica, motore fermo)  K = calibra con 317 g\r\n"
            "#          S = avvia controllo   X = stop   P = stato   H = aiuto\r\n");
}

static void stampa_stato(void)
{
  float F = zero_fatto ? ((float)hx_raw - raw_zero) / k_cnt_per_n : 0.0f;
  snprintf(txbuf, sizeof(txbuf),
           "# stato: %s | zero %s | raw %ld | F %ld mN | k %ld cnt/N | eta camp. %lu ms | non validi %lu\r\n",
           in_controllo ? "CONTROLLO" : "FERMO", zero_fatto ? "fatto" : "DA FARE", (long)hx_raw,
           (long)lroundf(F * 1000.0f), (long)lroundf(k_cnt_per_n),
           (unsigned long)(HAL_GetTick() - hx_t_ms), (unsigned long)hx_invalidi);
  usb_print(txbuf);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // Contatore di cicli della CPU (DWT): serve a delay_us() e a misurare l'inferenza
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  esc_avvia();          // PRIMA di tutto: l'ESC riceve subito il neutro
  hx_avvia();

  /* Autotest della rete: deve dare gli stessi numeri di PyTorch */
  float err_max = 0.0f;
  for (int k = 0; k < N_TEST; k++) {
    float e = fabsf(actor_forward(&TEST_OBS[k * N_OBS]) - TEST_AZIONE[k]);
    if (e > err_max) err_max = e;
  }
  uint8_t rete_ok = (err_max < 1e-5f);

  /* Armo l'ESC: neutro per 3 s (come il delay(3000) dello sketch Arduino).
   * Intanto leggo l'HX711 e do tempo a Windows di riconoscere la porta. */
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < T_ARMO_MS) hx_aggiorna();

  iwdg_avvia();         // da qui in poi: se il programma si blocca, reset automatico

  snprintf(txbuf, sizeof(txbuf), "# FanStm32 pronto | clock %lu MHz | TIM4 PSC %lu | autotest rete %s (errore %lu e-9)\r\n",
           (unsigned long)(SystemCoreClock / 1000000U), (unsigned long)TIM4->PSC,
           rete_ok ? "OK" : "FALLITO", (unsigned long)(err_max * 1e9f));
  usb_print(txbuf);
  stampa_aiuto();

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    iwdg_carica();          // "sono vivo": senza questa chiamata il chip si resetta
    hx_aggiorna();          // legge l'HX711 appena ha un campione (circa ogni 12 ms)

    /* --- comando dal PC (arrivato nell'interrupt USB) --- */
    char cmd = cmd_ricevuto;
    cmd_ricevuto = 0;
    if (cmd == 'X' && in_controllo) ferma("comando X dal PC");
    else if (cmd == 'H') stampa_aiuto();
    else if (cmd == 'P') stampa_stato();
    else if (!in_controllo) {
      if (cmd == 'Z') {
        esc_scrivi_us(US_NEUTRO);
        usb_print("# Zero: 3 s, cella SCARICA e motore fermo...\r\n");
        if (hx_mediana(3000, &raw_zero)) {
          zero_fatto = 1;
          snprintf(txbuf, sizeof(txbuf), "# Zero fatto: raw_zero = %ld\r\n", (long)lroundf(raw_zero));
        } else {
          snprintf(txbuf, sizeof(txbuf), "# Zero FALLITO: pochi campioni validi dall'HX711\r\n");
        }
        usb_print(txbuf);
      } else if (cmd == 'K') {
        float raw_peso;
        if (!zero_fatto) usb_print("# Prima fai lo zero: comando Z\r\n");
        else {
          usb_print("# Calibrazione: 3 s con 317 g appoggiati sulla cella...\r\n");
          if (hx_mediana(3000, &raw_peso)) {
            k_cnt_per_n = (raw_peso - raw_zero) / (MASSA_CAL_KG * G_CAL);
            snprintf(txbuf, sizeof(txbuf), "# Nuovo k = %ld conteggi/N (default %ld). Vale fino al reset.\r\n",
                     (long)lroundf(k_cnt_per_n), (long)lroundf(K_DEFAULT));
          } else {
            snprintf(txbuf, sizeof(txbuf), "# Calibrazione FALLITA: pochi campioni validi\r\n");
          }
          usb_print(txbuf);
        }
      } else if (cmd == 'S') {
        avvia_controllo();
      }
    }

    /* --- passo di controllo ogni 20 ms --- */
    if (in_controllo && (int32_t)(HAL_GetTick() - t_prossimo) >= 0) {
      t_prossimo += DT_MS;       // istanti fissi: 0, 20, 40, ... ms (non "20 ms dopo la fine")
      passo_controllo();
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
