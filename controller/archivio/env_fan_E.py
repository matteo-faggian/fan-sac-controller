"""
FanEnv v2 - Ambiente Gymnasium per il controllo in forza di un FAN tramite ESC.
Modello fisico costruito sui dati misurati sul banco (system identification).

La load cell misura:   F_lc = T - m*g*sin(theta) + bias + rumore

Novita' rispetto alla v1 (tutte da sysid_params.json, prodotto da analisi_sysid.py):
  - mappa comando -> spinta da TABELLA misurata (quasi lineare), non piu' k*ESC^2
  - comando nella stessa scala del banco: u in [0, U_MAX] con U_MAX = 20 (scala 0..40)
  - deadband con ISTERESI: il motore parte sopra u_avvio, si ferma sotto u_arresto
  - RITARDO DI AVVIO da fermo (~0.2 s): il motore deve sincronizzarsi prima di spingere
  - tau diverse: in moto (~78 ms) e in spegnimento (~174 ms, il rotore rallenta per inerzia)
  - RITARDO DI ATTUAZIONE di 1-2 passi (misurato ~27 ms + tempo di calcolo sullo STM32)
  - rumore proporzionale alla spinta (vibrazioni): sigma = s0 + s1*T
  - inclinazione massima 14 gradi e force_ref 1.15 N: compito fattibile con ~2.2 N di spinta

SAC osserva (normalizzato):
    [ F_lc / F_scale, errore / F_ref, dF_lc filtrata / F_scale,
      ultimo comando (0..1), integrale errore / I_max ]

SAC decide:
    azione a in [-1, 1]  ->  c = (a+1)/2 in [0, 1]  ->  u = c * U_MAX  (scala del banco 0..40)

Sul banco: impulso ESC [us] = 1472 - u/40 * (1472 - 544)
"""

import json
from collections import deque
from pathlib import Path

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# Valori di riserva, usati solo se sysid_params.json non c'e' (stessi numeri del test di ottobre)
PARAMETRI_DEFAULT = dict(
    u_tab_moto=[1, 2, 3, 4, 6, 8, 10, 12, 15, 18, 20],
    F_tab_moto=[0.123, 0.302, 0.456, 0.617, 0.873, 1.090, 1.331, 1.557, 1.835, 2.206, 2.206],
    u_avvio=1.5, u_arresto=0.5,
    tau_moto=0.078, tau_moto_min=0.033, tau_moto_max=0.112,
    tau_arresto=0.174, ritardo_avvio=0.200, ritardo_moto=0.027,
    rumore_s0=0.0011, rumore_s1=0.0208, zero_dopo=0.132,
)


def carica_parametri(percorso=None):
    """Legge i parametri misurati; se il file non esiste usa i valori di riserva."""
    percorso = Path(percorso) if percorso else Path(__file__).parent / "sysid_params.json"
    par = dict(PARAMETRI_DEFAULT)
    if percorso.exists():
        with open(percorso) as f:
            par.update(json.load(f))
    return par


class FanEnv(gym.Env):

    metadata = {"render_modes": []}

    def __init__(
        self,
        force_ref=1.15,             # [N]   forza desiderata sulla load cell (centro della zona utile)
        dt=0.02,                    # [s]   periodo di controllo (50 Hz)
        max_episode_steps=1000,     # [-]   durata episodio (20 s)
        domain_randomization=True,
        u_max=20.0,                 # [-]   comando massimo nella scala del banco (0..40)
        c_min=0.05,                 # [-]   comando minimo normalizzato: u_min = c_min*u_max = 1,
                                    #       sopra la soglia di arresto -> la rete non puo' spegnere il motore
        max_angle=14.0,             # [deg] inclinazione massima della corsia
        file_parametri=None,        # percorso di sysid_params.json (default: accanto a questo file)
    ):
        super().__init__()

        # ---------------- Controllo ----------------
        self.force_ref = force_ref
        self.dt = dt
        self.max_episode_steps = max_episode_steps
        self.domain_randomization = domain_randomization
        self.u_max = u_max
        self.c_min = c_min
        self.max_angle = max_angle

        # ---------------- Parametri misurati ----------------
        p = carica_parametri(file_parametri)
        # aggiungo il punto (0, 0): sotto il primo livello misurato la spinta tende a zero
        self.u_tab = np.r_[0.0, p["u_tab_moto"]]
        self.F_tab = np.r_[0.0, p["F_tab_moto"]]
        self.u_avvio_nom = p["u_avvio"]
        self.u_arresto_nom = p["u_arresto"]
        self.tau_nom = p["tau_moto"]
        self.tau_min, self.tau_max = p["tau_moto_min"], p["tau_moto_max"]
        self.tau_off_nom = p["tau_arresto"]
        self.rit_avvio_nom = p["ritardo_avvio"]
        self.rumore_s0 = p["rumore_s0"]
        self.rumore_s1_nom = p["rumore_s1"]
        self.zero_dopo = p["zero_dopo"]

        self.g = 9.81
        self.mass_nom = 0.317       # [kg] massa carrellino

        # ---------------- Normalizzazioni / reward ----------------
        self.f_scale = 2.5          # [N]   scala per forza e derivata (spinta massima ~2.2 N)
        # [N*s] saturazione integrale (anti-windup). Era 2.0: con un errore di 0.1 N l'osservazione
        # integral/i_max cresceva di solo 0.05 al secondo, un segnale troppo debole perche' la rete
        # lo usasse per togliere l'offset. Con 0.5 lo stesso errore la sposta 4 volte piu' in fretta.
        self.i_max = 0.5
        self.w_e1 = 1.0             # peso del termine LINEARE sull'errore |e_n| (vedi reward in step)
        self.w_du = 5.0             # peso penalita' variazione comando
        self.w_u = 0.01             # peso sullo sforzo di controllo
        self.beta_df = 0.2          # coefficiente EMA per filtrare dF_lc

        # ---------------- Spazi ----------------
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        obs_high = np.array([10.0, 10.0, 10.0, 1.0, 1.0], dtype=np.float32)
        obs_low = np.array([-10.0, -10.0, -10.0, 0.0, -1.0], dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

    # ======================================================
    # PARAMETRI DELL'EPISODIO
    # ======================================================

    def _sample_parameters(self):
        """Parametri fisici dell'episodio: randomizzati attorno ai valori misurati, o nominali."""
        if self.domain_randomization:
            r = self.np_random
            self.mass = self.mass_nom * r.uniform(0.95, 1.05)        # massa pesata: incertezza piccola
            # guadagno: copre isteresi salita/discesa, riscaldamento e incertezza di calibrazione
            self.gain = r.uniform(0.85, 1.15)
            self.tau = r.uniform(0.8 * self.tau_min + 0.2 * self.tau_nom, self.tau_max)  # ~40..112 ms
            self.tau_off = self.tau_off_nom * r.uniform(0.85, 1.15)
            self.u_avvio = self.u_avvio_nom * r.uniform(0.8, 1.3)
            self.u_arresto = min(self.u_arresto_nom * r.uniform(0.6, 1.6), self.u_avvio)
            self.rit_avvio = self.rit_avvio_nom * r.uniform(0.75, 1.25)
            self.n_ritardo = int(r.integers(1, 3))                    # 1 o 2 passi (20-40 ms)
            self.rumore_s1 = self.rumore_s1_nom * r.uniform(0.7, 1.5)
            self.bias = r.uniform(-0.05, self.zero_dopo + 0.05)       # zero che si sposta (misurato +0.13 N)
        else:
            self.mass = self.mass_nom
            self.gain = 1.0
            self.tau = self.tau_nom
            self.tau_off = self.tau_off_nom
            self.u_avvio = self.u_avvio_nom
            self.u_arresto = self.u_arresto_nom
            self.rit_avvio = self.rit_avvio_nom
            self.n_ritardo = 1
            self.rumore_s1 = self.rumore_s1_nom
            self.bias = 0.0

        # coefficienti del primo ordine discretizzato in modo esatto: alpha = 1 - e^(-dt/tau)
        self.alpha_on = 1.0 - np.exp(-self.dt / self.tau)
        self.alpha_off = 1.0 - np.exp(-self.dt / self.tau_off)
        self.passi_avvio = int(round(self.rit_avvio / self.dt))     # ritardo di avvio in passi

    # ======================================================
    # DISTURBO: INCLINAZIONE DELLA CORSIA (non osservata)
    # ======================================================

    def _schedule_disturbance(self):
        """Programma il prossimo cambio di inclinazione (gradino o rampa)."""
        r = self.np_random
        self.angle_target = r.uniform(-self.max_angle, self.max_angle)
        self.steps_to_next_change = int(r.integers(100, 300))
        if r.random() < 0.5:
            self.angle_rate = np.inf                                 # gradino
        else:
            ramp_time = r.uniform(0.5, 3.0)                          # [s] rampa
            self.angle_rate = abs(self.angle_target - self.angle) / ramp_time

    def _update_disturbance(self):
        self.steps_to_next_change -= 1
        if self.steps_to_next_change <= 0:
            self._schedule_disturbance()
        diff = self.angle_target - self.angle
        max_step = self.angle_rate * self.dt
        self.angle += np.clip(diff, -max_step, max_step)

    # ======================================================
    # MODELLO DEL ROTORE (dai dati misurati)
    # ======================================================

    def _rotor_step(self, u):
        """Aggiorna la spinta per un passo, dato il comando u (scala 0..40) che arriva all'ESC."""
        if not self.in_moto:
            # motore fermo: parte solo se u resta sopra la soglia per tutto il ritardo di avvio
            if u >= self.u_avvio:
                self.conta_avvio += 1
                if self.conta_avvio >= self.passi_avvio:
                    self.in_moto = True
            else:
                self.conta_avvio = 0
        elif u < self.u_arresto:
            # in moto ma sotto la soglia di arresto: il motore si spegne
            self.in_moto = False
            self.conta_avvio = 0

        if self.in_moto:
            target = self.gain * np.interp(u, self.u_tab, self.F_tab)   # spinta a regime da tabella
            self.thrust += self.alpha_on * (target - self.thrust)
        else:
            self.thrust += self.alpha_off * (0.0 - self.thrust)        # rallenta per inerzia

    def _measure(self):
        """Lettura della load cell: spinta - gravita' + zero spostato + rumore proporzionale."""
        gravity_force = self.mass * self.g * np.sin(np.radians(self.angle))
        sigma = self.rumore_s0 + self.rumore_s1 * abs(self.thrust)
        noise = self.np_random.normal(0.0, sigma)
        return self.thrust - gravity_force + self.bias + noise

    def _get_obs(self, force, delta_force, error):
        obs = np.array(
            [
                force / self.f_scale,
                error / self.force_ref,
                delta_force / self.f_scale,
                self.last_c,
                self.integral / self.i_max,
            ],
            dtype=np.float32,
        )
        return np.clip(obs, self.observation_space.low, self.observation_space.high)

    # ======================================================
    # RESET
    # ======================================================

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._sample_parameters()

        self.thrust = 0.0
        self.in_moto = False
        self.conta_avvio = 0
        self.coda_u = deque([0.0] * self.n_ritardo)   # comandi "in viaggio" verso l'ESC
        self.last_c = 0.0
        self.integral = 0.0
        self.df_filt = 0.0
        self.t = 0

        self.angle = self.np_random.uniform(-self.max_angle, self.max_angle)
        self._schedule_disturbance()
        self.angle_target = self.angle

        force = self._measure()
        error = self.force_ref - force
        self.last_force = force

        obs = self._get_obs(force, 0.0, error)
        info = {"force": force, "force_ref": self.force_ref, "angle": self.angle, "ESC": 0.0, "u": 0.0}
        return obs, info

    # ======================================================
    # STEP
    # ======================================================

    def step(self, action):
        self.t += 1

        # azione [-1, 1] -> comando normalizzato c [c_min, 1] -> u nella scala del banco
        a = float(np.clip(action[0], -1.0, 1.0))
        c = self.c_min + (1.0 - self.c_min) * 0.5 * (a + 1.0)
        u = c * self.u_max

        # ritardo di attuazione: il comando di adesso arriva all'ESC dopo n_ritardo passi
        self.coda_u.append(u)
        u_applicato = self.coda_u.popleft()

        self._update_disturbance()
        self._rotor_step(u_applicato)

        force = self._measure()
        self.df_filt += self.beta_df * ((force - self.last_force) - self.df_filt)
        error = self.force_ref - force
        self.integral = float(np.clip(self.integral + error * self.dt, -self.i_max, self.i_max))

        # reward normalizzato
        # Termine quadratico e_n^2: punisce molto gli errori grandi, ma vicino a zero e' quasi
        # piatto (derivata 2*e_n -> 0): un errore costante di 0.15 N costa solo 0.017 a passo,
        # e la policy SAC v2 si "accontentava" di lasciarlo. Il termine lineare |e_n| ha derivata
        # costante: anche l'ultimo pezzetto di errore viene pagato, e la policy e' spinta ad annullarlo.
        e_n = error / self.force_ref
        du = c - self.last_c
        reward = (-(e_n ** 2) - self.w_e1 * abs(e_n)
                  - self.w_du * du ** 2 - self.w_u * c ** 2)

        self.last_force = force
        self.last_c = c
        obs = self._get_obs(force, self.df_filt, error)

        terminated = False
        truncated = self.t >= self.max_episode_steps
        info = {
            "force": force,
            "force_ref": self.force_ref,
            "angle": self.angle,
            "ESC": c,               # comando normalizzato 0..1 (come nella v1)
            "u": u,                 # comando nella scala del banco 0..40
            "thrust": self.thrust,
            "in_moto": self.in_moto,
        }
        return obs, float(reward), terminated, truncated, info


# ==========================================================
# TEST RAPIDO + TRAINING
# ==========================================================

if __name__ == "__main__":
    from gymnasium.utils.env_checker import check_env

    env = FanEnv()
    check_env(env)
    print("check_env: OK")

    obs, info = env.reset(seed=0)
    env.action_space.seed(0)
    total = 0.0
    for _ in range(env.max_episode_steps):
        obs, r, term, trunc, info = env.step(env.action_space.sample())
        total += r
        if term or trunc:
            break
    print(f"Reward episodio random: {total:.1f}")

    try:
        from stable_baselines3 import SAC
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.callbacks import EvalCallback
    except ImportError:
        print("stable-baselines3 non installato: training saltato.")
        raise SystemExit

    train_env = Monitor(FanEnv())
    eval_cb_env = Monitor(FanEnv(domain_randomization=False))

    eval_cb = EvalCallback(
        eval_cb_env,
        best_model_save_path="./best_sac_fan",
        log_path="./logs",
        eval_freq=5_000,
        n_eval_episodes=5,
        deterministic=True,
    )

    model = SAC(
        "MlpPolicy",
        train_env,
        learning_rate=3e-4,
        buffer_size=200_000,
        batch_size=256,
        gamma=0.99,      # orizzonte ~1/(1-gamma) = 100 passi = 2 s (era 1 s): l'errore che resta
                         # dopo un cambio di inclinazione dura secondi, la policy deve "vederlo"
        verbose=1,
        seed=0,
    )
    model.learn(total_timesteps=300_000, callback=eval_cb)   # la v2 a 150k non era a convergenza
    model.save("sac_fan")

    model = SAC.load("./best_sac_fan/best_model.zip")   # .zip esplicito: evita conflitti con cartelle omonime

    def evaluate(env, n_ep=10):
        rms, du, spegnimenti = [], [], []
        for ep in range(n_ep):
            obs, _ = env.reset(seed=100 + ep)
            errs, acts, moto = [], [], []
            for _ in range(env.max_episode_steps):
                a, _ = model.predict(obs, deterministic=True)
                obs, r, term, trunc, info = env.step(a)
                errs.append(info["force_ref"] - info["force"])
                acts.append(info["ESC"])
                moto.append(info["in_moto"])
                if term or trunc:
                    break
            errs, acts, moto = np.array(errs), np.array(acts), np.array(moto)
            rms.append(np.sqrt(np.mean(errs[100:] ** 2)))      # escludo i primi 2 s (avvio)
            du.append(np.mean(np.abs(np.diff(acts))))
            spegnimenti.append(int(np.sum(moto[:-1] & ~moto[1:])))
        return np.mean(rms), np.std(rms), np.mean(du), np.mean(spegnimenti)

    for name, dr in [("nominale", False), ("randomizzato", True)]:
        m, s, d, k = evaluate(FanEnv(domain_randomization=dr))
        print(f"[{name}] RMS errore (dopo 2 s): {m:.3f} ± {s:.3f} N | mean|Δc|: {d:.4f} | "
              f"spegnimenti motore/episodio: {k:.1f}")

    import matplotlib.pyplot as plt

    data = np.load("./logs/evaluations.npz")
    timesteps, results = data["timesteps"], data["results"]
    mean_r, std_r = results.mean(axis=1), results.std(axis=1)
    plt.figure(figsize=(8, 5))
    plt.plot(timesteps, mean_r, marker="o", label="Reward medio (5 ep)")
    plt.fill_between(timesteps, mean_r - std_r, mean_r + std_r, alpha=0.25, label="±1σ")
    plt.title("Curva di apprendimento – EvalCallback")
    plt.xlabel("Timesteps"); plt.ylabel("Reward episodio")
    plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig("learning_curve.png", dpi=150)
    plt.show()
