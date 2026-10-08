/* =============================================================================
 * actor.c — inferenza dell'actor SAC in C puro (nessuna libreria esterna)
 * -----------------------------------------------------------------------------
 * Rete: x(7) -> Linear 256 -> ReLU -> Linear 256 -> ReLU -> Linear 1 -> tanh
 * Un "Linear" calcola  y[r] = b[r] + somma_c W[r][c] * x[c]   (prodotto matrice-vettore)
 * I pesi sono in actor_weights.h come array piatti: W[r][c] = W[r*n_in + c].
 * Operazioni: 7*256 + 256*256 + 256 = ~67.6k moltiplicazioni-somme (MAC).
 * ============================================================================= */
#include <math.h>
#include "actor.h"
#include "actor_weights.h"

/* Buffer degli strati nascosti in RAM: 2 x 256 float = 2 KB.
 * "static" li tiene fuori dallo stack (che sul micro e' piccolo). */
static float h1[ACT_N_H1];
static float h2[ACT_N_H2];

/* y = W x + b, con ReLU opzionale. W ha n_out righe e n_in colonne. */
static void lineare(const float *W, const float *b, const float *x, float *y,
                    int n_in, int n_out, int relu)
{
    for (int r = 0; r < n_out; r++) {
        const float *riga = &W[r * n_in];  /* puntatore all'inizio della riga r */
        float s = b[r];                    /* parto dal bias */
        for (int c = 0; c < n_in; c++) {
            s += riga[c] * x[c];           /* una MAC: con la FPU e' 1 istruzione */
        }
        y[r] = (relu && s < 0.0f) ? 0.0f : s;  /* ReLU: i negativi diventano 0 */
    }
}

float actor_forward(const float *obs)
{
    float mu;
    lineare(W1, B1, obs, h1, ACT_N_IN, ACT_N_H1, 1);   /* 7   -> 256, ReLU */
    lineare(W2, B2, h1,  h2, ACT_N_H1, ACT_N_H2, 1);   /* 256 -> 256, ReLU */
    lineare(WMU, BMU, h2, &mu, ACT_N_H2, 1, 0);        /* 256 -> 1, lineare */
    return tanhf(mu);                                  /* schiaccia in [-1, 1] */
}
