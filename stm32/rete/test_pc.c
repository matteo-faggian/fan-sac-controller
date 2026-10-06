/* test_pc.c — verifica sul PC: il C deve riprodurre PyTorch sui 100 vettori di test.
 * Compila:  gcc -O2 -o test_pc test_pc.c actor.c -lm */
#include <stdio.h>
#include <math.h>
#include "actor.h"
#include "actor_weights.h"
#include "test_vectors.h"

int main(void)
{
    float err_max = 0.0f;
    for (int k = 0; k < N_TEST; k++) {
        float a = actor_forward(&TEST_OBS[k * ACT_N_IN]);
        float e = fabsf(a - TEST_AZIONE[k]);
        if (e > err_max) err_max = e;
        if (k < 3) printf("test %d: C = %+.6f   PyTorch = %+.6f\n", k, a, TEST_AZIONE[k]);
    }
    printf("errore massimo su %d test: %.2e  -> %s\n", N_TEST, err_max,
           err_max < 1e-5f ? "OK" : "DIVERSO!");
    return err_max < 1e-5f ? 0 : 1;
}
