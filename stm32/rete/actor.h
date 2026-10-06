#ifndef ACTOR_H
#define ACTOR_H
/* Calcola l'azione deterministica della policy SAC: obs[7] -> azione in [-1, 1] */
float actor_forward(const float *obs);
#endif
