#ifndef VOFA_H
#define VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define VOFA_TAIL_SIZE 4
#define VOFA_MAX_CH    64

extern uint8_t vofa_buf[VOFA_MAX_CH * sizeof(float) + VOFA_TAIL_SIZE];
extern float *ptr_vofa_buf;

void SendToVOFA(uint8_t num_floats);


#ifdef __cplusplus
}
#endif

#endif // VOFA_H