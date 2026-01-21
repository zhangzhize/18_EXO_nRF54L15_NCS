#include "bsp_vofa.h"
#include <string.h>

const uint8_t VOFA_TAIL[VOFA_TAIL_SIZE] = { 0x00, 0x00, 0x80, 0x7F};

uint8_t vofa_buf[VOFA_MAX_CH * sizeof(float) + VOFA_TAIL_SIZE] = {0};
float *ptr_vofa_buf = (float *)vofa_buf;

void SendToVOFA(uint8_t num_floats)
{
    if (num_floats == 0 || num_floats > VOFA_MAX_CH) return;
    uint16_t payload_bytes = num_floats * sizeof(float);

    memcpy(((uint8_t*)ptr_vofa_buf) + payload_bytes, VOFA_TAIL, VOFA_TAIL_SIZE);
    // app_uart_tx((uint8_t*)ptr_vofa_buf, payload_bytes + VOFA_TAIL_SIZE);
}