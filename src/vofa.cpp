#include "vofa.hpp"
#include "app_uart.h"
#include "bsp_ble.h"

static void VofaTransmit(uint8_t* data, uint16_t data_size)
{
    /** 使用串口发送 */
    app_uart_tx(data, data_size);
    /** 使用蓝牙发送 */
    // bt_nus_send(data, data_size);
}

Vofa::Vofa()
{
    ptr_vofa_data_ = (float *)vofa_data_;
}

void Vofa::SendOneFrame(uint16_t float_size)
{
    if (float_size <= VOFA_MAX_FLOAT_SIZE)
    {
        uint16_t count = 4 * float_size;
        /* JUST FLOAT格式 帧尾 */
        vofa_data_[count++] = 0x00;
        vofa_data_[count++] = 0x00;
        vofa_data_[count++] = 0x80;
        vofa_data_[count++] = 0x7f;
        VofaTransmit((uint8_t *)vofa_data_, count);
    }
}

Vofa *CallVofaCreate(void)
{
    return new Vofa;
}

void CallVofaDestroy(Vofa **pptr_vofa)
{
    if (pptr_vofa != nullptr && *pptr_vofa != nullptr)
    {
        delete *pptr_vofa;
        *pptr_vofa = nullptr;
    }
}

void CallVofaSendOneFrame(Vofa *ptr_vofa, uint16_t float_size)
{
    if (ptr_vofa != nullptr)
    {
        ptr_vofa->SendOneFrame(float_size);
    }
}



