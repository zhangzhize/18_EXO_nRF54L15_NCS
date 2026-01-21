#ifndef BSP_ESB_H
#define BSP_ESB_H

#include "main.h"
#include "stdint.h"
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_ESB) && (CONFIG_ESB == 1)

#include <esb.h>

#ifndef RF_CHANNEL
#define RF_CHANNEL 20
#endif

#define _RADIO_SHORTS_COMMON                                       \
    (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
     RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                          \
     RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

extern bool ready;
extern struct esb_payload rx_payload;
extern struct esb_payload tx_payload;

enum
{
    kFoot_ADC = 0x01,
    kFoot_ADC_IMU = 0x02,
};

struct sensor_data_packet_t
{
    uint8_t data_type;
    uint8_t left_foot_adc_data[8];
    uint8_t right_foot_adc_data[8];
    uint8_t left_foot_imu_data[12];
    uint8_t right_foot_imu_data[12];
};
BUILD_ASSERT(sizeof(struct sensor_data_packet_t) == 41, "sensor_data_packet_t must be 41 bytes");

extern struct sensor_data_packet_t sensor_data;

int clocks_start(void);
int esb_initialize(void);

#else /* CONFIG_ESB */

/*
 * ESB 未启用时：
 * - 不包含 <esb.h>，避免头文件缺失/类型缺失导致编译失败。
 * - 提供 stub，使上层代码可以更灵活地“同一套源码，不同配置”。
 */

#ifndef RF_CHANNEL
#define RF_CHANNEL 20
#endif

/* 占位类型，避免上层引用 struct esb_payload 时无法编译。
 * 注意：ESB 关闭时该类型不可用于真实收发。
 */
struct esb_payload {
	uint8_t _unused;
};

/* 与 ESB 版本保持一致的 symbol（用于链接期满足引用）。 */
static inline int clocks_start(void)
{
	return -ENOTSUP;
}

static inline int esb_initialize(void)
{
	return -ENOTSUP;
}

#endif /* CONFIG_ESB */

#ifdef __cplusplus
}
#endif

#endif /* BSP_ESB_H */
