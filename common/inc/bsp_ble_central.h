#ifndef BSP_BLE_CENTRAL_H
#define BSP_BLE_CENTRAL_H

#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/uuid.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** Generic callback for received sensor data (raw bytes) */
typedef void (*ble_central_data_cb_t)(const bt_addr_le_t *addr,
                                      const uint8_t *data,
                                      size_t len);

/**
 * @brief Device profile — one per sensor type
 */
struct ble_device_profile
{
  const bt_addr_le_t *macs; /* known MAC addresses */
  uint8_t mac_count; /* number of MACs */
  const struct bt_uuid *svc_uuid; /* GATT service UUID */
  const struct bt_uuid *chrc_uuid; /* notify characteristic UUID */
  const struct bt_uuid *write_chrc_uuid; /* optional: write characteristic UUID */
  ble_central_data_cb_t data_cb; /* raw data callback */
  void (*post_subscribe_cb)(struct bt_conn *conn, uint16_t write_handle); /* called once */
};

#if CONFIG_BT && CONFIG_BT_CENTRAL

/**
 * @brief Init BLE Central and start scanning.
 *
 * Call once after bsp_ble_init(). Scans for devices matching previously
 * registered profiles, auto-connects, discovers GATT, and subscribes.
 *
 * @param profiles   Array of device profiles.
 * @param count       Number of profiles.
 * @return 0 on success.
 */
int bsp_ble_central_start(const struct ble_device_profile *profiles,
                          uint8_t count);

/**
 * @brief Check whether all target sensors are connected and subscribed.
 * @return true if safe to start streaming data.
 */
bool bsp_ble_central_all_subscribed(void);

/**
 * @brief Check whether all target sensors are subscribed and using the
 *        requested high-throughput BLE parameters.
 *
 * This requires 2M PHY, data length extension, and the configured ATT MTU.
 *
 * @return true if all target links are ready for high-rate sensor streaming.
 */
bool bsp_ble_central_all_links_ready(void);

#else /* CONFIG_BT && CONFIG_BT_CENTRAL */

static inline int bsp_ble_central_start(const struct ble_device_profile *profiles,
                                        uint8_t count)
{
  (void)profiles;
  (void)count;
  return -ENOTSUP;
}

static inline bool bsp_ble_central_all_subscribed(void)
{
  return false;
}

static inline bool bsp_ble_central_all_links_ready(void)
{
  return false;
}

#endif /* CONFIG_BT && CONFIG_BT_CENTRAL */

#ifdef __cplusplus
}
#endif

#endif /* BSP_BLE_CENTRAL_H */
