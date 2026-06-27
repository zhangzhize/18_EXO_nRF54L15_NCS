#include <zephyr/kernel.h>

#if CONFIG_BT

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "bsp_ble.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bsp_ble, LOG_LEVEL_INF);

int bsp_ble_init(void)
{
  int err = bt_enable(NULL);

  if (err)
  {
    LOG_ERR("Failed to enable Bluetooth (err: %d)", err);
    return -1;
  }

  return 0;
}

#endif /* CONFIG_BT */
