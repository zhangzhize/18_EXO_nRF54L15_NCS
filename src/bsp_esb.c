#if defined(CONFIG_ESB) && (CONFIG_ESB == 1)

#include <esb.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif
#if NRF54L_ERRATA_20_PRESENT
#include <hal/nrf_power.h>
#endif /* NRF54L_ERRATA_20_PRESENT */
#if defined(NRF54LM20A_ENGA_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54LM20A_ENGA_XXAA) */

#include "bsp_esb.h"
#include "bsp_led.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bsp_esb, LOG_LEVEL_ERR);

bool ready = true;
struct esb_payload rx_payload;
struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0x01, 0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);

static void event_handler(struct esb_evt const *event)
{
    ready = true;

    static int led_toggle_cnt = 0;
    switch (event->evt_id)
    {
    case ESB_EVENT_TX_SUCCESS:
        LOG_DBG("TX SUCCESS EVENT");
        led_toggle_cnt++;
        if (led_toggle_cnt >= 500)
        {
            led_toggle_cnt = 0;
            bsp_led_toggle();
        }
        break;
    case ESB_EVENT_TX_FAILED:
        LOG_DBG("TX FAILED EVENT");
        break;
    case ESB_EVENT_RX_RECEIVED:
        while (esb_read_rx_payload(&rx_payload) == 0)
        {
#if (THIS_NODE_ID == STM32H7_NODE_ID)
            if (rx_payload.length == 1 + sizeof(foot_sensor_packet_t))
            {
                if (rx_payload.data[0] == LEFT_FOOT_NODE_ID)
                {
                    memcpy(&exo_sensor_data.left_foot, rx_payload.data + 1, sizeof(foot_sensor_packet_t));
                }
                else if (rx_payload.data[0] == RIGHT_FOOT_NODE_ID)
                {
                    memcpy(&exo_sensor_data.right_foot, rx_payload.data + 1, sizeof(foot_sensor_packet_t));
                }
            }

            led_toggle_cnt++;
            if (led_toggle_cnt >= 500)
            {
                led_toggle_cnt = 0;
                bsp_led_toggle();
            }

#endif
        }
        break;
    }
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr)
    {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0)
    {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do
    {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res)
        {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

#if NRF54L_ERRATA_20_PRESENT
    if (nrf54l_errata_20())
    {
        nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
    }
#endif /* NRF54L_ERRATA_20_PRESENT */

#if defined(NRF54LM20A_ENGA_XXAA)
    /* MLTPAN-39 */
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif

    LOG_DBG("HF clock started");
    return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void)
{
    int err;
    int res;
    const struct device *radio_clk_dev =
        DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
    struct onoff_client radio_cli;

    /** Keep radio domain powered all the time to reduce latency. */
    nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

    sys_notify_init_spinwait(&radio_cli.notify);

    err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

    do
    {
        err = sys_notify_fetch_result(&radio_cli.notify, &res);
        if (!err && res)
        {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err == -EAGAIN);

    nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
    nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

    LOG_DBG("HF clock started");
    return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

int esb_initialize(void)
{
    int err;
    /* These are arbitrary default addresses. In end user products
     * different addresses should be used for each set of devices.
     */
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = 600; /* 600 us */
    config.bitrate = ESB_BITRATE_4MBPS;
    config.event_handler = event_handler;
    /* set device role based on THIS_NODE_ID */
#if THIS_NODE_ID == STM32H7_NODE_ID
    config.mode = ESB_MODE_PRX;
#else
    config.mode = ESB_MODE_PTX;
#endif
    config.selective_auto_ack = true;
    if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING))
    {
        config.use_fast_ramp_up = true;
    }

    err = esb_init(&config);

    if (err)
    {
        return err;
    }

    err = esb_set_base_address_0(base_addr_0);
    if (err)
    {
        return err;
    }

    err = esb_set_base_address_1(base_addr_1);
    if (err)
    {
        return err;
    }

    err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
    if (err)
    {
        return err;
    }

    err = esb_set_rf_channel(RF_CHANNEL);
    if (err)
    {
        return err;
    }

    return 0;
}

#endif /* CONFIG_ESB */
