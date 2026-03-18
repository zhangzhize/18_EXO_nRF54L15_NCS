#include "bsp_led.h"
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void bsp_led_init(void)
{
    int ret;

    if (!device_is_ready(led.port))
    {
        return;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return;
    }

    bsp_led_on();
    k_sleep(K_MSEC(1000));
    bsp_led_off();
}

void bsp_led_toggle(void)
{
    if (!device_is_ready(led.port))
    {
        return;
    }

    gpio_pin_toggle_dt(&led);
}

void bsp_led_on(void)
{
    if (!device_is_ready(led.port))
    {
        return;
    }
    gpio_pin_set_dt(&led, 1);
}

void bsp_led_off(void)
{
    if (!device_is_ready(led.port))
    {
        return;
    }
    gpio_pin_set_dt(&led, 0);
}