#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "ardupilotmega/mavlink.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
#define SYSID 1 
#define COMPID MAV_COMP_ID_MISSIONPLANNER
#define TARGET_SYS  1 //target system ID for MAVLink messages(i.e the autopilot)
#define TARGET_COMP 1 // target component ID for MAVLink messages(i.e the autopilot)


static void tx_burst(void) {
    if (!device_is_ready(uart_dev)) {
        printk("uart1 not ready\n");
        return;
    }
    for (int i = 0; i < 500; ++i) {
        uart_poll_out(uart_dev, 0x55);
    }
}

void Send(const uint8_t *buf, size_t len)
{
    if (!device_is_ready(uart_dev)) {
        return; /* Optionally assert/log */
    }
    for (size_t i = 0; i < len; ++i) {
        uart_poll_out(uart_dev, buf[i]);
    }
}

void SendHeartbeat(void) {

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        SYSID,         
        COMPID,           
        &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,
        MAV_AUTOPILOT_INVALID,
        0, 0,
        MAV_STATE_ACTIVE
    );

    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
}

int main(void)
{
    if (!device_is_ready(led.port)) {
        printk("LED device not ready\n");
        return 0;
    }

    if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) != 0) {
        printk("Failed to configure LED pin\n");
        return 0;
    }

    if (!device_is_ready(led1.port)) {
        printk("LED device not ready\n");
        return 0;
    }

    if (gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE) != 0) {
        printk("Failed to configure LED pin\n");
        return 0;
    }
    if (!device_is_ready(led2.port)) {
        printk("LED device not ready\n");
        return 0;
    }

    if (gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE) != 0) {
        printk("Failed to configure LED pin\n");
        return 0;
    }

    if (!device_is_ready(led3.port)) {
        printk("LED device not ready\n");
        return 0;
    }

    if (gpio_pin_configure_dt(&led3, GPIO_OUTPUT_INACTIVE) != 0) {
        printk("Failed to configure LED pin\n");
        return 0;
    }

    printk("nRF5340 blinky running\n");

    while (1) {
        printk("Toggling Red LED\n");
        gpio_pin_toggle_dt(&led);
        k_msleep(500);
        printk("Toggling Blue LED\n");
        gpio_pin_toggle_dt(&led1);
        k_msleep(500);
        printk("Toggling Green LED\n");
        gpio_pin_toggle_dt(&led2);
        k_msleep(500);
        printk("Toggling Green LED\n");
        gpio_pin_toggle_dt(&led3);
        k_msleep(500);
        SendHeartbeat();
        k_msleep(500);
    }
}
