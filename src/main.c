#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include "ardupilotmega/mavlink.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>

#include <nrfx_uarte.h>
#include <hal/nrf_gpio.h>

#define UARTE_INST  1
#define TX_PIN  6
#define RX_PIN  7

#define UART_DEVICE_NODE DT_NODELABEL(uart1)
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


#define SYSID 1 
#define COMPID MAV_COMP_ID_MISSIONPLANNER
#define TARGET_SYS  1 
#define TARGET_COMP 1


static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	
	case UART_TX_DONE:
		// do something
        //printk("UART_TX_DONE\n");
		break;

	case UART_TX_ABORTED:
		// do something
        printk("UART_TX_ABORTED\n");
		break;
		
	case UART_RX_RDY:
		// do something
		break;

	case UART_RX_BUF_REQUEST:
		// do something
		break;

	case UART_RX_BUF_RELEASED:
		// do something
		break;
		
	case UART_RX_DISABLED:
		// do something
		break;

	case UART_RX_STOPPED:
		// do something
		break;
		
	default:
		break;
	}
}

void Send(const uint8_t *buf, size_t len)
{
    
    if (uart_tx(uart_dev, buf, len, SYS_FOREVER_MS) != 0) {
        printk("uart_tx failed\n");
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
    //printk("%d \n", buf);
    printk("Sending Heartbeat\n");
}

int main(void)
{
    
    printk("nRF5340 blinky running\n");
    if (!device_is_ready(uart_dev)) {
        return;
    }
    int err;
    err = uart_callback_set(uart_dev, uart_cb, NULL);
    if (err) {
        return err;
    }
    while (1) {
        SendHeartbeat();
        k_msleep(50);
    }
}

