#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "serial.h"

#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/***************************************************************************************
 *  Function: SerialInit
 *
 *  Setup serial port for user interface.
 *
 *  Returns:
 *  - Void
 *  Parameters:
 *  - Void
 *****************************************************************************************/
void SerialInit()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_2, GPIO_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0));
}


