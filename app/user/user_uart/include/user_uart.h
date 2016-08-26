/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */ 

#ifndef __USER_UART_H__
#define __USER_UART_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "../../../include/user_config.h"


#define CUS_UART0_QUEUE_LENGTH (10)
#define CUS_UART_TX_MAX     (128)  // uart tx buf max len


#if USER_UART_CTRL_DEV_EN

#define SERIAL_SOF					0x7E
#define ENCRYPT_MODE				0

/* cmd status */
#define CMD_SUCCESS					0
#define CMD_FAIL					1
#define CMD_INVALID_HEAD			1
#define CMD_CRC_ERROR				2

/* cmd id */
#define CUSTOMIZE_CMD_CHANGE_UART_CFG			0X01
#define CUSTOMIZE_CMD_CHANGE_UART_CFG_RESP		0X81
#define CUSTOMIZE_CMD_FACTORY_RESET				0X02
#define CUSTOMIZE_CMD_DATA_UPLOAD				0X09
#define CUSTOMIZE_CMD_DATA_UPLOAD_RESP			0X89
#define CUSTOMIZE_CMD_DEV_CTRL					0X0A
#define CUSTOMIZE_CMD_DEV_CTRL_RESP				0X8A

#define CMD_WIFI_MODULE_READY		0XF9
#define CMD_WIFI_CLOUD_READY		0XF8


extern xQueueHandle xQueueCusUart;
extern char device_status_change;
extern VIRTUAL_DEV virtual_device;

typedef struct
{
	int rx_len;
	char rx_buf[CUS_UART_TX_MAX];
}CusUartIntrPtr;

typedef struct _serial_cmd_t
{
	uint8 waiting_resp;
	uint8 resp_status;
}serial_cmd_t;

void debug_print_hex_data(char*buf, int len);
void ICACHE_FLASH_ATTR cus_wifi_handler_alinkdata2mcu(u8 dat_index, int dat_value);
void ICACHE_FLASH_ATTR user_uart_task(void *pvParameters);
void ICACHE_FLASH_ATTR user_uart_dev_start(void);
void ICACHE_FLASH_ATTR user_key_short_press(void);

#endif

#endif

