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

#include "c_types.h"
#include <stdio.h>
#include <string.h>
#include "esp_common.h"
#include "user_uart.h"


#if USER_UART_CTRL_DEV_EN

#include "../../../include/driver/uart.h"

xQueueHandle xQueueCusUart;
serial_cmd_t ctrl_serial_cmd;
CusUartIntrPtr dev_data_from_mcu;
char uart_cfg_change = 0;
char uart_cfg_baud_index;

void debug_print_hex_data(char*buf, int len)
{
	int i = 0;
	printf("\n_____________[%d]__________\n",len);
	for(i=0;i<len;i++)
	{
		printf("%X ",*(buf+i));
	}
	printf("\n____________________________\n");
	return;
}	

static char ICACHE_FLASH_ATTR sum8(uint8 *buffer, uint8 len)
{
	uint8 i;
	uint8 checksum = 0;
	for(i = 0; i < len; i++)
	{
		checksum += buffer[i];
	}
	return (checksum);
}

int uart0_write_data(u8 *data, int len)
{
	int re_len = 0;
	int i = 0;
	for(i  = 0; i <len; i++)
	{
		uart0_write_char(*(data+i));
	}
	return i;
}
uint16 form_serial_packet(uint8 cmdid, uint8 *data, uint8 datalen, uint8 *buf)
{
	uint8 *p = buf;

	*p++ = SERIAL_SOF;
	*p++ = ENCRYPT_MODE;
	*p++ = datalen + 1;
	*p++ = cmdid;
	if((data != NULL) && (datalen > 0)) {
		memcpy(p, data, datalen);
		p = p + datalen;
	}
	*p = sum8(buf, (p - buf));
	p++;
	debug_print_hex_data(buf, p-buf);
	return (p - buf);
}

serial_cmd_t* ICACHE_FLASH_ATTR issue_ctrl_cmd(uint8 *buf, uint8 len)
{
	uint16 pkt_len;
	uint8 serial_pkt[CUS_UART_TX_MAX];
	memset(serial_pkt, 0, sizeof(serial_pkt));
	pkt_len = form_serial_packet(CUSTOMIZE_CMD_DEV_CTRL, buf, len, serial_pkt);

	debug_print_hex_data(serial_pkt, pkt_len);
	uart0_write_data(serial_pkt, pkt_len);
	ctrl_serial_cmd.waiting_resp = 1;
	return &ctrl_serial_cmd;
}

int ICACHE_FLASH_ATTR do_ctrl(void *command, uint8 len)
{
	serial_cmd_t *cur_cmd = NULL;
	uint8 timeout = 0;
	if(len == 0) {
		return -CMD_FAIL;
	}
	cur_cmd = issue_ctrl_cmd(command, len);
	if(cur_cmd != NULL) {
		while(cur_cmd->waiting_resp == 1) {
			os_delay_us(2000); //delay 2 OS_TICK,20ms
			timeout++;
			if(timeout > 30) { //wait 600ms
				ESP_DBG(("Wait serial response timeout.\n"));
				cur_cmd->waiting_resp = 0;
				cur_cmd->resp_status = 1;
				return -CMD_FAIL;
			}
		}
	}
	memcpy((uint8*)command, dev_data_from_mcu.rx_buf, dev_data_from_mcu.rx_len);

	return CMD_SUCCESS;
}

void ICACHE_FLASH_ATTR serial_resp_out(uint8 resp_id, uint8 status)
{
	uint8 buf[64];
	uint8 len;
	uint8 left;
	uint8 *pos = NULL;
	uint8 *p = &buf[0];

	*p++ = SERIAL_SOF;
	*p++ = ENCRYPT_MODE;
	*p++ = 2;
	*p++ = resp_id;
	*p++ = status;
	*p = sum8(&buf[0], p - &buf[0]);
	p++;
	uart0_write_data(&buf[0], p - &buf[0]);
}

void ICACHE_FLASH_ATTR cus_wifi_handler_alinkdata2mcu(u8 dat_index, int dat_value)
{
	ESP_DBG(("data2mcu handler, index[%x],data_value[%x]",dat_index,dat_value));
	// here handler user own uart protocol...
	uint8 command[2];
	command[0] = dat_index;
	command[1] = dat_value;
	do_ctrl(command, 2);	
	return;
}
static u8 ICACHE_FLASH_ATTR execute_serial_cmd(uint8 cmdid, uint8 *data, uint8 datalen)
{
	ESP_DBG(("execute_serial_cmd=%d\r\n", cmdid));
	switch(cmdid)
	{
		case CUSTOMIZE_CMD_FACTORY_RESET:
			user_key_short_press();
			break;
		case CUSTOMIZE_CMD_DEV_CTRL_RESP:
			{
				uint8 command[2];
				ESP_DBG(("cmd ctrl resp\r\n"));
				if (ctrl_serial_cmd.waiting_resp == 1)
					ctrl_serial_cmd.waiting_resp = 0;
				memcpy(command, data, datalen);
				if (command[0] == 0)
				{
					ESP_DBG(("This is command for power\r\n"));
					virtual_device.power = command[1];
				}
			}
			break;
		case CUSTOMIZE_CMD_CHANGE_UART_CFG:
			{
				uint8 command[2];
				ESP_DBG(("cmd change uart cfg\r\n"));
				if (datalen == 2)
				{
					memcpy(command, data, datalen);
				}
				serial_resp_out(CUSTOMIZE_CMD_CHANGE_UART_CFG_RESP, CMD_SUCCESS);

				uart_cfg_baud_index = command[0];
				uart_cfg_change = 1;
				 vTaskDelay(100);
			}
			break;
		case CUSTOMIZE_CMD_DATA_UPLOAD:
			{	
				ESP_DBG(("cmd data upload\r\n"));
				device_status_change = 1;
				serial_resp_out(CUSTOMIZE_CMD_DATA_UPLOAD_RESP, CMD_SUCCESS);
			}
			break;
		default:
			break;

	}
}
static u8 ICACHE_FLASH_ATTR cus_uart_data_handle(char *dat_in, int in_len, char *dat_out)
{
	char *p = dat_in;
	char *data;
	u8 resp = CMD_SUCCESS;
	uint8 len, crc, cmdid;
	ESP_DBG(("uart data handler.."));
	while(in_len > 3)
	{
		if(*p != SERIAL_SOF)
		{
			resp = CMD_INVALID_HEAD;
			ESP_DBG(("header invalid\r\n"));
			return;
		}
		len = *(p + 1 + 1) + 3;
		crc = sum8(p, len);
		if(*(p + len) != crc) {
			resp = CMD_CRC_ERROR;
			//serial_resp_out(CMD_PACKET_ERROR_RESP, CMD_CRC_ERROR);
			ESP_DBG(("crc invalid\r\n"));
			return;
		}
		cmdid = *(p + 3);
		data = p + 4;
		len = *(p + 1 + 1) - 1;
		execute_serial_cmd(cmdid, data, len);
		//in case two packet coming together
		len = *(p + 1 + 1) + 4;
		p = p + len;
		in_len = in_len - len;
	}

	return 0x00;
}

void ICACHE_FLASH_ATTR user_uart_task(void *pvParameters)
{

	u32 sys_time_value = system_get_time();
	char uart_beat_data[]={0x7e, 0x0, 0x02, 0xFA, 0x0, 0x7A};

	vTaskDelay(100);
	serial_resp_out(CMD_WIFI_MODULE_READY,CMD_SUCCESS);

	while(1)
	{
		if (xQueueReceive(xQueueCusUart, (void *)&dev_data_from_mcu, (portTickType)500/*portMAX_DELAY*/)) // wait about 5sec 
		{
			ESP_DBG(("data uart recv.."));
			debug_print_hex_data(dev_data_from_mcu.rx_buf,dev_data_from_mcu.rx_len);

			if(dev_data_from_mcu.rx_len>0x00){
				cus_uart_data_handle(dev_data_from_mcu.rx_buf, dev_data_from_mcu.rx_len,NULL);
			}
		}
		if((system_get_time()-sys_time_value)>=(60*1000*1000))  //about 1min, send data to uart0, demo beat data
		{
			ESP_DBG(("uart beat data.[%d][%d]",sys_time_value,system_get_time()));
			ESP_DBG(("heap_size %d\n", system_get_free_heap_size()));
			uart0_write_data(uart_beat_data,sizeof(uart_beat_data));
			sys_time_value = system_get_time();
		}

		if(uart_cfg_change == 1)
		{
			uart_cfg_change = 0;
			if(uart_cfg_baud_index == 1)
			{
				ESP_DBG(("UART baud to B19200\r\n"));
				UART_ConfigTypeDef uart_config;
				uart_config.baud_rate    = BIT_RATE_19200;
				uart_config.data_bits     = UART_WordLength_8b;
				uart_config.parity          = USART_Parity_None;
				uart_config.stop_bits     = USART_StopBits_1;
				uart_config.flow_ctrl      = USART_HardwareFlowControl_None;
				uart_config.UART_RxFlowThresh = 120;
				uart_config.UART_InverseMask = UART_None_Inverse;
				UART_ParamConfig(UART0, &uart_config);

			}
		}
	}

	vTaskDelete(NULL);

}

void ICACHE_FLASH_ATTR user_uart_dev_start(void)
{
	ESP_DBG(("Before uart init......\r\n"));
	uart_init_new();   // cfg uart0 connection device MCU, cfg uart1 TX debug output
	ESP_DBG(("after uart init......\r\n"));
	memset((uint8*)&ctrl_serial_cmd, 0, sizeof(serial_cmd_t));

	xQueueCusUart = xQueueCreate((unsigned portBASE_TYPE)CUS_UART0_QUEUE_LENGTH, sizeof(CusUartIntrPtr));
	xTaskCreate(user_uart_task, (uint8 const *)"uart", 256, NULL, tskIDLE_PRIORITY + 2, NULL);

	return;
}

#endif
