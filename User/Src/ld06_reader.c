# include "ld06_reader.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>


uint8_t ld06_rx_buffer[];



void ld06_process_buffer()
{
	printf("processing buffer\r\n");
	if (ld06_rx_buffer[0] == LD06_HEADER_BYTE)
	{
		printf("header correctly received\r\n");
	}
}