# include "ld06_reader.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>


uint8_t ld06_rx_buffer[];



void ld06_process_buffer()
{
	/**
	 * tries to find a sequence of header byte + packet length.
	 * since both of these are fixed, it should be a decent indicator to find the start of a frame.
	 */

	printf("packet reception\r\n");
	int i;
	for (i = 0; i < LD06_RX_BUFFER_SIZE-1; i++)
	{
		if (ld06_rx_buffer[i] == LD06_HEADER_BYTE)
		{
			printf("header found at %i\r\n", i);
		}
	}
}