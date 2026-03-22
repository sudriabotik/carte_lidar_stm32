# include "ld06_reader.h"


uint8_t ld06_rx_buffer[];



void ld06_process_buffer()
{
	if (ld06_rx_buffer[0] == LD06_HEADER_BYTE)
	{
		printf("header correctly received");
	}
}