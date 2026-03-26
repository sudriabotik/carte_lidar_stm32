# include "ld06_reader.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>
# include <math.h>


uint8_t ld06_rx_buffer[];

unsigned int points_buffer_index = 0;
struct Point2D ld06_points_buffer[];

void ld06_dump_points()
{
	printf("\r\nx y\r\n");
	for (int i = 0; i < LD06_POINTS_BUFFER_SIZE; i++) printf("%.3f %.3f\r\n", ld06_points_buffer[i].x, ld06_points_buffer[i].y);
}

void ld06_process_buffer()
{
	/**
	 * tries to find a sequence of header byte + packet length.
	 * since both of these are fixed, it should be a decent indicator to find the start of a frame.
	 */

	// printf("packet reception\r\n");
	int start = -1;
	for (start = 0; start < LD06_RX_BUFFER_SIZE-1-LD06_PACKET_SIZE; start++)
	{
		if (ld06_rx_buffer[start] == LD06_HEADER_BYTE)
		{
			// printf("header found at %i\r\n", start);
			// for (int y = 0; y < LD06_PACKET_SIZE; y++) printf("%#02X ", ld06_rx_buffer[start + y]);
			// printf("\r\n");
			break;
		}
	}

	if (start == -1) return;

	uint16_t packet_lenght = ld06_rx_buffer[start+1];
	uint16_t speed = ld06_rx_buffer[start+2] | (ld06_rx_buffer[start+3] << 8);

	uint16_t measurement_amount = packet_lenght - 10;

	float start_angle = (float) (ld06_rx_buffer[start+4] | (ld06_rx_buffer[start+5] << 8)) / 100.0f;
	float end_angle = (float) (ld06_rx_buffer[start + LD06_PACKET_SIZE - 5] | (ld06_rx_buffer[start + LD06_PACKET_SIZE - 4] << 8)) / 100.0f;
	
	if (end_angle < start_angle) end_angle += 360.0f;
	
	float step_angle = (float)(end_angle - start_angle) / measurement_amount;

	// printf("angle from %f to %f, step is %f\r\n", start_angle, end_angle, step_angle);

	uint16_t mesurement_start = start + 6;

	
	for (int i = mesurement_start; i < LD06_MEASUREMENT_AMOUNT * 3; i+=3)
	{
		int length = ld06_rx_buffer[mesurement_start + i] | (ld06_rx_buffer[mesurement_start + i + 1] << 8);
		struct Point2D result;
		float current_angle = start_angle + step_angle * i;
		result.x = cosf(current_angle / 180 * M_PI) * length;
		result.y = sinf(current_angle / 180 * M_PI) * length;

		ld06_points_buffer[points_buffer_index] = result;
		points_buffer_index ++;
		if (points_buffer_index >= LD06_POINTS_BUFFER_SIZE)
		{
			ld06_dump_points();
			points_buffer_index = 0;
		}
	}
	
}


