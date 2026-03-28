# include "lidar_reader.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>
# include <math.h>


uint8_t lidar_rx_buffer_dma_target[];
uint8_t lidar_rx_buffer[];

unsigned int points_buffer_index = 0;
struct Point2D lidar_points_buffer[];
struct Point2D lidar_current_points[];

// the lidar angle at the start of the current 360 scan
float measurement_initial_angle;
// 1 if a 360 measurement is ongoing, 0 if a new must be started at the next processed frame.
int measurement_ongoing;



void lidar_dump_points()
{
	printf("\r\nx y\r\n");
	for (int i = 0; i < LIDAR_POINTS_BUFFER_SIZE; i++) printf("%.3f %.3f\r\n", lidar_points_buffer[i].x, lidar_points_buffer[i].y);
}



/**
 * @param header_index : the index in lidar_rx_buffer of the dataframe to read.
 */
void lidar_process_frame(int header_index)
{
	// The rotation speed of the lidar
	uint16_t speed = lidar_rx_buffer[header_index+2] | (lidar_rx_buffer[header_index+3] << 8);

	// need to divide the value by 100 to get degrees
	float start_angle = (float) (lidar_rx_buffer[header_index+4] | (lidar_rx_buffer[header_index+5] << 8)) / 100.0f;
	float end_angle = (float) (lidar_rx_buffer[header_index + LIDAR_PACKET_SIZE - 5] | (lidar_rx_buffer[header_index + LIDAR_PACKET_SIZE - 4] << 8)) / 100.0f;

	if (measurement_ongoing == 0)
	{
		// start a new 360 measurement
		measurement_initial_angle = start_angle - 0.1f; // small margin for loop detection
		measurement_ongoing = 1;
	}
	
	// avoids the end angle being smaller than the start angle, which would make the step angle negative
	if (end_angle < start_angle) end_angle += 360.0f;
	
	float step_angle = (float)(end_angle - start_angle) / LIDAR_MEASUREMENT_AMOUNT;

	uint16_t mesurement_start = header_index + 6;
	for (int i = mesurement_start; i < LIDAR_MEASUREMENT_AMOUNT; i++)
	{
		// the measured distance
		int distance = lidar_rx_buffer[mesurement_start + i*3] | (lidar_rx_buffer[mesurement_start + i*3 + 1] << 8);
		
		// the angle at which the lidar was when it took this measurement
		float current_angle = start_angle + step_angle * i;

		struct Point2D result;
		result.x = cosf(current_angle / 180 * M_PI) * distance;
		result.y = sinf(current_angle / 180 * M_PI) * distance;

		lidar_points_buffer[points_buffer_index] = result;
		points_buffer_index ++;

		if (points_buffer_index >= LIDAR_POINTS_BUFFER_SIZE)
		{
			lidar_dump_points();
			points_buffer_index = 0;
		}
	}
}



/**
 * 
 */
void lidar_process_buffer()
{
	// printf("packet reception\r\n");

	// stops searching for headers early enough
	int end = LIDAR_RX_BUFFER_SIZE-1-LIDAR_PACKET_SIZE;

	for (int i = 0; i < LIDAR_RX_BUFFER_SIZE-1-LIDAR_PACKET_SIZE; i++)
	{
		if (lidar_rx_buffer[i] == LIDAR_HEADER_BYTE)
		{
			lidar_process_frame(i);
		}
	}
}


