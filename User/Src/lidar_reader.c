# include "lidar_reader.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>
# include <math.h>
# include <memory.h>


volatile uint8_t lidar_rx_buffer_dma_target[];
volatile uint8_t lidar_rx_buffer[];
volatile int lidar_rx_buffer_busy = 0;
volatile int lidar_rx_buffer_new = 0;

unsigned int points_buffer_index = 0;
struct Point2D lidar_points_buffer[];
unsigned int current_points_end = 0;
struct Point2D lidar_points_current[];

// the lidar angle at the start of the current 360 scan
float measurement_initial_angle;
// 1 if a 360 measurement is ongoing, 0 if a new must be started at the next processed frame.
int measurement_ongoing = 0;

// if this switches from 1 to 0, then the current 360 measurement is finished.
int measurement_initial_angle_comp = 0;

// the index of the nearest point in lidar_points_current
unsigned int current_nearest_point = 0;
// the index of the nearest point in lidar_points_buffer
unsigned int buffer_nearest_point = 0;



void lidar_dump_points()
{
	// __disable_irq();
	printf("\r\nx y\r\n");
	
	for (unsigned int i = 0; i < current_points_end; i++)
	{
		// convert to position

		float pos_x = cosf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		float pos_y = sinf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		// float pos_x = lidar_points_buffer[i].x;
		// float pos_y = lidar_points_buffer[i].y;

		printf("%.3f %.3f\r\n", pos_x, pos_y);
	}
	// __enable_irq();
}

void lidar_compare_with_initial_angle(float current_angle)
{
	measurement_initial_angle_comp = coord_get_delta_angle_deg(measurement_initial_angle, current_angle) < 0;
}

/**
 * Called at the end of a 360 measurement.
 * @note When this is called, lidar_points_current, current_points_end, and current_nearest_point are freshly set with new values.
 */
void lidar_process_360_points()
{
	printf("nearest obstacle is at an angle of %.3f", lidar_points_current[current_nearest_point].x);
}

/**
 * @param header_index : the index in lidar_rx_buffer of the dataframe to read.
 */
int lidar_process_frame(unsigned int header_index)
{
	// The rotation speed of the lidar
	// uint16_t speed = lidar_rx_buffer[header_index+2] | (lidar_rx_buffer[header_index+3] << 8);

	// need to divide the value by 100 to get degrees
	printf("received angle value start Ox%2X Ox%2X and end Ox%2X Ox%2X\r\n", lidar_rx_buffer[header_index+4], lidar_rx_buffer[header_index+5], lidar_rx_buffer[header_index + LIDAR_PACKET_SIZE - 5], lidar_rx_buffer[header_index + LIDAR_PACKET_SIZE - 4]);
	float start_angle = (float) (((unsigned int)lidar_rx_buffer[header_index+4]) | (unsigned int)(lidar_rx_buffer[header_index+5] << 8)) / 100.0f;
	float end_angle = (float) (((unsigned int)lidar_rx_buffer[header_index + LIDAR_PACKET_SIZE - 5]) | (unsigned int)(lidar_rx_buffer[header_index + LIDAR_PACKET_SIZE - 4] << 8)) / 100.0f;
	printf("the angles are %.3f start and %.3f end\r\n", start_angle, end_angle);

	if (measurement_ongoing == 0)
	{
		// start a new 360 measurement

		printf("started new measurement\r\n");
		measurement_initial_angle = start_angle - 0.1f; // small margin for loop detection
		measurement_ongoing = 1;
		buffer_nearest_point = 0;
	}
	
	// avoids the end angle being smaller than the start angle, which would make the step angle negative
	if (end_angle < start_angle) end_angle += 360.0f;
	
	float step_angle = (float)(end_angle - start_angle) / LIDAR_MEASUREMENT_AMOUNT;

	unsigned int mesurement_start = header_index + 6;

	for (unsigned int i = 0; i < LIDAR_MEASUREMENT_AMOUNT; i++)
	{
		// the measured distance
		int distance = lidar_rx_buffer[header_index + mesurement_start + i*3] | (lidar_rx_buffer[header_index + mesurement_start + i*3 + 1] << 8);
		
		// the angle at which the lidar was when it took this measurement
		float current_angle = start_angle + step_angle * i;

		// check if we completed a full loop since the start of the measurement
		int last_comp_val = measurement_initial_angle_comp;
		lidar_compare_with_initial_angle(current_angle);

		// printf("comp : %i\r\n", measurement_initial_angle_comp);
		// printf("angle : %.3f\r\n", current_angle);
		// printf("points buffer index : %u\r\n", points_buffer_index);

		// checks if we are just now completing a full revolution since the start of the measurement
		if (last_comp_val == 1 && measurement_initial_angle_comp == 0)
		{
			// saves the 360 measurement's point and process it before processing the other points
			// memcpy(lidar_points_current, lidar_points_buffer, LIDAR_POINTS_BUFFER_SIZE);
			current_points_end = points_buffer_index;
			current_nearest_point = buffer_nearest_point;
			printf("point buffer index and end : %u and %u", points_buffer_index, current_points_end);

			measurement_ongoing = 0;
			
			lidar_process_360_points();
			lidar_dump_points();

			points_buffer_index = 0;

			return 1;
		}
			

		struct Point2D result;
		// result.x = cosf(current_angle / 180 * M_PI) * distance;
		// result.y = sinf(current_angle / 180 * M_PI) * distance;
		result.x = current_angle;
		result.y = distance;

		if (result.x < 0.01f && result.x > -0.01f) printf("got a zero\r\n");

		lidar_points_buffer[points_buffer_index] = result;
		// printf("added point %.3f %.3f", result.x, result.y);

		if (result.y < lidar_points_buffer[buffer_nearest_point].y) buffer_nearest_point = points_buffer_index;
		
		if (points_buffer_index < LIDAR_POINTS_BUFFER_SIZE)
		{
			points_buffer_index ++;
		}
		else
		{
			printf("WARN : reached end of point buffer");
		}
	}

	return 0;
}



/**
 * 
 */
void lidar_process_buffer()
{
	lidar_rx_buffer_busy = 1;

	// stops searching for headers early enough
	unsigned int end = LIDAR_RX_BUFFER_SIZE-1-LIDAR_PACKET_SIZE;

	for (unsigned int i = 0; i < end; i++)
	{
		if (lidar_rx_buffer[i] == LIDAR_HEADER_BYTE)
		{
			printf("processing packet at %i\r\n", i);
			if (lidar_process_frame(i))
			{
				break;
			}
		}
	}

	lidar_rx_buffer_busy = 0;
}


