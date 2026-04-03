# include "lidar_reader.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>
# include <math.h>
# include <memory.h>

# include <lipkg_copy.h>

volatile uint8_t lidar_rx_buffer_1[];
volatile uint8_t lidar_rx_buffer_2[];
volatile uint8_t* lidar_rx_buffer_dma_pointer;
volatile int lidar_rx_buffer_busy = 0;
volatile int lidar_rx_buffer_new = 0;

unsigned int points_buffer_index = 0;
struct Point2D lidar_points_buffer[];



// the lidar angle at the start of the current 360 scan
float measurement_initial_angle;
// 1 if a 360 measurement is ongoing, 0 if a new must be started at the next processed frame.
int measurement_ongoing = 0;
// if this switches from 1 to 0, then the current 360 measurement is finished.
int measurement_initial_angle_comp = 0;


// the index of the nearest point in lidar_points_buffer
unsigned int buffer_nearest_point = 0;



void lidar_dump_points()
{
	// __disable_irq();
	printf("\r\nx y\r\n");
	
	for (unsigned int i = 0; i < points_buffer_index; i++)
	{
		// convert to position

		// float pos_x = cosf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		// float pos_y = sinf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		float pos_x = lidar_points_buffer[i].x;
		float pos_y = lidar_points_buffer[i].y;

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
	// printf("nearest obstacle is at an angle of %.3f", lidar_points_current[buffer_nearest_point].x);
}

/**
 * @param header_index : the index in lidar_rx_buffer of the dataframe to read.
 */
int lidar_process_frame(unsigned int header_index, volatile uint8_t* buffer)
{
	// The rotation speed of the lidar
	// uint16_t speed = lidar_rx_buffer[header_index+2] | (lidar_rx_buffer[header_index+3] << 8);

	// need to divide the value by 100 to get degrees
	/* printf("received angle value start Ox%2X Ox%2X and end Ox%2X Ox%2X\r\n",
			buffer[header_index+4], buffer[header_index+5], buffer[header_index + LIDAR_PACKET_SIZE - 5], buffer[header_index + LIDAR_PACKET_SIZE - 4]); */
	
	float start_angle = (float) (((unsigned int)buffer[header_index+4]) | (unsigned int)(buffer[header_index+5] << 8)) / 100.0f;
	float end_angle = (float) (((unsigned int)buffer[header_index + LIDAR_PACKET_SIZE - 5]) | (unsigned int)(buffer[header_index + LIDAR_PACKET_SIZE - 4] << 8)) / 100.0f;

	// printf("the angles are %.3f start and %.3f end\r\n", start_angle, end_angle);

	// avoids the end angle being smaller than the start angle, which would make the step angle negative
	if (end_angle < start_angle) end_angle += 360.0f;

	if (measurement_ongoing == 0)
	{
		// start a new 360 measurement
		// printf("started new measurement\r\n");
		measurement_initial_angle = start_angle - 0.1f; // small margin for loop detection
		measurement_ongoing = 1;
		buffer_nearest_point = 0; // reset the index of the nearest point
	}
	
	float step_angle = (float)(end_angle - start_angle) / LIDAR_MEASUREMENT_AMOUNT;

	unsigned int mesurement_start = header_index + 6;

	for (unsigned int i = 0; i < LIDAR_MEASUREMENT_AMOUNT; i++)
	{
		// the measured distance
		unsigned int distance = (unsigned int)(buffer[header_index + mesurement_start + i*3 + 0]) | (unsigned int)(buffer[header_index + mesurement_start + i*3 + 1] << 8);

		// printf("the values of the measurement : Ox%2X Ox%2X Ox%2X\r\n", buffer[header_index + mesurement_start + i*3], buffer[header_index + mesurement_start + i*3 + 1], buffer[header_index + mesurement_start + i*3 + 2]);
		// printf("the values of the measurement : Ox%u Ox%u Ox%u\r\n", buffer[header_index + mesurement_start + i*3], buffer[header_index + mesurement_start + i*3 + 1], buffer[header_index + mesurement_start + i*3 + 2]);
		
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
			lidar_process_360_points();
			lidar_dump_points();

			// reset the buffer position and measurement state
			measurement_ongoing = 0;
			points_buffer_index = 0;

			return 1;
		}
			

		struct Point2D result;
		// result.x = cosf(current_angle / 180 * M_PI) * distance;
		// result.y = sinf(current_angle / 180 * M_PI) * distance;
		// stores the raw angle and distance in the point
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
/*
void lidar_process_buffer(volatile uint8_t* buffer)
{
	lidar_rx_buffer_busy = 1;

	// stops searching for headers early enough
	unsigned int end = LIDAR_RX_BUFFER_SIZE-1-LIDAR_PACKET_SIZE;

	for (unsigned int i = 0; i < end; i++)
	{
		if (buffer[i] == LIDAR_HEADER_BYTE)
		{
			// printf("processing packet at %i\r\n", i);
			if (lidar_process_frame(i, buffer))
			{
				break;
			}
		}
	}

	lidar_rx_buffer_busy = 0;
}
*/


void lidar_process_buffer(volatile uint8_t* buffer)
{
	lidar_rx_buffer_busy = 1;

	for (unsigned int i = 0; i < LIDAR_RX_BUFFER_SIZE; i++)
	{
		// printf("0x%02X ", buffer[i]);
		if (AnalysisOne(buffer[i]))
		{
			// printf("\r\n");
			printf("one frame collected\r\n");

			// do some preprocessing of the values
			float angle_start = buffer_frame.start_angle / 100.0f;
			float angle_end = buffer_frame.end_angle / 100.0f;
			if (angle_end > 360.0f) angle_end -= 360.0f;
			printf("start %.1f end %.1f\r\n", angle_start, angle_end);

			if (! measurement_ongoing)
			{
				measurement_initial_angle = angle_start;
				measurement_ongoing = 1;
				points_buffer_index = 0;
			}

			uint32_t diff = ((uint32_t)buffer_frame.end_angle + 36000 - (uint32_t)buffer_frame.start_angle) % 36000;
			float step = (float)diff / (POINT_PER_PACK - 1) / 100.0;

			for (int x = 0; x < POINT_PER_PACK; x++)
			{
				lidar_points_buffer[points_buffer_index].x = fmod(angle_start + x * step, 360.0f);
				lidar_points_buffer[points_buffer_index].y = buffer_frame.point[x].distance;
				// points_buffer_index ++;
				if (points_buffer_index < LIDAR_POINTS_BUFFER_SIZE)
				{
					points_buffer_index ++;
				}
				else
				{
					printf("WARN : reached end of point buffer");
				}
			}

			// check if we completed a full loop since the start of the measurement
			int last_comp_val = measurement_initial_angle_comp;
			lidar_compare_with_initial_angle(angle_end);
			printf("last comp %i, comp %i, end angle %.1f, init_angle %.1f\r\n", last_comp_val, measurement_initial_angle_comp, angle_end, measurement_initial_angle);

			if (last_comp_val == 1 && measurement_initial_angle_comp == 0)
			{
				lidar_process_360_points();
				lidar_dump_points();

				// reset the buffer position and measurement state
				measurement_ongoing = 0;
				points_buffer_index = 0;

				break;
			}
		}
		else
		{
			// printf("frame dropped\r\n");
		}
	}

	printf("done processing buffer\r\n");

	lidar_rx_buffer_busy = 0;
}
