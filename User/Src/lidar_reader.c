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



/**
 * Filled with readings until the lidar does a full revolution.
 * Once the full revolution is done, the data is interpreted,
 * and this buffer gets filled from the start again, and so on.
 */
struct lidar_datapoint lidar_points_buffer[LIDAR_POINTS_BUFFER_SIZE];
/** The current write index in lidar_points_buffer */
int points_buffer_index = 0;



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
	
	for (int i = 0; i < points_buffer_index; i++)
	{

		if (filter_out_area(get_world_position(lidar_points_buffer[i])))
		{
			continue;
		}
		// convert to position
		// float pos_x = cosf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		// float pos_y = sinf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		float pos_x = (float)lidar_points_buffer[i].x_pos;
		float pos_y = (float)lidar_points_buffer[i].y_pos;

		printf("%.3f %.3f\r\n", pos_x, pos_y);
	}

	printf("--END--\r\n");
	// __enable_irq();
}



int lidar_compare_with_initial_angle(float current_angle)
{
	return coord_get_delta_angle_deg(measurement_initial_angle, current_angle) < 0;
}


/**
 * Called at the end of a 360 measurement.
 * @note When this is called, lidar_points_current, current_points_end, and current_nearest_point are freshly set with new values.
 */
void lidar_process_360_points()
{
	// printf("nearest obstacle is at an angle of %.3f", lidar_points_current[buffer_nearest_point].x);
	printf("processing 360 data\r\n");
	lidar_processor_find_blob(lidar_points_buffer, points_buffer_index);
	
}



void lidar_process_buffer(volatile uint8_t* buffer)
{
	lidar_rx_buffer_busy = 1;

	for (unsigned int i = 0; i < LIDAR_RX_BUFFER_SIZE; i++)
	{
		if (AnalysisOne(buffer[i]))
		{
			printf("one frame collected\r\n");

			float angle_start = buffer_frame.start_angle / 100.0f;
			float angle_end = buffer_frame.end_angle / 100.0f;
			if (angle_end > 360.0f) angle_end -= 360.0f; // normalize the end angle

			if (! measurement_ongoing)
			{
				measurement_initial_angle = angle_start;
				measurement_ongoing = 1;
				points_buffer_index = 0;
			}

			// calculates the angle step between each measurement
			// code taken from the LD06 lidar ROS driver
			uint32_t diff = ((uint32_t)buffer_frame.end_angle + 36000 - (uint32_t)buffer_frame.start_angle) % 36000;
			float step = (float)diff / (POINT_PER_PACK - 1) / 100.0;

			for (int x = 0; x < POINT_PER_PACK; x++)
			{
				float current_angle = fmod(angle_start + x * step, 360.0f);
				lidar_points_buffer[points_buffer_index].angle = roundf(current_angle * 100.0f);
				lidar_points_buffer[points_buffer_index].distance = buffer_frame.point[x].distance;
				lidar_points_buffer[points_buffer_index].x_pos = cosf(current_angle / 180 * M_PI) * buffer_frame.point[x].distance;
				lidar_points_buffer[points_buffer_index].y_pos = sinf(current_angle / 180 * M_PI) * buffer_frame.point[x].distance;

				if (points_buffer_index < LIDAR_POINTS_BUFFER_SIZE)
				{
					points_buffer_index ++;
				}
				else
				{
					// shouldn't happen, the buffer should be big enough
					printf("WARN : reached end of point buffer");
				}
			}

			// check if we completed a full loop since the start of the measurement
			int last_comp_val = measurement_initial_angle_comp;
			measurement_initial_angle_comp = lidar_compare_with_initial_angle(angle_end);
			printf("last comp %i, comp %i, end angle %.1f, init_angle %.1f\r\n", last_comp_val, measurement_initial_angle_comp, angle_end, measurement_initial_angle);

			if (last_comp_val == 1 && measurement_initial_angle_comp == 0)
			{
				lidar_process_360_points();
				lidar_dump_points();

				// reset the buffer position and measurement state
				measurement_ongoing = 0;
				points_buffer_index = 0;

				// break and stop processing the rx buffer, one measurement is enough
				break;
			}
		}
		else
		{
			// printf("frame dropped\r\n");
		}
	}

	lidar_rx_buffer_busy = 0;
}
