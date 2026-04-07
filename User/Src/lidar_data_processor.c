# include "lidar_data_processor.h"

# include <stdio.h>
# include <math.h>


/** over how many points the distance derivative is taken */


/** The minimum width of the blob, in mm */
int blob_w_min = 10;

/** The maximum width of the blob, in mm */
int blob_w_max = 100;

int16_t lidar_x_pos = 200;
int16_t lidar_y_pos = 350;
float lidar_angle = 90;



/** circular index */
unsigned int c_index(int index, int data_size)
{
	return index % data_size;
}


/** over how many points we take the derivative */
# define DERIVATIVE_COUNT 3

/**
 * Takes the derivative of the distance at a point, approximed with the points around.
 */
float calc_derivative_distance(struct lidar_datapoint* data, int data_size, unsigned int index)
{
	int half_offset = DERIVATIVE_COUNT / 2;
	float slope_avg = 0;

	for (unsigned int i = 0; i < DERIVATIVE_COUNT; i++)
	{
		// calculate the slope between two neighbor points
		float slope = data[c_index(index + i - half_offset, data_size)].distance - data[c_index(index + i - half_offset + 1, data_size)].distance;
		slope *= data[c_index(index + i - half_offset, data_size)].angle - data[c_index(index + i - half_offset + 1, data_size)].angle;

		slope_avg += slope;
	}

	slope_avg = slope_avg / DERIVATIVE_COUNT;

	return slope_avg;
}

struct Point2D get_world_position(struct lidar_datapoint point)
{
	struct Point2D temp;
	temp.x = lidar_x_pos + cosf(lidar_angle / 180 * M_PI) * point.x_pos + sinf(lidar_angle / 180 * M_PI) * point.y_pos;
	temp.y = - lidar_x_pos + sinf(lidar_angle / 180 * M_PI) * point.x_pos + cosf(lidar_angle / 180 * M_PI) * point.y_pos;
	return temp;
}

/**
 * Check if a point is valid or not given its area.
 * @returns 0 if usable, 1 if need to be filtered out.
 */
int filter_out_area(struct Point2D world_coord)
{
	if (world_coord.x < 0 || world_coord.x > 3000) return 1;
	if (world_coord.y < 0 || world_coord.y > 2000) return 1;
	return 0;
}

int filter_out_distance(struct lidar_datapoint* data, int c_index)
{
	if (data[c_index].distance > 1000) return 1;
	return 0;
}

struct Point2D lidar_processor_find_blob(struct lidar_datapoint* data, int data_size)
{
	int index_last_break = 0;
	float last_distance_slope = 0.0f;

	printf("angle slope break\r\n");

	for (int i = 0; i < data_size + blob_w_max; i++)
	{
		int discontinuity = 0;
		float angle = data[c_index(i, data_size)].angle / 100.0f;

		if (filter_out_distance(data, c_index(i, data_size)))
		{
			// there is a discontinuity in the object
			index_last_break = i;
			discontinuity = 1;
		}

		float distance_slope = calc_derivative_distance(data, data_size, i);

		if (fabs(distance_slope - last_distance_slope) > 10 || distance_slope > 50)
		{
			// there is a discontinuity in the object
			index_last_break = i;
			discontinuity = 1;
		}


		printf("%.3f %.3f %i\r\n", angle, distance_slope, discontinuity);
	}

	printf("--END--\r\n");
}
