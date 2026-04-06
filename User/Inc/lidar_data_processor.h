# ifndef __LIDAR_DATA_PROCESSOR_H
# define __LIDAR_DATA_PROCESSOR_H

# include <stdint.h>
# include "coords.h"

struct lidar_datapoint
{
	uint16_t x_pos;
	uint16_t y_pos;
	/** store the distance relative to the robot, to avoid recalculating it */
	uint16_t distance;
	/** store the angle, in HUNDREDTH OF A DEGREE */
	uint16_t angle;
};

struct Point2D lidar_processor_find_blob(struct lidar_datapoint* data, int data_size);

# endif
