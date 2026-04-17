# ifndef __CANOPEN_CMD_H
# define __CANOPEN_CMD_H

# include <stdint.h>


typedef enum
{
	LIDAR_NODE_NOTHING,
	LIDAR_NODE_OBSTACLE_NEAR,
	LIDAR_NODE_AVOID,
	LIDAR_NODE_ERROR = 100,
} lidar_node_status;



void canopen_handler_init();

void canopen_handler_process();

void canopen_cmd_set_status(lidar_node_status status);

void canopen_cmd_set_opponent_data(uint16_t pos_x_mm, uint16_t pos_y_mm);

# endif
