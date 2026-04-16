# ifndef __CANOPEN_CMD_H
# define __CANOPEN_CMD_H



#include <stdint.h>
typedef enum
{
	LIDAR_NODE_NOTHING = 0,
	LIDAR_NODE_OBSTACLE_FRONT = 1,
	LIDAR_NODE_OBSTACLE_BACK = 2,
	LIDAR_NODE_ERROR = 100,
} lidar_node_status;


void canopen_cmd_set_opponent_data(uint16_t pos_x_mm, uint16_t pos_y_mm);

void canopen_cmd_set_status(lidar_node_status status);

int16_t canopen_cmd_get_robot_x();
int16_t canopen_cmd_get_robot_y();
int16_t canopen_cmd_get_robot_theta();

void canopen_handler_init();

void canopen_handler_process();

# endif
