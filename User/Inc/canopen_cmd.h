# ifndef __CANOPEN_CMD_H
# define __CANOPEN_CMD_H



typedef enum
{
	LIDAR_NODE_NOTHING,
	LIDAR_NODE_OBSTACLE_NEAR,
	LIDAR_NODE_AVOID,
	LIDAR_NODE_ERROR = 100,
} lidar_node_status;



void canopen_handler_init();

void canopen_handler_process();

# endif
