# ifndef __CANOPEN_HANDLER_H
# define __CANOPEN_HANDLER_H



typedef enum
{
	LIDAR_NODE_OK,
} LIDAR_NODE_STATUS;



void canopen_handler_init();

void canopen_handler_process();

# endif
