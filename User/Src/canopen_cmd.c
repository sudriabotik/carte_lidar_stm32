# include "canopen_cmd.h"

# include <stdint.h>
# include "CO_app_STM32.h"
# include "OD.h"
#include "lidar_data_processor.h"
//# include "CO_PDO.h"


/* Référence externe au stack CANopen */
extern CANopenNodeSTM32* canopenNodeSTM32;

void canopen_cmd_set_opponent_data(uint16_t pos_x_mm, uint16_t pos_y_mm)
{
	OD_RAM.x2300_pos_robot_adversaire_x = pos_x_mm;
	OD_RAM.x2301_pos_robot_adversaire_y = pos_y_mm;
}

void canopen_cmd_set_status(lidar_node_status status)
{
	OD_RAM.x2302_status_carte_lidar = (uint8_t)status;
}

void canopen_cmd_send_lidar_packet()
{
	CO_TPDOsendRequest(&canopenNodeSTM32->canOpenStack->TPDO[2]);
}

void canopen_cmd_fetch_robot_pos()
{
	set_robot_x(OD_RAM.x2200_pos_robot_x);
	set_robot_y(OD_RAM.x2201_pos_robot_y);
	set_robot_theta(OD_RAM.x2202_pos_robot_theta);
}

void canopen_cmd_init()
{
	canopen_cmd_set_status(LIDAR_NODE_NOTHING);

	OD_RAM.x2200_pos_robot_x = 200;
	OD_RAM.x2201_pos_robot_y = 0;
	OD_RAM.x2202_pos_robot_theta = 0; // in degrees, trigonometric direction
}

