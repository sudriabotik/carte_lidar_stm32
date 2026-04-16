# include "canopen_cmd.h"

# include <stdint.h>
# include "CO_app_STM32.h"
# include "OD.h"
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

int16_t canopen_cmd_get_robot_x()
{
	return OD_RAM.x2200_pos_robot_x;
}

int16_t canopen_cmd_get_robot_y()
{
	return OD_RAM.x2201_pos_robot_y;
}

int16_t canopen_cmd_get_robot_theta()
{
	return OD_RAM.x2202_pos_robot_theta;
}

void canopen_cmd_send_lidar_packet()
{
	CO_TPDOsendRequest(&canopenNodeSTM32->canOpenStack->TPDO[2]);
}

void canopen_cmd_handler_init()
{

}

void canopen_cmd_handler_process()
{

}

