# include "canopen_handler.h"

# include <stdint.h>
# include "OD.h"
# include "CO_PDO.h"
# include "CO_app_STM32.h"

/* Référence externe au stack CANopen */
extern CANopenNodeSTM32* canopenNodeSTM32;

void set_opponent_data(uint16_t pos_x_mm, uint16_t pos_y_mm)
{
	OD_RAM.x2300_pos_robot_adversaire_x = pos_x_mm;
	OD_RAM.x2301_pos_robot_adversaire_y = pos_y_mm;
}

void send_opponent_data()
{
	CO_TPDOsendRequest(&canopenNodeSTM32->canOpenStack->TPDO[2]);
}

void canopen_handler_init()
{

}

void canopen_handler_process()
{

}

