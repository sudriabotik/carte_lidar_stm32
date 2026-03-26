# ifndef __LD06_READER_H
# define __LD06_READER_H

/**
 * 
 * @note LD06 data frame format from https://gibbard.me/lidar/ :
 * 
 * <header_byte>
 * <length (uint8)>
 * <speed (uint16)>
 * <start_angle (uint16)>
 * then, the measurements :
 * <length (uint16)> + <confidence (uint8)> * LD06_MEASUREMENT_AMOUNT
 * <stop_angle (uint16)>
 * <timestamp (uint16)>
 * <crc (uint8)>
 * 
 * the data is in little endian, for example 1023 is sent as 0xff 0x03
 * 
 * total size occupied by static data : 10 bytes
 * size of one measurement : 3 bytes
 * 
 * 
 * IMPORTANT NOTES
 * I don't know what is the purpose of the length field. It is always 44 when the actual total byte count of the packet is 47.
 * So, the program use a hardcoded value of 47 instead.
 */

# include "usart.h"
# include <inttypes.h>

# define LD06_HEADER_BYTE 0x54
# define LD06_PACKET_SIZE 47
# define LD06_MEASUREMENT_AMOUNT 12
# define LD06_POINTS_BUFFER_SIZE 400

# define LD06_UART huart2

/** Twice the size of the packet size, to compensate for data reception offsets if necessary */
# define LD06_RX_BUFFER_SIZE 94

extern uint8_t ld06_rx_buffer[LD06_RX_BUFFER_SIZE];

struct Point2D
{
	float x, y;
};

extern struct Point2D ld06_points_buffer[LD06_POINTS_BUFFER_SIZE];


void ld06_process_buffer();



# endif