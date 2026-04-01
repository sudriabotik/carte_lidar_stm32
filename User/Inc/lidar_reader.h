# ifndef __LIDAR_READER_H
# define __LIDAR_READER_H

/**
 * 
 * @note LD06 data frame format from https://gibbard.me/lidar/ :
 * 
 * <header_byte>
 * <length (uint8)>
 * <speed (uint16)>
 * <start_angle (uint16)>
 * then, the measurements :
 * <length (uint16)> + <confidence (uint8)> * LIDAR_MEASUREMENT_AMOUNT
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

# include "coords.h"

# define LIDAR_HEADER_BYTE 0x54
# define LIDAR_PACKET_SIZE 47
# define LIDAR_MEASUREMENT_AMOUNT 12
# define LIDAR_POINTS_BUFFER_SIZE 400

# define LIDAR_UART huart2

/** Twice the size of the packet size, to compensate for data reception offsets if necessary */
# define LIDAR_RX_BUFFER_SIZE 8000u

extern uint8_t lidar_rx_buffer_dma_target[LIDAR_RX_BUFFER_SIZE];
extern uint8_t lidar_rx_buffer[LIDAR_RX_BUFFER_SIZE];
/** Indicates whether it is ok or not to write new data to the lidar_rx_buffer */
extern volatile int lidar_rx_buffer_busy;
extern volatile int lidar_rx_buffer_new;

/**
 * Filled with readings until the lidar does a full revolution. Then, the content is put in lidar_current_points.
 * Then, it gets filled again, and the process repeats.
 */
extern struct Point2D lidar_points_buffer[LIDAR_POINTS_BUFFER_SIZE];

/**
 * Contains the last full 360 degrees measurement of the lidar.
 */
extern struct Point2D lidar_points_current[LIDAR_POINTS_BUFFER_SIZE];


void lidar_process_buffer();



# endif
