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
 *
 *
 * @note The data frame format has been confirmed by looking at the repositiory https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2#,
 * which seem official given the copyright headers in the files.
 */

# include "usart.h"
# include <inttypes.h>

# include "coords.h"

# define LIDAR_POINTS_BUFFER_SIZE 1000u
# define LIDAR_RX_BUFFER_SIZE 4000u
# define LIDAR_UART huart2



/** Two UART RX buffers we are alternating through, one receives DMA data and the other is processed */
extern volatile uint8_t lidar_rx_buffer_1[LIDAR_RX_BUFFER_SIZE];
extern volatile uint8_t lidar_rx_buffer_2[LIDAR_RX_BUFFER_SIZE];
/** Pointer indicating which rx buffer is currently used by the DMA */
extern volatile uint8_t* lidar_rx_buffer_dma_pointer;
/** Indicates whether it is ok or not to write new data to the lidar_rx_buffer */
extern volatile int lidar_rx_buffer_busy;
extern volatile int lidar_rx_buffer_new;



void lidar_process_buffer(volatile uint8_t* buffer);



# endif
