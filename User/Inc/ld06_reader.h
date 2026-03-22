# ifndef __LD06_READER_H
# define __LD06_READER_H

# include "usart.h"
# include <inttypes.h>

# define LD06_HEADER_BYTE 0x54
# define LD06_PACKET_SIZE 47

# define LD06_UART huart2

/** Twice the size of the packet size, to compensate for data reception offsets if necessary */
# define LD06_RX_BUFFER_SIZE 94

extern uint8_t ld06_rx_buffer[LD06_RX_BUFFER_SIZE];


void ld06_process_buffer();



# endif