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
# include "lidar_data_processor.h"

# define LIDAR_POINTS_BUFFER_SIZE 1000
# define LIDAR_RX_BUFFER_SIZE 2000  // Réduit pour éviter overflow du buffer de points (2000 bytes = ~42 frames × 12 pts = 504 points)
# define LIDAR_UART huart2



/** Two UART RX buffers we are alternating through, one receives DMA data and the other is processed */
extern volatile uint8_t lidar_rx_buffer_1[LIDAR_RX_BUFFER_SIZE];
extern volatile uint8_t lidar_rx_buffer_2[LIDAR_RX_BUFFER_SIZE];

/** Double buffering avec interruption UART2 */
extern volatile uint16_t uart_buffer_write_index;  // Position écriture dans buffer actif (0-4095)
extern volatile int8_t uart_buffer_active;         // Buffer actif pour ISR: -1 = pleins, 1 = buffer_1, 2 = buffer_2
extern volatile uint8_t uart_buffer_ready;         // Buffer prêt à traiter: 0, 1 ou 2

/** Legacy DMA variables (non utilisées en mode interruption) */
extern volatile uint8_t* lidar_rx_buffer_dma_pointer;
extern volatile int lidar_rx_buffer_busy;
extern volatile int lidar_rx_buffer_new;

/**
 * @brief Traite un buffer DMA complet (recherche frames LIDAR)
 * @param buffer Pointeur vers lidar_rx_buffer_1 ou lidar_rx_buffer_2
 * @note Parse le buffer entier avec AnalysisOne(), détecte scan 360°
 */
void lidar_process_buffer(volatile uint8_t* buffer);

/**
 * @brief Traite une frame LIDAR en mode polling (accumulation pour scan 360°)
 * @note À appeler après chaque frame valide détectée par AnalysisOne()
 * @note Accumule les points dans lidar_points_buffer[], détecte fin scan 360°
 * @note Appelle lidar_process_360_points() quand révolution complète
 */
void lidar_process_polling(void);

# endif
