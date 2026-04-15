# ifndef __LIPKG_COPY_H
# define __LIPKG_COPY_H

# include <stdint.h>



enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;


extern LiDARFrameTypeDef buffer_frame;

/**
 * @brief Parse un octet LIDAR (machine à états)
 * @param byte L'octet reçu depuis UART2
 * @return 1 si une frame complète de 47 bytes est validée (CRC OK), 0 sinon
 * @note Remplit buffer_frame quand une frame valide est détectée
 */
int AnalysisOne(uint8_t byte);

# endif
