# ifndef __LIDAR_DATA_PROCESSOR_H
# define __LIDAR_DATA_PROCESSOR_H

# include <stdint.h>
# include "coords.h"

struct lidar_datapoint
{
	int16_t x_pos;
	int16_t y_pos;
	/** store the distance relative to the robot, to avoid recalculating it */
	uint16_t distance;
	/** store the angle, in HUNDREDTH OF A DEGREE */
	uint16_t angle;
};

struct Point2D get_world_position(struct lidar_datapoint point);

/**
 * @brief Transforme un point du repère LIDAR vers le repère TERRAIN - VERSION CORRIGÉE
 * @param point Point mesuré par le LIDAR (coordonnées cartésiennes x_pos, y_pos)
 * @return Coordonnées du point dans le repère terrain absolu (mm)
 * @note Cette version V2 corrige le bug de get_world_position() qui utilisait
 *       "- lidar_x_pos" au lieu de "lidar_y_pos" pour calculer temp.y
 */
struct Point2D get_world_position_v2(struct lidar_datapoint point);

int filter_out_area(struct Point2D world_coord);

/**
 * @brief Compte et affiche les points LIDAR à l'intérieur du terrain de jeu
 * @param data Buffer des points LIDAR d'un scan 360°
 * @param data_size Nombre de points dans le buffer
 * @note Fonction de TEST pour valider le référentiel terrain
 */
void print_count_point_in_map(struct lidar_datapoint* data, int data_size);

struct Point2D lidar_processor_find_blob(struct lidar_datapoint* data, int data_size);

# endif
