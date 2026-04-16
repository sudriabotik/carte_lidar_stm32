# ifndef __LIDAR_DATA_PROCESSOR_H
# define __LIDAR_DATA_PROCESSOR_H

# include <stdint.h>
# include "coords.h"

#define MAX_TERRAIN_POINTS  500  // Maximum de points LIDAR dans un scan 360°

struct lidar_datapoint
{
	int16_t x_pos;
	int16_t y_pos;
	/** store the distance relative to the robot, to avoid recalculating it */
	uint16_t distance;
	/** store the angle, in HUNDREDTH OF A DEGREE */
	uint16_t angle;
};

/**
 * Structure pour un point en coordonnées terrain (world)
 */
struct TerrainPoint {
	float x;           // Coordonnées terrain X (mm)
	float y;           // Coordonnées terrain Y (mm)
	uint16_t distance; // Distance au robot LIDAR (mm)
};

/**
 * Structure pour retourner les points filtrés avec coordonnées terrain
 */
struct FilteredPoints {
	struct TerrainPoint points[MAX_TERRAIN_POINTS];
	int count;  // Nombre de points valides après filtrage
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


struct Point2D get_world_position_v3(struct lidar_datapoint point);


int filter_out_area(struct Point2D world_coord);

/**
 * @brief Filtre les points LIDAR pour ne garder que ceux dans le terrain
 * @param data Buffer des points LIDAR d'un scan 360°
 * @param data_size Nombre de points dans le buffer
 * @return Structure contenant les points valides (distance > 105mm ET dans terrain)
 * @note Filtre 1: Distance > 105mm (points erronés du LIDAR)
 *       Filtre 2: Coordonnées terrain dans [0-3000, 0-2000]mm
 */
struct FilteredPoints point_in_map(struct lidar_datapoint* data, int data_size);

struct Point2D lidar_processor_find_blob(struct lidar_datapoint* data, int data_size);

# endif
