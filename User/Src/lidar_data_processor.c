# include "lidar_data_processor.h"

# include <stdio.h>
# include <math.h>
//# include "od.h"

/** over how many points the distance derivative is taken */


//on utilise 
uint16_t pos_x_robot = 2500;
uint16_t pos_y_robot = 400;
int pos_theta_robot = 90;


uint16_t get_robot_x() {return pos_x_robot;}
uint16_t get_robot_y() {return pos_y_robot;}
int get_robot_theta() {return pos_theta_robot;}


int from_clock_wise_to_trigo_in_deg(uint16_t deg_clk_wise)
{
	int temp;

	if (deg_clk_wise >= 180)
	{
		temp = 180 - (deg_clk_wise - 180);
	}
	else
	{
		temp = -deg_clk_wise;
	}
	return temp;
}

float from_deg_to_rad(int angle_deg)
{
	float angle_rad = angle_deg / 180.0f * M_PI;
	return angle_rad;
}

int normalise_angle_deg(int angle_deg)
{
	int temp;
	if (angle_deg > 180)
	{
		temp = -180 + (angle_deg - 180);
	}
	else if (angle_deg < -180)
	{
		temp = 180 + (angle_deg + 180);
	}
	else
	{
		temp = angle_deg;
	}
	return temp;
}


struct Point2D get_world_position_v3(struct lidar_datapoint point)
{
	struct Point2D temp;

	int angle_lidar_deg_trigo = from_clock_wise_to_trigo_in_deg(point.angle / 100);
	int angle_deg_btw_rob_adv_ref_terrain = pos_theta_robot + angle_lidar_deg_trigo;
	int angle_deg_normalise = normalise_angle_deg(angle_deg_btw_rob_adv_ref_terrain);
	float angle_rad_btw_rob_adv_ref_terrain = from_deg_to_rad(angle_deg_normalise);

	temp.x = pos_x_robot + cosf(angle_rad_btw_rob_adv_ref_terrain) * point.distance;
	temp.y = pos_y_robot + sinf(angle_rad_btw_rob_adv_ref_terrain) * point.distance;

	return temp;
}

/**
 * Check if a point is valid or not given its area.
 * @returns 0 if usable, 1 if need to be filtered out.
 */
int filter_out_area(struct Point2D world_coord)
{
	if (world_coord.x < 0 || world_coord.x > 3000) return 1;
	if (world_coord.y < 0 || world_coord.y > 2000) return 1;
	return 0;
}

int filter_out_distance(struct lidar_datapoint* data, int c_index)
{
	if (data[c_index].distance > 1000) return 1;
	return 0;
}

/**
 * @brief Filtre les points LIDAR pour ne retourner que ceux dans le terrain
 * @param data Buffer des points LIDAR mesurés lors d'un scan 360°
 * @param data_size Nombre de points dans le buffer (typiquement ~400 points)
 * @return Structure contenant uniquement les points valides (distance > 105mm ET dans terrain)
 *
 * Filtres appliqués :
 * 1. Distance > 105mm (rejette les points erronés trop proches du LIDAR)
 * 2. Coordonnées terrain dans [0-3000, 0-2000]mm (rejette points hors terrain)
 */
struct FilteredPoints point_in_map(struct lidar_datapoint* data, int data_size)
{
	struct FilteredPoints result = {0};
	int points_rejected_distance = 0;   // Points rejetés car distance < 105mm
	int points_outside_terrain = 0;     // Points hors du terrain

	// Parcourir tous les points du scan 360°
	for (int i = 0; i < data_size; i++)
	{
		// Filtre 1 : Distance minimale (points erronés du LIDAR)
		if (data[i].distance < 105)
		{
			points_rejected_distance++;
			continue;
		}

		// Filtre 2 : Conversion en coordonnées terrain
		struct Point2D world_pos = get_world_position_v3(data[i]);

		// Filtre 3 : Points hors terrain (0-3000, 0-2000)
		if (filter_out_area(world_pos))
		{
			points_outside_terrain++;
			continue;
		}

		// Point valide → stocker les coordonnées terrain (world_pos déjà calculé)
		result.points[result.count].x = world_pos.x;
		result.points[result.count].y = world_pos.y;
		result.points[result.count].distance = data[i].distance;
		result.count++;

		// Debug: Afficher les coordonnées terrain de chaque point valide
		printf("  Point %d: T(%.0f, %.0f) mm\r\n", result.count, world_pos.x, world_pos.y);
	}

	// Affichage debug du filtrage
	printf("=== FILTRAGE POINTS ===\r\n");
	printf("Total points recus: %d\r\n", data_size);
	printf("Points rejetes (distance < 105mm): %d\r\n", points_rejected_distance);
	printf("Points hors terrain: %d\r\n", points_outside_terrain);
	printf("Points valides dans terrain: %d\r\n", result.count);
	printf("----------------------\r\n");

	return result;
}

