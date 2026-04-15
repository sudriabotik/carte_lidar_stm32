# include "lidar_data_processor.h"

# include <stdio.h>
# include <math.h>
//# include "od.h"

/** over how many points the distance derivative is taken */


/** The minimum width of the blob, in mm */
int blob_w_min = 10;

/** The maximum width of the blob, in mm */
int blob_w_max = 100;

// ce n'est pas trés clair ces variables
int16_t lidar_x_pos = 700;
int16_t lidar_y_pos = 300;
float lidar_angle = 90; 

//on utilise 
uint16_t pos_x_robot = 700 ;
uint16_t pos_y_robot = 300 ;
int pos_theta_robot = 0;


/** circular index */
unsigned int c_index(int index, int data_size)
{
	return index % data_size;
}


/** over how many points we take the derivative */
# define DERIVATIVE_COUNT 3

/**
 * Takes the derivative of the distance at a point, approximed with the points around.
 */
float calc_derivative_distance(struct lidar_datapoint* data, int data_size, unsigned int index)
{
	int half_offset = DERIVATIVE_COUNT / 2;
	float slope_avg = 0;

	for (unsigned int i = 0; i < DERIVATIVE_COUNT; i++)
	{
		// calculate the slope between two neighbor points
		float slope = data[c_index(index + i - half_offset, data_size)].distance - data[c_index(index + i - half_offset + 1, data_size)].distance;
		slope *= data[c_index(index + i - half_offset, data_size)].angle - data[c_index(index + i - half_offset + 1, data_size)].angle;

		slope_avg += slope;
	}

	slope_avg = slope_avg / DERIVATIVE_COUNT;

	return slope_avg;
}

struct Point2D get_world_position(struct lidar_datapoint point)
{
	struct Point2D temp;
	temp.x = lidar_x_pos + cosf(lidar_angle / 180 * M_PI) * point.x_pos + sinf(lidar_angle / 180 * M_PI) * point.y_pos;
	temp.y = - lidar_x_pos + sinf(lidar_angle / 180 * M_PI) * point.x_pos + cosf(lidar_angle / 180 * M_PI) * point.y_pos;
	return temp;
}

/**
 * @brief Transforme un point du repère LIDAR vers le repère TERRAIN (monde) - VERSION CORRIGÉE
 * @param point Point mesuré par le LIDAR en coordonnées cartésiennes (x_pos, y_pos en mm)
 * @return Coordonnées du point dans le repère terrain absolu (mm)
 *
 * Cette fonction effectue une transformation de coordonnées en 2 étapes:
 * 1. ROTATION du point selon l'orientation du robot (lidar_angle)
 * 2. TRANSLATION vers la position du robot sur le terrain (lidar_x_pos, lidar_y_pos)
 *
 * Formule mathématique (matrice de rotation 2D + translation):
 *
 *   [X_terrain]   [cos(θ)  -sin(θ)]   [X_lidar]   [X_robot]
 *   [Y_terrain] = [sin(θ)   cos(θ)] × [Y_lidar] + [Y_robot]
 *
 * Exemple concret:
 * - Robot positionné à (700, 300) mm sur le terrain
 * - Robot orienté à 0° (flèche LIDAR pointe vers l'avant)
 * - LIDAR détecte un obstacle à (500, 0) mm en coordonnées LIDAR (500mm devant le robot)
 * - Coordonnées terrain = (700 + 500×cos(0°) + 0×sin(0°), 300 + 500×sin(0°) + 0×cos(0°))
 *                       = (700 + 500, 300 + 0)
 *                       = (1200, 300) mm sur le terrain
 *
 * @note Cette version V2 corrige le bug de la version précédente qui utilisait
 *       "- lidar_x_pos" au lieu de "lidar_y_pos" pour le calcul de temp.y
 */
struct Point2D get_world_position_v2(struct lidar_datapoint point)
{
	struct Point2D temp;

	// Conversion angle en radians (lidar_angle est en degrés)
	float angle_rad = lidar_angle / 180.0f * M_PI;

	// Coordonnée X terrain = Position X du robot + composantes X et Y du point après rotation
	// X_terrain = X_robot + cos(θ) × X_lidar + sin(θ) × Y_lidar
	temp.x = lidar_x_pos
	       + cosf(angle_rad) * point.x_pos   // Composante X du point tourné
	       + sinf(angle_rad) * point.y_pos;  // Contribution de Y après rotation

	// Coordonnée Y terrain = Position Y du robot + composantes X et Y du point après rotation
	// Y_terrain = Y_robot + sin(θ) × X_lidar + cos(θ) × Y_lidar
	// ⚠️ CORRECTION: Utilise lidar_y_pos au lieu de "- lidar_x_pos"
	temp.y = lidar_y_pos
	       + sinf(angle_rad) * point.x_pos   // Contribution de X après rotation
	       + cosf(angle_rad) * point.y_pos;  // Composante Y du point tourné

	return temp;
}

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
 * @brief Compte et affiche les points LIDAR qui sont à l'intérieur du terrain de jeu
 * @param data Buffer des points LIDAR mesurés lors d'un scan 360°
 * @param data_size Nombre de points dans le buffer (typiquement ~400 points)
 *
 * Cette fonction sert à TESTER le référentiel terrain en comptant:
 * - Combien de points sont DANS le terrain (0 ≤ X ≤ 3000mm, 0 ≤ Y ≤ 2000mm)
 * - Combien de points sont HORS du terrain (murs, espace vide)
 *
 * Test attendu:
 * - Terrain VIDE → Peu de points inside (~50-100 points, seulement les bords)
 * - Ajouter obstacle → Points inside AUGMENTE (ex: ~150-200 points)
 * - Retirer obstacle → Points inside DIMINUE (retour ~50-100)
 */
void print_count_point_in_map(struct lidar_datapoint* data, int data_size)
{
	int total_points_analyzed = 0;      // Total de points reçus du LIDAR
	int points_inside_terrain = 0;      // Points DANS le terrain (valides)
	int points_outside_terrain = 0;     // Points HORS terrain (ignorés)

	lidar_datapoint data_filtred ; 

	printf("=== TEST REFERENCIEL TERRAIN ===\r\n");
	printf("Points DANS terrain (angle, dist, LIDAR_xy, TERRAIN_xy):\r\n");

	// Parcourir tous les points du scan 360°
	for (int i = 0; i < data_size; i++)
	{
		total_points_analyzed++;


		// Transformer le point du repère LIDAR → repère terrain (monde)
		struct Point2D world_pos = get_world_position_v3(data[i]);

		// Vérifier si le point est dans le terrain (0-3000mm × 0-2000mm)
		if (filter_out_area(world_pos))
		{
			// Point HORS terrain (à ignorer)
			points_outside_terrain++;
		}
		else
		{
			// Point DANS terrain (valide pour analyse)
			points_inside_terrain++;

			// Afficher les détails de ce point pour debug
			// Format: Point X: a=XXX.X° d=XXXXmm L(x,y) T(x,y)
			float angle_deg = data[i].angle / 100.0f;  // Convertir centièmes de degré → degrés
			printf("P%d: a=%.1f d=%u L(%d,%d) T(%.0f,%.0f)\r\n",
			       points_inside_terrain,
			       angle_deg,
			       data[i].distance,
			       data[i].x_pos,
			       data[i].y_pos,
			       world_pos.x,
			       world_pos.y);
		}
	}

	// Calculer le pourcentage de points dans le terrain
	float percentage_inside = 0.0f;
	if (total_points_analyzed > 0)
	{
		percentage_inside = (float)points_inside_terrain / (float)total_points_analyzed * 100.0f;
	}

	// Affichage du résumé
	printf("---\r\n");
	printf("Total points analyses: %d\r\n", total_points_analyzed);
	printf("Points DANS terrain (0-3000, 0-2000): %d\r\n", points_inside_terrain);
	printf("Points HORS terrain: %d\r\n", points_outside_terrain);
	printf("Pourcentage dans terrain: %.1f%%\r\n", percentage_inside);
	printf("--END--\r\n");
}

struct Point2D lidar_processor_find_blob(struct lidar_datapoint* data, int data_size)
{
	int index_last_break = 0;
	float last_distance_slope = 0.0f;
	int blob_count = 0;  // Compteur de blobs détectés

	printf("angle slope break\r\n");

	for (int i = 0; i < data_size + blob_w_max; i++)
	{
		int discontinuity = 0;
		float angle = data[c_index(i, data_size)].angle / 100.0f;

		if (filter_out_distance(data, c_index(i, data_size)))
		{
			// there is a discontinuity in the object
			index_last_break = i;
			discontinuity = 1;
		}

		float distance_slope = calc_derivative_distance(data, data_size, i);

		if (fabs(distance_slope - last_distance_slope) > 10 || distance_slope > 50)
		{
			// there is a discontinuity in the object
			index_last_break = i;
			discontinuity = 1;
		}

		// Compte les discontinuités (blobs potentiels)
		if (discontinuity == 1)
		{
			blob_count++;
		}

		// Print verbeux commenté pour réduire le traffic UART
		// printf("anfle=%.3f  distance_slope%.3f discontinuity%i\r\n", angle, distance_slope, discontinuity);
	}

	// Affiche le résultat final
	printf("Blobs detected: %d\r\n", blob_count);
	printf("--END--\r\n");
}
