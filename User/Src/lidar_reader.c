# include "lidar_reader.h"
#include "lidar_data_processor.h"
# include "stm32g4xx_hal.h"
# include <stdio.h>
# include <math.h>
# include <memory.h>

# include <lipkg_copy.h>
# include "adversary_tracking.h"



volatile uint8_t lidar_rx_buffer_1[LIDAR_RX_BUFFER_SIZE] = {0};
volatile uint8_t lidar_rx_buffer_2[LIDAR_RX_BUFFER_SIZE] = {0};

/** Variables pour double buffering avec interruption UART2 */
volatile uint16_t uart_buffer_write_index = 0;  // Position écriture (0-4095)
volatile int8_t uart_buffer_active = 1;         // Buffer actif: -1 = pleins, 1 = buffer_1, 2 = buffer_2
volatile uint8_t uart_buffer_ready = 0;         // Buffer prêt: 0 = aucun, 1 = buffer_1, 2 = buffer_2

/** Legacy DMA variables (non utilisées en mode interruption) */
volatile uint8_t* lidar_rx_buffer_dma_pointer;
volatile int lidar_rx_buffer_busy = 0;
volatile int lidar_rx_buffer_new = 0;



/**
 * Filled with readings until the lidar does a full revolution.
 * Once the full revolution is done, the data is interpreted,
 * and this buffer gets filled from the start again, and so on.
 */
struct lidar_datapoint lidar_points_buffer[LIDAR_POINTS_BUFFER_SIZE];
/** The current write index in lidar_points_buffer */
int points_buffer_index = 0;



// the lidar angle at the start of the current 360 scan
float measurement_initial_angle;
// 1 if a 360 measurement is ongoing, 0 if a new must be started at the next processed frame.
int measurement_ongoing = 0;
// if this switches from 1 to 0, then the current 360 measurement is finished.
int measurement_initial_angle_comp = 0;



// the index of the nearest point in lidar_points_buffer
unsigned int buffer_nearest_point = 0;



/**
 * @brief Affiche tous les points du scan 360° en coordonnées x,y (mm)
 * @note Filtre les points hors zone valide, affiche format: x y (un point par ligne)
 */
void lidar_dump_points()
{
	// __disable_irq();
	printf("\r\nx y\r\n");
	
	for (int i = 0; i < points_buffer_index; i++)
	{

		// convert to position
		// float pos_x = cosf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		// float pos_y = sinf(lidar_points_buffer[i].x / 180 * M_PI) * lidar_points_buffer[i].y;
		float pos_x = (float)lidar_points_buffer[i].x_pos;
		float pos_y = (float)lidar_points_buffer[i].y_pos;

		printf("%.3f %.3f\r\n", pos_x, pos_y);
	}

	printf("--END--\r\n");
	// __enable_irq();
}


/**
 * @brief Compare l'angle actuel avec l'angle initial du scan 360°
 * @param current_angle Angle actuel du LIDAR (en degrés, 0-360)
 * @return 1 si l'angle actuel est "après" l'angle initial, 0 si "avant" (détecte boucle complète)
 */
int lidar_compare_with_initial_angle(float current_angle)
{
	return coord_get_delta_angle_deg(measurement_initial_angle, current_angle) < 0;
}


/**
 * @brief Traite les données d'un scan complet 360° (détection et tracking adversaire)
 * @note Appelée quand le LIDAR a fait une révolution complète
 * @note Utilise lidar_points_buffer[] rempli avec ~400 points (angle + distance + x + y)
 * @note Chaîne de traitement : Filtrage → Tracking adversaire
 */
void lidar_process_360_points()
{
	printf("processing 360 data\r\n");

	if (points_buffer_index < 800)
    {
        printf("SCAN INCOMPLET: %d points (< 800) - IGNORE\r\n", points_buffer_index);
        return;  // Ne pas traiter ce scan
    }


	// Étape 1 : Filtrer les points valides (distance > 105mm ET dans terrain)
	struct FilteredPoints filtered = point_in_map(lidar_points_buffer, points_buffer_index);

	// Étape 2 : Tracking de l'adversaire avec les points filtrés
	adversary_tracking(filtered.points, filtered.count);

	// Etape 3 : Retrouver la position de l'adversaire relative au robot.
	struct AdversaryTracking opponent_data = get_adversary_state();
	float tmp_x = opponent_data.position_x - get_robot_x();
	float tmp_y = opponent_data.position_y - get_robot_y();
	float robot_rotation = get_robot_theta();

	// rotate the relative x and y to be in the robot's basis (+x front, +y left)
	float rel_x = tmp_x * cosf(robot_rotation / 180 * M_PI) - tmp_y * sinf(robot_rotation / 180 * M_PI);
	float rel_y = tmp_x * sinf(robot_rotation / 180 * M_PI) + tmp_y * cosf(robot_rotation / 180 * M_PI);
	

	// Etape 3 : Detecter si l'adversaire est dans un carré devant ou derrière le robot
	
	// first check if the opponent is in a "tunnel" extending infinitely behind and in front of the robot
	if (rel_y > - 150 && rel_y < 150)
	{
		printf("THE OPPONENT IS IN THE TUNNER\r\n");
	}
	
}



/**
 * @brief Traite un buffer DMA complet (4096 bytes) pour extraire les frames LIDAR
 * @param buffer Pointeur vers lidar_rx_buffer_1 ou lidar_rx_buffer_2
 * @note Parse chaque octet avec AnalysisOne(), accumule points dans lidar_points_buffer[]
 * @note Détecte scan 360° complet et appelle lidar_process_360_points()
 */
void lidar_process_buffer(volatile uint8_t* buffer)
{
	lidar_rx_buffer_busy = 1;

	for (unsigned int i = 0; i < LIDAR_RX_BUFFER_SIZE; i++)
	{
		if (AnalysisOne(buffer[i]))
		{
			//printf("one frame collected\r\n");

			float angle_start = buffer_frame.start_angle / 100.0f;
			float angle_end = buffer_frame.end_angle / 100.0f;
			if (angle_end > 360.0f) angle_end -= 360.0f; // normalize the end angle

			if (! measurement_ongoing)
			{
				measurement_initial_angle = angle_start;
				measurement_ongoing = 1;
				points_buffer_index = 0;
			}

			// calculates the angle step between each measurement
			// code taken from the LD06 lidar ROS driver
			uint32_t diff = ((uint32_t)buffer_frame.end_angle + 36000 - (uint32_t)buffer_frame.start_angle) % 36000;
			float step = (float)diff / (POINT_PER_PACK - 1) / 100.0;

			for (int x = 0; x < POINT_PER_PACK; x++)
			{
				float current_angle = fmod(angle_start + x * step, 360.0f);
				lidar_points_buffer[points_buffer_index].angle = roundf(current_angle * 100.0f);
				lidar_points_buffer[points_buffer_index].distance = buffer_frame.point[x].distance;
				lidar_points_buffer[points_buffer_index].x_pos = cosf(current_angle / 180 * M_PI) * buffer_frame.point[x].distance;
				lidar_points_buffer[points_buffer_index].y_pos = - sinf(current_angle / 180 * M_PI) * buffer_frame.point[x].distance;

				if (points_buffer_index < LIDAR_POINTS_BUFFER_SIZE)
				{
					points_buffer_index ++;
				}
				else
				{
					// shouldn't happen, the buffer should be big enough
					//printf("WARN : reached end of point buffer");
				}
			}

			// check if we completed a full loop since the start of the measurement
			int last_comp_val = measurement_initial_angle_comp;
			measurement_initial_angle_comp = lidar_compare_with_initial_angle(angle_end);
			//printf("last comp %i, comp %i, end angle %.1f, init_angle %.1f\r\n", last_comp_val, measurement_initial_angle_comp, angle_end, measurement_initial_angle);

			if (last_comp_val == 1 && measurement_initial_angle_comp == 0)
			{
				lidar_process_360_points();
				//lidar_dump_points();

				// reset the buffer position and measurement state
				measurement_ongoing = 0;
				points_buffer_index = 0;

				// break and stop processing the rx buffer, one measurement is enough
				break;
			}
		}
		else
		{
			// printf("frame dropped\r\n");
		}
	}

	lidar_rx_buffer_busy = 0;
}


/**
 * @brief Traite une frame LIDAR en mode polling (accumulation pour scan 360°)
 * @note À appeler après chaque frame valide détectée par AnalysisOne()
 * @note Utilise buffer_frame global rempli par AnalysisOne()
 */
void lidar_process_polling(void)
{
	// Extraction des angles depuis la frame reçue
	float angle_start = buffer_frame.start_angle / 100.0f;
	float angle_end = buffer_frame.end_angle / 100.0f;
	if (angle_end > 360.0f) angle_end -= 360.0f; // normalise l'angle de fin

	// Initialise un nouveau scan 360° si nécessaire
	if (!measurement_ongoing)
	{
		measurement_initial_angle = angle_start;
		measurement_ongoing = 1;
		points_buffer_index = 0;
		measurement_initial_angle_comp = 0;  // Initialise pour garantir détection 1→0
	}

	// Calcule l'angle step entre chaque mesure (12 points par frame)
	// Code tiré du driver ROS LD06
	uint32_t diff = ((uint32_t)buffer_frame.end_angle + 36000 - (uint32_t)buffer_frame.start_angle) % 36000;
	float step = (float)diff / (POINT_PER_PACK - 1) / 100.0;

	// Accumule les 12 points de la frame dans le buffer global
	for (int x = 0; x < POINT_PER_PACK; x++)
	{
		float current_angle = fmod(angle_start + x * step, 360.0f);
		lidar_points_buffer[points_buffer_index].angle = roundf(current_angle * 100.0f);
		lidar_points_buffer[points_buffer_index].distance = buffer_frame.point[x].distance;
		lidar_points_buffer[points_buffer_index].x_pos = cosf(current_angle / 180 * M_PI) * buffer_frame.point[x].distance;
		lidar_points_buffer[points_buffer_index].y_pos = sinf(current_angle / 180 * M_PI) * buffer_frame.point[x].distance;

		if (points_buffer_index < LIDAR_POINTS_BUFFER_SIZE)
		{
			points_buffer_index++;
		}
		else
		{
			// Ne devrait pas arriver, le buffer doit être assez grand
			printf("WARN : reached end of point buffer\r\n");
		}
	}

	// Vérifie si on a complété une révolution complète 360°
	int last_comp_val = measurement_initial_angle_comp;
	measurement_initial_angle_comp = lidar_compare_with_initial_angle(angle_end);

	// Détecte le passage de 1 → 0 (on vient de repasser par l'angle initial)
	if (last_comp_val == 1 && measurement_initial_angle_comp == 0)
	{
		// Scan 360° complet détecté!
		lidar_process_360_points();
		// lidar_dump_points();  // Décommenter pour afficher tous les points (génère beaucoup de prints!)

		// Réinitialise pour le prochain scan
		measurement_ongoing = 0;
		points_buffer_index = 0;
	}
}
