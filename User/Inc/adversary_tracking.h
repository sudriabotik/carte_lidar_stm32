/*
 * adversary_tracking.h
 *
 * Module de tracking d'adversaire pour robot LIDAR
 * Détecte et suit la position d'un adversaire sur le terrain
 */

#ifndef ADVERSARY_TRACKING_H
#define ADVERSARY_TRACKING_H

#include <stdint.h>
#include "lidar_reader.h"
#include "lidar_data_processor.h"

// Paramètres de tracking (modifiables selon les tests)
#define MIN_POINTS_FOR_BLOB 8             // Au moins 6 points pour considérer un objet
#define DIAMETRE_MAX_BLOB 400              // Distance max entre 2 points d'un même blob (mm)
#define MAX_POSITION_JUMP 500              // Déplacement max entre 2 scans (mm)
#define MIN_CONSECUTIVE_DETECTIONS 2       // Scans consécutifs pour confirmer
#define MAX_CONSECUTIVE_ABSENCES 3         // Scans sans détection avant de perdre l'adversaire

/**
 * Structure de tracking de l'adversaire
 */
struct AdversaryTracking {
    uint8_t is_detected;           // 0 = pas d'adversaire, 1 = adversaire confirmé
    float position_x;              // Position X terrain (mm)
    float position_y;              // Position Y terrain (mm)
    uint8_t confidence;            // Confiance 0-100%
    int consecutive_detections;    // Nombre de détections consécutives
    int consecutive_absences;      // Nombre de scans sans détection
};

/**
 * Fonction principale de tracking
 * @param filtered_points Points en coordonnées terrain déjà filtrés
 * @param count Nombre de points filtrés
 */
void adversary_tracking(struct TerrainPoint* filtered_points, int count);

/**
 * Récupérer l'état actuel du tracking
 * @return Structure AdversaryTracking avec la position et l'état de l'adversaire
 */
struct AdversaryTracking get_adversary_state(void);

/**
 * Reset manuel du tracking (optionnel)
 */
void reset_adversary_tracking(void);

#endif /* ADVERSARY_TRACKING_H */
