/*
 * adversary_tracking.c
 *
 * Implémentation du système de tracking d'adversaire
 * Détecte les blobs et suit leur position avec filtrage temporel
 */

#include "adversary_tracking.h"
#include "lidar_data_processor.h"
#include <math.h>
#include <stdio.h>


//# define CLUSTER_VERBOSE

// Variable statique de tracking (persiste entre les appels)
static struct AdversaryTracking tracker = {0};

/**
 * Structure temporaire pour un blob détecté dans un scan
 */
struct BlobDetection {
    uint8_t found;        // 0 = pas de blob, 1 = blob trouvé
    float center_x;       // Centre X du blob (mm)
    float center_y;       // Centre Y du blob (mm)
    int point_count;      // Nombre de points dans le blob
};

/**
 * Structure pour un cluster détecté
 */
struct Cluster {
    float sum_x;
    float sum_y;
    int point_count;
    float center_x;
    float center_y;
    float distance_to_robot;
    // Bounding box (boîte englobante)
    float min_x, max_x;
    float min_y, max_y;
    // Indices des points appartenant au cluster
    uint8_t point_indices[MAX_TERRAIN_POINTS];
};

/**
 * Détecter des blobs dans les points filtrés avec clustering DBSCAN
 * Sépare les objets proches et retourne le plus proche du robot
 */
static struct BlobDetection detect_blob(struct TerrainPoint* points, int count)
{
    struct BlobDetection blob = {0};

    // Pas assez de points pour former un objet
    if (count < MIN_POINTS_FOR_BLOB)
    {
        return blob;  // blob.found = 0
    }

    // Étape 1 : Clustering avec bounding box - Grouper les points dans boîtes 400×400mm
    uint8_t assigned[MAX_TERRAIN_POINTS] = {0};  // 0 = non assigné
    struct Cluster clusters[10] = {0};           // Max 10 clusters
    int clusters_count = 0;

    for (int i = 0; i < count; i++)
    {
        if (assigned[i])
            continue;

        // Créer un nouveau cluster avec le point i
        clusters[clusters_count].sum_x = points[i].x;
        clusters[clusters_count].sum_y = points[i].y;
        clusters[clusters_count].point_count = 1;

        // Initialiser la bounding box avec le premier point
        clusters[clusters_count].min_x = points[i].x;
        clusters[clusters_count].max_x = points[i].x;
        clusters[clusters_count].min_y = points[i].y;
        clusters[clusters_count].max_y = points[i].y;

        // Enregistrer l'indice du point
        clusters[clusters_count].point_indices[0] = i;
        assigned[i] = clusters_count + 1;

        // Chercher tous les points qui peuvent entrer dans la bounding box
        for (int j = i + 1; j < count; j++)
        {
            if (assigned[j])
                continue;

            // Calculer la nouvelle bounding box si on ajoute le point j
            float new_min_x = (points[j].x < clusters[clusters_count].min_x) ? points[j].x : clusters[clusters_count].min_x;
            float new_max_x = (points[j].x > clusters[clusters_count].max_x) ? points[j].x : clusters[clusters_count].max_x;
            float new_min_y = (points[j].y < clusters[clusters_count].min_y) ? points[j].y : clusters[clusters_count].min_y;
            float new_max_y = (points[j].y > clusters[clusters_count].max_y) ? points[j].y : clusters[clusters_count].max_y;

            // Vérifier que la bounding box reste ≤ 400mm en X ET en Y
            float new_width = new_max_x - new_min_x;
            float new_height = new_max_y - new_min_y;

            if (new_width <= DIAMETRE_MAX_BLOB && new_height <= DIAMETRE_MAX_BLOB)
            {
                // Le point peut être ajouté au cluster
                clusters[clusters_count].sum_x += points[j].x;
                clusters[clusters_count].sum_y += points[j].y;

                // Mettre à jour la bounding box
                clusters[clusters_count].min_x = new_min_x;
                clusters[clusters_count].max_x = new_max_x;
                clusters[clusters_count].min_y = new_min_y;
                clusters[clusters_count].max_y = new_max_y;

                // Enregistrer l'indice du point
                clusters[clusters_count].point_indices[clusters[clusters_count].point_count] = j;
                clusters[clusters_count].point_count++;
                assigned[j] = clusters_count + 1;
            }
        }

        clusters_count++;
        if (clusters_count >= 10)
            break;  // Max clusters atteint
    }

    // Étape 2 : Calculer le centroïde et la distance pour chaque cluster
    for (int i = 0; i < clusters_count; i++)
    {
        clusters[i].center_x = clusters[i].sum_x / clusters[i].point_count;
        clusters[i].center_y = clusters[i].sum_y / clusters[i].point_count;

        // Distance du cluster au robot (robot à l'origine dans son référentiel)
        // On utilise la position du robot dans le référentiel terrain
        // Pour simplifier, on calcule la distance euclidienne depuis (0,0) du cluster
        clusters[i].distance_to_robot = sqrtf(clusters[i].center_x * clusters[i].center_x +
                                               clusters[i].center_y * clusters[i].center_y);
    }

# ifdef CLUSTER_VERBOSE
    // Étape 3 : Afficher tous les clusters avec leurs points pour debug

    printf("=== CLUSTERING ===\r\n");
    printf("Clusters detectes: %d\r\n", clusters_count);
    for (int i = 0; i < clusters_count; i++)
    {
        float width = clusters[i].max_x - clusters[i].min_x;
        float height = clusters[i].max_y - clusters[i].min_y;

        printf("Cluster %d: %d pts, centre (%.0f, %.0f)\r\n",
               i + 1,
               clusters[i].point_count,
               clusters[i].center_x,
               clusters[i].center_y);
        printf("  bbox [%.0f-%.0f x %.0f-%.0f] (%.0f x %.0f mm), dist robot %.0f mm\r\n",
               clusters[i].min_x, clusters[i].max_x,
               clusters[i].min_y, clusters[i].max_y,
               width, height,
               clusters[i].distance_to_robot);

        // Afficher tous les points du cluster
        printf("  Points: ");
        for (int p = 0; p < clusters[i].point_count; p++)
        {
            uint8_t idx = clusters[i].point_indices[p];
            printf("(%.0f,%.0f)", points[idx].x, points[idx].y);
            if (p < clusters[i].point_count - 1)
                printf(" ");
        }
        printf("\r\n");
    }
# endif

    // Étape 4 : Trouver le cluster le plus PROCHE du robot (pas le plus gros)
    int closest_cluster = -1;
    float min_distance = 100000.0f;

    for (int i = 0; i < clusters_count; i++)
    {
        // Filtrer les clusters avec trop peu de points
        if (clusters[i].point_count < MIN_POINTS_FOR_BLOB)
            continue;

        if (clusters[i].distance_to_robot < min_distance)
        {
            min_distance = clusters[i].distance_to_robot;
            closest_cluster = i;
        }
    }

    // Étape 5 : Retourner le cluster le plus proche
    if (closest_cluster >= 0)
    {
        blob.found = 1;
        blob.center_x = clusters[closest_cluster].center_x;
        blob.center_y = clusters[closest_cluster].center_y;
        blob.point_count = clusters[closest_cluster].point_count;

        // printf("---\r\n");
        // printf("BLOB SELECTIONNE: Cluster %d (le plus proche)\r\n", closest_cluster + 1);
        // printf("------------------\r\n");
    }
    else
    {
        // printf("---\r\n");
        // printf("AUCUN CLUSTER VALIDE (tous < %d points)\r\n", MIN_POINTS_FOR_BLOB);
        // printf("------------------\r\n");
    }

    return blob;
}

/**
 * Mettre à jour le tracking avec le blob détecté
 * Gère la confirmation, la cohérence du déplacement, et les absences
 */
static void update_tracking(struct BlobDetection blob)
{
    // ========== CAS 1 : AUCUN BLOB DÉTECTÉ ==========
    if (!blob.found)
    {
        tracker.consecutive_absences++;
        tracker.consecutive_detections = 0;

        // Si trop de scans sans détection → adversaire disparu
        if (tracker.consecutive_absences >= MAX_CONSECUTIVE_ABSENCES)
        {
            tracker.is_detected = 0;
            tracker.confidence = 0;
        }
        return;
    }

    // ========== CAS 2 : BLOB DÉTECTÉ ==========
    tracker.consecutive_absences = 0;

    // Sous-cas 2a : PREMIÈRE DÉTECTION (pas d'adversaire tracké)
    if (!tracker.is_detected)
    {
        tracker.consecutive_detections++;

        // Confirmation après MIN_CONSECUTIVE_DETECTIONS scans
        if (tracker.consecutive_detections >= MIN_CONSECUTIVE_DETECTIONS)
        {
            tracker.is_detected = 1;
            tracker.position_x = blob.center_x;
            tracker.position_y = blob.center_y;
            tracker.confidence = 80;  // Confiance initiale
        }
        // Sinon, on attend encore (en attente de confirmation)
        return;
    }

    // Sous-cas 2b : ADVERSAIRE DÉJÀ TRACKÉ → vérifier cohérence
    float dx = blob.center_x - tracker.position_x;
    float dy = blob.center_y - tracker.position_y;
    float displacement = sqrtf(dx*dx + dy*dy);

    // Vérifier que le déplacement est cohérent (pas de téléportation)
    if (displacement < MAX_POSITION_JUMP)
    {
        // ✅ Déplacement cohérent → mettre à jour
        tracker.position_x = blob.center_x;
        tracker.position_y = blob.center_y;
        tracker.consecutive_detections++;

        // Augmenter la confiance (max 100)
        if (tracker.confidence < 100)
            tracker.confidence += 5;
    }
    else
    {
        // ❌ Téléportation détectée (main qui passe ?)
        tracker.consecutive_detections = 0;
        tracker.confidence -= 20;

        // Si confiance trop basse → reset
        if (tracker.confidence < 30)
        {
            tracker.is_detected = 0;
            tracker.confidence = 0;
        }
    }
}

/**
 * Fonction principale de tracking
 * Détecte le blob et met à jour le tracking avec affichage debug
 */
void adversary_tracking(struct TerrainPoint* filtered_points, int count)
{
    // Étape 1 : Détecter le blob dans les points filtrés
    struct BlobDetection blob = detect_blob(filtered_points, count);

    // Étape 2 : Mettre à jour le tracking
    update_tracking(blob);

    // Étape 3 : AFFICHAGE DEBUG
    printf("=== ADVERSARY TRACKING ===\r\n");

    // Afficher le blob détecté (même si non confirmé)
    if (blob.found)
    {
# ifdef CLUSTER_VERBOSE
        printf("BLOB DETECTE:\r\n");
        printf("  Position: (%.0f, %.0f) mm\r\n", blob.center_x, blob.center_y);
        printf("  Nombre de points: %d\r\n", blob.point_count);
        printf("  Statut: ");

        if (tracker.is_detected)
        {
            printf("CONFIRME\r\n");
        }
        else
        {
            printf("EN ATTENTE (%d/%d scans)\r\n",
                   tracker.consecutive_detections,
                   MIN_CONSECUTIVE_DETECTIONS);
        }
# endif
    }
    else
    {
        printf("AUCUN BLOB (points < %d)\r\n", MIN_POINTS_FOR_BLOB);
    }

    printf("----------------------\r\n");

    // Afficher la position finale de l'adversaire tracké
    if (tracker.is_detected)
    {
        printf("*** ADVERSAIRE CONFIRME ***\r\n");
        printf("  Position finale: (%.0f, %.0f) mm\r\n",
               tracker.position_x,
               tracker.position_y);
        printf("  Confiance: %d%%\r\n", tracker.confidence);
        printf("  Detections consecutives: %d\r\n", tracker.consecutive_detections);
    }
    else
    {
        if (tracker.consecutive_absences > 0)
        {
            printf("Adversaire perdu (%d/%d absences)\r\n",
                   tracker.consecutive_absences,
                   MAX_CONSECUTIVE_ABSENCES);
        }
        else
        {
            printf("Aucun adversaire tracke\r\n");
        }
    }

    printf("==========================\r\n");
}

/**
 * Récupérer l'état actuel du tracking
 * Utile pour envoyer la position via CANopen
 */
struct AdversaryTracking get_adversary_state(void)
{
    return tracker;
}

/**
 * Reset manuel du tracking
 */
void reset_adversary_tracking(void)
{
    tracker.is_detected = 0;
    tracker.position_x = 0;
    tracker.position_y = 0;
    tracker.confidence = 0;
    tracker.consecutive_detections = 0;
    tracker.consecutive_absences = 0;
}
