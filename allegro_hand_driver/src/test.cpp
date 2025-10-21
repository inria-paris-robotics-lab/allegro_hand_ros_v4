#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>  // Pour usleep()
#include <cmath>     // Pour sin()
#include <iomanip>   // Pour std::fixed, std::setprecision
#include <chrono>    // Pour la mesure précise du temps

// Inclure directement l'en-tête de votre driver bas-niveau
#include "allegro_hand_driver/AllegroHandDrv.h"

// Définir le nombre d'articulations pour la clarté
constexpr int DOF = 16;

int main() {
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    const std::string can_channel = "can0";
    const int num_cycles_to_run = 1000;

    std::cout << "--- Test de Diagnostic du Driver Bas-Niveau (Lecture/Écriture/Timing) ---" << std::endl;
    std::cout << "Tentative de connexion sur l'interface CAN : " << can_channel << std::endl;

    // =========================================================================
    // 1. Instancier et Initialiser le Driver
    // =========================================================================
    allegro::AllegroHandDrv driver;

    if (!driver.init(can_channel)) {
        std::cerr << "[ERREUR] Échec de l'initialisation du driver. Vérifiez les points habituels." << std::endl;
        return -1;
    }

    std::cout << "[SUCCÈS] Driver initialisé. Les servos sont actifs." << std::endl;

    // Tableaux pour les données
    double current_positions[DOF] = {0.0};
    double current_velocities[DOF] = {0.0};
    double desired_torques[DOF] = {0.0};

    // =========================================================================
    // 2. Boucle de Diagnostic
    // =========================================================================
    // Initialiser le point de départ pour la mesure de temps du premier cycle
    auto last_cycle_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_cycles_to_run; ++i) {
        
        // --- ÉTAPE DE LECTURE (CHRONOMÉTRÉE) ---
        auto read_start_time = std::chrono::high_resolution_clock::now();
        int read_loops = 0;
        
        // On attend de recevoir les données des 4 doigts
        while (!driver.isJointInfoReady()) {
            usleep(500); // Petite pause de 0.5ms pour ne pas saturer le CPU
            read_loops++;
        }
        auto read_end_time = std::chrono::high_resolution_clock::now();
        // --- FIN DE LA LECTURE ---

        driver.getJointInfo(current_positions, current_velocities);
        
        // --- ÉTAPE DE CALCUL ET D'ÉCRITURE ---
        double torque_cmd = 0.3 * sin(0.01 * i);
        desired_torques[1] = torque_cmd;
        driver.setTorque(desired_torques);
        driver.writeJointTorque();

        driver.resetJointInfoReady();
        
        // --- MESURE ET AFFICHAGE DES DURÉES ---
        auto current_cycle_time = std::chrono::high_resolution_clock::now();
        
        // Calcul des durées en microsecondes
        auto cycle_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(current_cycle_time - last_cycle_time).count();
        auto read_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(read_end_time - read_start_time).count();
        
        // Mettre à jour le temps pour le prochain cycle
        last_cycle_time = current_cycle_time;
        
        // On affiche les informations de timing tous les 10 cycles pour la lisibilité
        if (i > 0 && i % 10 == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "\n--- Cycle #" << i << " ---" << std::endl;
            std::cout << "  Durée Totale du Cycle Précédent : " << cycle_duration_us << " us (" << cycle_duration_us / 1000.0 << " ms)" << std::endl;
            std::cout << "  Temps d'Attente des Données CAN : " << read_duration_us << " us (" << read_duration_us / 1000.0 << " ms) avec " << read_loops << " tentatives." << std::endl;
            std::cout << "  Position (Art. 1): " << current_positions[1] << " rad" << std::endl;
        }

        // On retire l'usleep fixe pour mesurer la vitesse naturelle de la boucle.
        // Si la boucle est trop rapide, on peut remettre un petit usleep ici.
        usleep(10000); 
    }

    // =========================================================================
    // 3. SÉCURITÉ : Arrêt propre
    // =========================================================================
    std::cout << "Test terminé. Envoi d'une commande de couple nulle pour arrêter le mouvement." << std::endl;
    for(int j=0; j<DOF; ++j) {
        desired_torques[j] = 0.0;
    }
    driver.setTorque(desired_torques);
    driver.writeJointTorque();
    usleep(3000);
    driver.writeJointTorque();

    std::cout << "Arrêt propre." << std::endl;

    return 0;
}