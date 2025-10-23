#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <cmath>
#include <iomanip>
#include <chrono>

#include "allegro_hand_driver/AllegroHandDrv.h"

constexpr int DOF = 16;

int main() {
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    const std::string can_channel = "can0";
    const int num_cycles_to_run = 1000;

    std::cout << "--- Low-Level Driver Diagnostic Test (Read/Write/Timing) ---" << std::endl;
    std::cout << "Attempting to connect to CAN interface: " << can_channel << std::endl;

    // =========================================================================
    // 1. Instancier et Initialiser le Driver
    // =========================================================================
    allegro::AllegroHandDrv driver;

    if (!driver.init(can_channel)) {
        std::cerr << "[ERROR] Failed to initialize driver. Check usual points." << std::endl;
        return -1;
    }

    std::cout << "[SUCCESS] Driver initialized. Servos are active." << std::endl;

    // Arrays for data
    double current_positions[DOF] = {0.0};
    double current_velocities[DOF] = {0.0};
    double desired_torques[DOF] = {0.0};

    // =========================================================================
    // 2. Diagnostic Loop
    // =========================================================================
    // Initialize the starting point for timing the first cycle
    auto last_cycle_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_cycles_to_run; ++i) {
        
        // --- TIMED READ STEP ---
        auto read_start_time = std::chrono::high_resolution_clock::now();
        int read_loops = 0;
        
        while (!driver.isJointInfoReady()) {
            usleep(500); 
            read_loops++;
        }
        auto read_end_time = std::chrono::high_resolution_clock::now();

        driver.getJointInfo(current_positions, current_velocities);
        
        double torque_cmd = 0.3 * sin(0.01 * i);
        desired_torques[1] = torque_cmd;
        driver.setTorque(desired_torques);
        driver.writeJointTorque();

        driver.resetJointInfoReady();
        
        auto current_cycle_time = std::chrono::high_resolution_clock::now();
        
        auto cycle_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(current_cycle_time - last_cycle_time).count();
        auto read_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(read_end_time - read_start_time).count();

        last_cycle_time = current_cycle_time;
        
        if (i > 0 && i % 10 == 0) {
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "\n--- Cycle #" << i << " ---" << std::endl;
            std::cout << "  Durée Totale du Cycle Précédent : " << cycle_duration_us << " us (" << cycle_duration_us / 1000.0 << " ms)" << std::endl;
            std::cout << "  Temps d'Attente des Données CAN : " << read_duration_us << " us (" << read_duration_us / 1000.0 << " ms) avec " << read_loops << " tentatives." << std::endl;
            std::cout << "  Position (Art. 1): " << current_positions[1] << " rad" << std::endl;
        }

        usleep(10000); 
    }

    // =========================================================================
    // 3. Security Shutdown
    // =========================================================================
    std::cout << "Test completed. Sending a zero torque command to stop motion." << std::endl;
    for(int j=0; j<DOF; ++j) {
        desired_torques[j] = 0.0;
    }
    driver.setTorque(desired_torques);
    driver.writeJointTorque();
    usleep(3000);
    driver.writeJointTorque();

    return 0;
}