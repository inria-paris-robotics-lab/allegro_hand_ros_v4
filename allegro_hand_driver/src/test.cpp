#include <iostream>
#include <string>
#include <vector>
#include <unistd.h> 
#include <cmath> 
#include <iomanip>

#include "allegro_hand_driver/AllegroHandDrv.h"

constexpr int DOF = 16;

int main() {
    const std::string can_channel = "can0";
    const int num_cycles = 500; 

    std::cout << "--- Test of the Allegro Hand Low-Level Driver (Read/Write) ---" << std::endl;
    std::cout << "Attempting to connect to CAN interface: " << can_channel << std::endl;

    allegro::AllegroHandDrv driver;

    if (!driver.init(can_channel)) {
        std::cerr << "[ERROR] Driver initialization failed. Please check the usual points." << std::endl;
        return -1;
    }

    std::cout << "[SUCCESS] Driver initialized. Servos are active." << std::endl;

    double current_positions[DOF] = {0.0};
    double current_velocities[DOF] = {0.0};
    double desired_torques[DOF] = {0.0};

    for (int i = 0; i < num_cycles; ++i) {
        // --- READING ---
        while (!driver.isJointInfoReady()) {
            driver.readCANFrames();
            usleep(500);
        }
        driver.getJointInfo(current_positions, current_velocities);

        if (i % 50 == 0) {
             std::cout << std::fixed << std::setprecision(3);
             std::cout << "--- READ Cycle #" << i << " ---" << std::endl;
             std::cout << "  Joint 1 Position: " << current_positions[1] << " rad" << std::endl;
             std::cout << "  Joint 1 Velocity: " << current_velocities[1] << " rad/s" << std::endl;
        }

        // --- WRITING ---
        double torque_cmd = 0.3 * sin(0.05 * i);
        desired_torques[1] = torque_cmd; 

        driver.setTorque(desired_torques);
        driver.writeJointTorque();
        
        if (i % 50 == 0) {
             std::cout << "--> Sending torque command: " << torque_cmd << " Nm to Joint 1" << std::endl;
        }

        // --- END CYCLE ---
        driver.resetJointInfoReady();
        usleep(3000); 
    }

    std::cout << "Ending tests: Sending null torque command to stop the movement" << std::endl;
    for(int j = 0; j < DOF; ++j) {
        desired_torques[j] = 0.0;
    }
    driver.setTorque(desired_torques);
    driver.writeJointTorque(); 
    usleep(3000);
    driver.writeJointTorque();

    std::cout << "Clean STOP." << std::endl;

    return 0;
}
