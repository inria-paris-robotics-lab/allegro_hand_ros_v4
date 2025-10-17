#ifndef ALLEGRO_HAND_INTERFACE_HPP
#define ALLEGRO_HAND_INTERFACE_HPP
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <string.h>
#include <class_loader/class_loader.hpp>

#include "allegro_hand_driver/AllegroHandDrv.h" // Defines DOF_JOINTS

// class AllegroHandDrv;

namespace allegro_hand_interface
{  
    public:
        enum READ_STATUS { ERROR = -1, SUCCESS = 0, NOT_READY};
        AllegroHandHardwareInterface();
        ~AllegroHandHardwareInterface() override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override ;

        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    protected:
        double current_position[DOF_JOINTS] = {0.0}; on remplace le DOF_JOINT avec le info.joints.size() pour le recup 
        double current_velocity[DOF_JOINTS] = {0.0};
        double desired_torque[DOF_JOINTS] = {0.0};
        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_commands_effort_;

        // CAN device
        allegro::AllegroHandDrv *canDevice;

        // Flags
        int lEmergencyStop = 0;
}
#endif  // ALLEGRO_HAND_INTERFACE_HPP