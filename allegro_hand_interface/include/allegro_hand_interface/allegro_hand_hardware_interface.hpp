#ifndef ALLEGRO_HAND_HARDWARE_INTERFACE_HPP
#define ALLEGRO_HAND_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <class_loader/class_loader.hpp>
namespace allegro {
  class AllegroHandDrv;
}

namespace allegro_hand_interface{
    class AllegroHandHardwareInterface : public hardware_interface::SystemInterface{
    public:
        AllegroHandHardwareInterface();
        ~AllegroHandHardwareInterface() override;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private: 
        
        std::unique_ptr<allegro::AllegroHandDrv> driver_;

        std::vector<double> hw_states_position_;
        std::vector<double> hw_states_velocity_;
        std::vector<double> hw_commands_effort_;

        std::string can_channel_name_;
        
        rclcpp::Logger logger_;
    };

} 

#endif  // ALLEGRO_HAND_HARDWARE_INTERFACE_HPP