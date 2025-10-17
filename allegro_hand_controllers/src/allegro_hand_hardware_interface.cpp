#include <allegro_hand_controllers/allegro_hand_hardware_interface.hpp>

namespace allegro_hand_interface
{  
    hardware_interface::CallbackReturn AllegroHandHardware::on_init(const hardware_interface::HardwareInfo & info){
        
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Récupérer les paramètres depuis l'URDF (équivalent de ros::param::get en ROS 1)
        can_channel_name_ = info_.hardware_parameters.at("can_channel")
        RCLCPP_INFO(rclcpp::get_logger("AllegroHandHardware"), "Paramètre 'CAN_CH' = %s", can_channel_name_.c_str());

        // Allouer la mémoire pour nos tampons de données
        const auto num_joints = info_.joints.size();
        hw_states_position_.resize(num_joints, 0.0);
        hw_states_velocity_.resize(num_joints, 0.0);
        hw_commands_effort_.resize(num_joints, 0.0);

        // Vérifier que toutes les interfaces attendues sont présentes
        for (const auto & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "effort")
            {
            RCLCPP_FATAL(rclcpp::get_logger("AllegroHandHardware"), "L'articulation '%s' doit avoir exactement une interface de commande de type 'effort'.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2 || joint.state_interfaces[0].name != "position" || joint.state_interfaces[1].name != "velocity")
            {
            RCLCPP_FATAL(rclcpp::get_logger("AllegroHandHardware"), "L'articulation '%s' doit avoir les interfaces d'état 'position' et 'velocity'.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }


}// namespace allegro_hand_interface