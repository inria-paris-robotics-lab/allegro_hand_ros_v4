#include <allegro_hand_interface/allegro_hand_hardware_interface.hpp>
#include "allegro_hand_driver/AllegroHandDrv.h"
#include "pluginlib/class_list_macros.hpp"

constexpr int DOF_JOINTS = 16;

namespace allegro_hand_interface
{
    AllegroHandHardwareInterface::AllegroHandHardwareInterface() : logger_(rclcpp::get_logger("AllegroHandHardwareInterface")) {}
    AllegroHandHardwareInterface::~AllegroHandHardwareInterface() = default;

    hardware_interface::CallbackReturn AllegroHandHardwareInterface::on_init(const hardware_interface::HardwareInfo & info){
        
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Hardware Interface '%s' init...", info_.name.c_str());

        if (info_.hardware_parameters.find("CAN_CH") == info_.hardware_parameters.end()){
            RCLCPP_FATAL(logger_, "'CAN_CH' parameter is missing in URDF.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        can_channel_name_ = info_.hardware_parameters.at("CAN_CH");

        RCLCPP_INFO(logger_, "Using CAN interface: '%s'", can_channel_name_.c_str());

        if (info_.joints.size() != DOF_JOINTS){
            RCLCPP_FATAL(logger_, "Wrong joints number (declared in URDF (%zu)) It didn't match with the real number (%d).",info_.joints.size(), DOF_JOINTS);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Allocating memory
        const auto num_joints = info_.joints.size();
        hw_states_position_.resize(num_joints, 0.0);
        hw_states_velocity_.resize(num_joints, 0.0);
        hw_commands_effort_.resize(num_joints, 0.0);

        // CHech if every excepted interface is declared and available
        for (const auto & joint : info_.joints){
            if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "effort"){
                RCLCPP_FATAL(rclcpp::get_logger("AllegroHandHardware"), "The joint '%s' must have 1 'effort' command interface.", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.state_interfaces.size() != 2 || joint.state_interfaces[0].name != "position" || joint.state_interfaces[1].name != "velocity"){
                RCLCPP_FATAL(rclcpp::get_logger("AllegroHandHardware"), "The joint '%s' must have 2 state interface: 'position' and 'velocity'.", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(logger_, "Init complete.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn AllegroHandHardwareInterface::on_activate(const rclcpp_lifecycle::State &){
        RCLCPP_INFO(logger_, "Activating allegro hand...");

        // Init state and command
        for (size_t i = 0; i < hw_states_position_.size(); i++){
            hw_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
            hw_commands_effort_[i] = 0.0;
        }
        RCLCPP_INFO(logger_, "États matériels réinitialisés à zéro.");

        // Create driver element
        driver_ = std::make_unique<allegro::AllegroHandDrv>();
        if (!driver_){
            RCLCPP_FATAL(logger_, "Fail to create AllegroHandDrv intances.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Init comunication
        if (!driver_->init(can_channel_name_)){
            RCLCPP_FATAL(logger_, "Init can on port '%s' failed.", can_channel_name_.c_str());
            // If fail free memory
            driver_.reset();
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Waiting init response from the hand...");
        int retries = 0;
        while (!driver_->isInitialized())
        {
            driver_->readCANFrames();
            usleep(1000); 
            if (retries++ > 1000) // Timeout 1s
            {
            RCLCPP_FATAL(logger_, "Timeout: Nothing receive from the hand");
            driver_.reset();
            return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(logger_, "Hand activate succesfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> AllegroHandHardwareInterface::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++){
            // Position
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

            // Speed
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> AllegroHandHardwareInterface::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            // Torque
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type AllegroHandHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
        if (driver_->readCANFrames() != 0){ // emergency stop
            RCLCPP_ERROR(logger_, "Error durin read of can bus: Emergency Stop probably pushed.");
            return hardware_interface::return_type::ERROR;
        }
        if (!driver_->isJointInfoReady()){ // nothing new
            return hardware_interface::return_type::OK;
        }

        double positions_temp[DOF_JOINTS];
        double velocities_temp[DOF_JOINTS];
        driver_->getJointInfo(positions_temp, velocities_temp);

        for (size_t i = 0; i < hw_states_position_.size(); i++) {
            hw_states_position_[i] = positions_temp[i];
            hw_states_velocity_[i] = velocities_temp[i];
        }
        driver_->resetJointInfoReady();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AllegroHandHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
        driver_->setTorque(hw_commands_effort_.data());
        if (driver_->writeJointTorque() != 0){
            RCLCPP_ERROR(logger_, "Error during the send of torques on CAN bus.");
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    };

    hardware_interface::CallbackReturn AllegroHandHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &){
        RCLCPP_INFO(logger_, "Disable allegro hand...");
        if (driver_) {
            // Send a null torque command for safety 
            std::fill(hw_commands_effort_.begin(), hw_commands_effort_.end(), 0.0);
            driver_->setTorque(hw_commands_effort_.data());
            driver_->writeJointTorque();
            driver_.reset();
        }
        RCLCPP_INFO(logger_, "Alledro Hand disable.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

}// namespace allegro_hand_interface
PLUGINLIB_EXPORT_CLASS(allegro_hand_interface::AllegroHandHardwareInterface, hardware_interface::SystemInterface)

