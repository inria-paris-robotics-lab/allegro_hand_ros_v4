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

        RCLCPP_INFO(logger_, "Using CAN interface: '%s' !", can_channel_name_.c_str());

        if (info_.joints.size() != DOF_JOINTS){
            RCLCPP_FATAL(logger_, "Wrong joints number (declared in URDF (%zu)) It didn't match with the real number (%d).",info_.joints.size(), DOF_JOINTS);
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Allocating memory
        const auto num_joints = info_.joints.size();
        hw_initial_positions_.resize(DOF_JOINTS, 0.0);
        hw_states_position_.resize(num_joints, 0.0);
        hw_last_states_position_.resize(num_joints, 0.0);
        hw_states_velocity_.resize(num_joints, 0.0);
        hw_commands_effort_.resize(num_joints, 0.0);
        homing_last_position_error_.resize(num_joints, 0.0);

        // Check if every excepted interface is declared and available
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

        RCLCPP_INFO(logger_, "Collecting init_value");

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            // Get the init pose via state interface position param
            const auto& state_iface = info_.joints[i].state_interfaces[0]; // We suppos that 'position' is the first 
            
            // we search for the param
            auto it = state_iface.parameters.find("initial_value");
            if (it != state_iface.parameters.end())
            {
                hw_initial_positions_[i] = std::stod(it->second);
                RCLCPP_INFO(logger_, "  - Joint '%s': %.4f rad", info_.joints[i].name.c_str(), hw_initial_positions_[i]);
            }
            else
            {
                RCLCPP_WARN(logger_, "The param 'initial_value' is missing for the joint '%s'. Using 0.0 as default value.", info_.joints[i].name.c_str());
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
            hw_last_states_position_[i] = 0.0;
            hw_states_velocity_[i] = 0.0;
            hw_commands_effort_[i] = 0.0;
        }
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

        int retries = 0;
        rclcpp::Rate rate(1000);  // 1 ms
        while (!driver_->isInitialized())
        {
            driver_->readCANFrames();
            rate.sleep();  // Non-blocking sleep, allows ROS 2 callbacks to run.
            if (retries++ > 1000) {
                RCLCPP_FATAL(logger_, "Timeout: Nothing received from the hand");
                driver_.reset();
                return hardware_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(logger_, "Waiting init response from the hand...");
        }

        // Set starting State
        control_state_ = ControlState::HOMING;
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
        if(control_state_==ControlState::HOMING){ // Prevent controller initialization during homing and have wrong init_pos.
            for (size_t i = 0; i < hw_states_position_.size(); i++) {
                hw_last_states_position_[i] = positions_temp[i];
                hw_states_position_[i] = hw_initial_positions_[i];
                hw_states_velocity_[i] = 0.0;
            }
        }
        else{ // Normal operation
            for (size_t i = 0; i < hw_states_position_.size(); i++) {
                hw_states_position_[i] = positions_temp[i];
                hw_states_velocity_[i] = velocities_temp[i];
            }
            if (control_state_ == ControlState::EXTERNAL_CONTROL){
                hw_last_states_position_ = hw_states_position_;
            }
        }
        driver_->resetJointInfoReady();
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AllegroHandHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration & period)
    {
        // Check if there is an external command
        bool external_command_active = false;
        for(const auto& effort : hw_commands_effort_) {
            if (effort != 0.0) {
                external_command_active = true;
                break;
            }
        }
        // Declare these variables outside the switch so they are not redeclared
        double torque_cmds[DOF_JOINTS] = {0.0};
        double dt = period.seconds() > 0.0 ? period.seconds() : 0.01;

        switch (control_state_){
            case ControlState::HOMING:
                homing_in_progress_ = false; // will be set to true if any joint is not homed yet
                for (size_t i = 0; i < DOF_JOINTS; ++i)
                {
                    double position_error = hw_initial_positions_[i] - hw_last_states_position_[i];
                    double error_derivative = (position_error - homing_last_position_error_[i]) / dt;
                    double desired_torque = homing_kp_ * position_error + homing_kd_ * error_derivative;
                    constexpr double MAX_HOMING_TORQUE = 0.3;
                    torque_cmds[i] = std::clamp(desired_torque, -MAX_HOMING_TORQUE, MAX_HOMING_TORQUE);
                    homing_last_position_error_[i] = position_error;
                    // RCLCPP_INFO(logger_, "Homing Joint %lu: %f rad",i,hw_last_states_position_[i]);

                    if (std::abs(position_error) > homing_tolerance_) {
                        homing_in_progress_ = true; // At least one joint is not homed yet
                    }
                }
                driver_->setTorque(torque_cmds);
                if(!homing_in_progress_){
                    RCLCPP_INFO(logger_, "Homing finished, passing to HOLDING mode.");
                    control_state_= ControlState::HOLDING;
                }
                break;
            case ControlState::EXTERNAL_CONTROL:
                if (!external_command_active) {
                    RCLCPP_INFO(logger_, "External command stopped. Returning to HOLDING mode.");
                    control_state_ = ControlState::HOLDING;
                }
                driver_->setTorque(hw_commands_effort_.data());
                break;
            default: // HOLDING
                // If there is no external command, continue to hold the Home position
                if (external_command_active) {
                    RCLCPP_INFO(logger_, "External command detected. Switching to EXTERNAL_CONTROL mode.");
                    control_state_ = ControlState::EXTERNAL_CONTROL;
                }
                for (size_t i = 0; i < DOF_JOINTS; ++i) {
                    double position_error = hw_last_states_position_[i] - hw_states_position_[i];
                    double error_derivative = (position_error - homing_last_position_error_[i]) / dt;
                    torque_cmds[i] = homing_kp_ * position_error + homing_kd_ * error_derivative;
                    homing_last_position_error_[i] = position_error;
                }
                driver_->setTorque(torque_cmds);
                break;
        }
        // Send command
        if (driver_->writeJointTorque() != 0) {
            RCLCPP_ERROR(logger_, "Error writing torques to CAN bus.");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }


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

