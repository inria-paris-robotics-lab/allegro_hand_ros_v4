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
        hw_states_velocity_.resize(num_joints, 0.0);
        hw_commands_effort_.resize(num_joints, 0.0);
        homing_last_position_error_.resize(num_joints, 0.0);

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

        RCLCPP_INFO(logger_, "Collecting init_value");

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            // Le paramètre 'initial_value' est dans l'interface d'état 'position'
            const auto& state_iface = info_.joints[i].state_interfaces[0]; // On suppose que 'position' est la première
            
            // On cherche le paramètre
            auto it = state_iface.parameters.find("initial_value");
            if (it != state_iface.parameters.end())
            {
                // On convertit la chaîne de caractères en double et on la stocke
                hw_initial_positions_[i] = std::stod(it->second);
                RCLCPP_INFO(logger_, "  - Articulation '%s': %.4f rad", info_.joints[i].name.c_str(), hw_initial_positions_[i]);
            }
            else
            {
                RCLCPP_WARN(logger_, "Le paramètre 'initial_value' est manquant pour l'articulation '%s'. Utilisation de 0.0.", info_.joints[i].name.c_str());
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

        RCLCPP_INFO(logger_, "Matériel activé. La procédure de Homing va commencer dans la boucle de contrôle.");

        // Fréquence de contrôle cible pour notre boucle manuelle (ex: 200 Hz)
        const auto control_period = std::chrono::microseconds(1000000 / 300);
        auto last_update_time = std::chrono::high_resolution_clock::now();
        bool is_homed = false;
        int homing_cycle_count = 0;

        while (!is_homed) // La boucle continue tant que le homing n'est pas terminé
        {   
            RCLCPP_INFO(logger_, "Homing en cours...");
            // Sécurité pour éviter une boucle infinie en cas de problème et informer l'utilisateur
            if (homing_cycle_count > 2000) { // Timeout après 10 secondes à 200Hz
                RCLCPP_WARN(logger_, "Le Homing prend plus de 10 secondes. Vérifiez les gains ou un éventuel blocage physique.");
                // On pourrait décider d'arrêter ici, mais on continue pour l'instant
                // return hardware_interface::CallbackReturn::ERROR;
                homing_cycle_count = 0; // Réinitialiser pour éviter de spammer
            }

            // --- Étape 1: Lire l'état actuel ---
            driver_->readCANFrames();
            if (driver_->isJointInfoReady())
            {
                double positions_temp[DOF_JOINTS], velocities_temp[DOF_JOINTS];
                driver_->getJointInfo(positions_temp, velocities_temp);
                for(size_t i=0; i<DOF_JOINTS; ++i) {
                    hw_states_position_[i] = positions_temp[i];
                    hw_states_velocity_[i] = velocities_temp[i];
                }
                driver_->resetJointInfoReady();
            }

            // --- Étape 2: Calculer la commande PD et vérifier la condition de fin ---
            double torque_cmds[DOF_JOINTS] = {0.0};
            is_homed = true; // On suppose que c'est fini, et on cherche une preuve du contraire
            double dt = std::chrono::duration<double>(control_period).count();

            for (size_t i = 0; i < DOF_JOINTS; ++i)
            {
                double position_error = hw_initial_positions_[i] - hw_states_position_[i];
                double error_derivative = (position_error - homing_last_position_error_[i]) / dt;
                
                double desired_torque = homing_kp_ * position_error + homing_kd_ * error_derivative;
                constexpr double MAX_HOMING_TORQUE = 0.3;
                torque_cmds[i] = std::clamp(desired_torque, -MAX_HOMING_TORQUE, MAX_HOMING_TORQUE);

                homing_last_position_error_[i] = position_error;

                RCLCPP_INFO(logger_, "Homing Joint %lu: error %f pos %f rad",i,position_error,hw_states_position_[i]);
                
                if (std::abs(position_error) > homing_tolerance_) {
                    is_homed = false; // Une articulation n'est pas arrivée, on doit continuer
                }
            }

            // --- Étape 3: Envoyer la commande ---
            driver_->setTorque(torque_cmds);
            if (driver_->writeJointTorque() != 0) {
                RCLCPP_ERROR(logger_, "Erreur lors de l'écriture du couple pendant le homing.");
                return hardware_interface::CallbackReturn::ERROR; // Erreur critique, on arrête tout
            }

            // --- Étape 4: Attendre la fin du cycle de contrôle ---
            last_update_time += control_period;
            std::this_thread::sleep_until(last_update_time);
            
            homing_cycle_count++;
        }
        // =========================================================================
        //                 FIN DE LA BOUCLE DE HOMING BLOQUANTE
        // =========================================================================

        RCLCPP_INFO(logger_, "Homing bloquant terminé avec succès.");

        // Mettre à jour l'état final une dernière fois
        driver_->readCANFrames();
        if(driver_->isJointInfoReady()){
            double p[DOF_JOINTS], v[DOF_JOINTS];
            driver_->getJointInfo(p, v);
            for(size_t i=0; i<DOF_JOINTS; ++i){
                hw_states_position_[i] = p[i];
                hw_states_velocity_[i] = v[i];
            }
            hw_last_states_position_ = hw_states_position_;
            driver_->resetJointInfoReady();
        }

        control_state_ = ControlState::HOLDING;

        RCLCPP_INFO(logger_, "Hand activée avec succès. Le ControllerManager peut démarrer.");
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
        if (control_state_ == ControlState::EXTERNAL_CONTROL){
            hw_last_states_position_ = hw_states_position_;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type AllegroHandHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration & period)
    {
        
        // On vérifie d'abord si une commande externe est arrivée
        bool external_command_active = false;
        for(const auto& effort : hw_commands_effort_) {
            if (effort != 0.0) {
                external_command_active = true;
                break;
            }
        }

        // --- MACHINE À ÉTATS ---
        if (control_state_ == ControlState::HOLDING)
        {
            // --- PHASE 2: MAINTENIR LA POSITION "HOME" ---
            if (external_command_active) {
                RCLCPP_INFO(logger_, "Commande externe détectée. Passage en mode EXTERNAL_CONTROL.");
                control_state_ = ControlState::EXTERNAL_CONTROL;
                // On passe directement au cas suivant
            } else {
                // S'il n'y a pas de commande externe, on continue de maintenir la position Home
                double torque_cmds[DOF_JOINTS] = {0.0};
                double dt = period.seconds() > 0.0 ? period.seconds() : 0.01;

                for (size_t i = 0; i < DOF_JOINTS; ++i) {
                    RCLCPP_INFO(logger_, "Holding Joint %lu: %f rad",i,hw_last_states_position_[i]);
                    double position_error = hw_last_states_position_[i] - hw_states_position_[i];
                    double error_derivative = (position_error - homing_last_position_error_[i]) / dt;
                    torque_cmds[i] = homing_kp_ * position_error + homing_kd_ * error_derivative;
                    homing_last_position_error_[i] = position_error;
                }
                driver_->setTorque(torque_cmds);
            }
        }

        if (control_state_ == ControlState::EXTERNAL_CONTROL)
        {
            // --- PHASE 3: SUIVRE LES COMMANDES EXTERNES ---
            driver_->setTorque(hw_commands_effort_.data());
            
            // Optionnel: Revenir en mode HOLDING si la commande externe s'arrête
            if (!external_command_active) {
                RCLCPP_INFO(logger_, "La commande externe s'est arrêtée. Retour en mode HOLDING.");
                control_state_ = ControlState::HOLDING;
            }
        }
        
        // Envoyer la commande de couple (soit du PD interne, soit du contrôleur externe) au matériel
        if (driver_->writeJointTorque() != 0) {
            RCLCPP_ERROR(logger_, "Erreur lors de l'écriture des couples sur le bus CAN.");
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

