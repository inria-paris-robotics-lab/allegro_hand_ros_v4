//
// Created by felixd on 10/1/15.
// Re-wrote by EtienneAr on 08/09/23
//
#pragma once

// Defines DOF_JOINTS.
#include "allegro_hand_driver/AllegroHandDrv.h"
using namespace allegro;

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

// Forward declaration.
class AllegroHandDrv;

class AllegroHWI : public hardware_interface::RobotHW {
 public:
  enum READ_STATUS { ERROR = -1, SUCCESS = 0, NOT_READY};

 public:
  AllegroHWI();
  virtual ~AllegroHWI();

  READ_STATUS read();
  void write();

 protected:
  double current_position[DOF_JOINTS] = {0.0};
  double current_velocity[DOF_JOINTS] = {0.0};
  double desired_torque[DOF_JOINTS] = {0.0};

  // CAN device
  allegro::AllegroHandDrv *canDevice;

  // Flags
  int lEmergencyStop = 0;

 private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
};

class AllegroNode {
 public:
  AllegroNode();
  virtual ~AllegroNode() = default;

  void run();

 private:
  AllegroHWI robot;
  controller_manager::ControllerManager controller_manager;
  ros::Time last_update;
};