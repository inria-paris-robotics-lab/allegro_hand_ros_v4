#include "allegro_node.h"
#include "allegro_hand_driver/AllegroHandDrv.h"
#include <string>
#include <assert.h>

// trim from end. see http://stackoverflow.com/a/217605/256798
static inline std::string &rtrim(std::string &s)
{
    s.erase(std::find_if(
        s.rbegin(), s.rend(),
        std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

std::string jointNames[DOF_JOINTS] =
        {
                "_joint_00", "_joint_01", "_joint_02", "_joint_03",
                "_joint_04", "_joint_05", "_joint_06", "_joint_07",
                "_joint_08", "_joint_09", "_joint_10", "_joint_11",
                "_joint_12", "_joint_13", "_joint_14", "_joint_15"
        };


AllegroHWI::AllegroHWI() {
  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
  }

  // Initialize CAN device
  std::string can_ch;
  assert(ros::param::has("~comm/CAN_CH"));
  ros::param::get("~comm/CAN_CH", can_ch);
  rtrim(can_ch);  // Ensure the ROS parameter has no trailing whitespace.

  // Initialize joint name
  std::string prefix_name;
  ros::param::get("~prefix", prefix_name);
  for (int i = 0; i < DOF_JOINTS; i++) {
    jointNames[i].insert(0, prefix_name);
  }

  canDevice = new allegro::AllegroHandDrv();
  if (canDevice->init(can_ch)) {
      usleep(3000);
  }
  else {
      delete canDevice;
      canDevice = 0;
  }

  // connect and register the joint state interface
  for (int i = 0; i < DOF_JOINTS; i++) {
    const hardware_interface::JointStateHandle state_handle(jointNames[i], &current_position[i], &current_velocity[i], &desired_torque[i]);
    jnt_state_interface.registerHandle(state_handle);
  }

  registerInterface(&jnt_state_interface);

  // connect and register the joint effort interface
  for (int i = 0; i < DOF_JOINTS; i++) {
    const hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(jointNames[i]), &desired_torque[i]);
    jnt_eff_interface.registerHandle(eff_handle);
  }

  registerInterface(&jnt_eff_interface);
}

AllegroHWI::~AllegroHWI() {
  if (canDevice) delete canDevice;
}

AllegroHWI::READ_STATUS AllegroHWI::read() {
  if (!canDevice) {
    return ERROR;
  }

  // try to update joint positions through CAN comm:
  lEmergencyStop |= canDevice->readCANFrames();

  // check if all positions are updated:
  if (lEmergencyStop) {
    return ERROR;
  }

  if(!canDevice->isJointInfoReady())
  {
    return NOT_READY;
  }

  // update joint positions and velocities:
  canDevice->getJointInfo(current_position, current_velocity);

  // reset joint position update flag:
  canDevice->resetJointInfoReady();

  return SUCCESS;
}

void AllegroHWI::write() {
  if (!canDevice) {
    return;
  }

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    ROS_ERROR("Command not send : Emergency stop");
  }

  // set & write torque to each joint:
  canDevice->setTorque(desired_torque);
  lEmergencyStop |= canDevice->writeJointTorque();
}

AllegroNode::AllegroNode()
: robot(),
  controller_manager(&robot),
  last_update(0)
{
}

void AllegroNode::run() {
  while (ros::ok())
  {
    AllegroHWI::READ_STATUS s = robot.read();

    if(s == AllegroHWI::READ_STATUS::ERROR) {
      ROS_ERROR("Error while reading hand info, controller exiting.");
      break;
    }

    if(s == AllegroHWI::READ_STATUS::NOT_READY) {
      continue;
    }

    if(s == AllegroHWI::READ_STATUS::SUCCESS) {
      const ros::Time now = ros::Time::now();
      controller_manager.update(now, now - last_update);
      robot.write();
      last_update = now;
    }
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "allegro_node");

  // Ros spin asynchronously for controller_manager service callbacks, on 2 threads.
  ros::AsyncSpinner spinner(2);
  spinner.start();

  AllegroNode node;
  node.run();
  return -1;
}