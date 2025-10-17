#!/bin/env python

import rospy
from joint_group_ff_controllers.msg import setpoint
from sensor_msgs.msg import JointState
from copy import copy
from math import sin

q0 = [0, 0.2, 0, 0] * 4
q0[12] = 0.6 # Thumb first joint cannot go to 0
v0 = [0] * 16
tau0 = [0] * 16

class MotionPlayer:
    def __init__(self, hand_name):
        self.hand_name = hand_name

    def wave(self, duration, waved_joints, offsets = None, wave_period=1.5, wave_amplitude=0.4, wave_ramp_time=10):
        cmd_freq = 100 #Hz

        if(not offsets):
            offsets = [i/len(waved_joints) for i in range(len(waved_joints))]

        # Prepare ros cmd msg
        pub = rospy.Publisher(f"/{self.hand_name}/joint_ff_controller/command", setpoint, queue_size=1)
        msg = setpoint()
        msg.timeout = rospy.Time.from_sec(3. / cmd_freq) # Can miss 3 deadlines
        msg.positions = q0
        msg.velocities = v0
        msg.efforts = tau0

        # Init values
        q_start = rospy.wait_for_message(f"/{self.hand_name}/joint_states", JointState).position
        q = copy(q0)
        q_command = copy(q_start)
        wave_start_time = rospy.Time.now().to_sec()
        wave_rate = rospy.Rate(cmd_freq)

        # Actual ctrl loop
        while((rospy.Time.now().to_sec() - wave_start_time) < duration and not rospy.is_shutdown()):
            t = rospy.Time.now().to_sec()

            # For each joint
            for i in range(len(waved_joints)):
                joint = waved_joints[i]
                phi = (t/wave_period + offsets[i]) * 2 * 3.141592
                q[joint] = q0[joint] + wave_amplitude / 2. * sin(phi)

            # Progressively go from the start q to the desired q
            scale_factor = (t-wave_start_time) / wave_ramp_time
            scale_factor = min(scale_factor, 1.0)
            q_command = [q_start[i] * (1-scale_factor) + q[i] * scale_factor for i in range(len(q))]

            # Send command for the whole hand
            msg.positions = q_command
            pub.publish(msg)
            wave_rate.sleep()


if __name__ == "__main__":
    rospy.init_node("wave")
    mp = MotionPlayer(rospy.get_param("~hand_name"))
    while(not rospy.is_shutdown()):
        print("Executing pattern one...")
        mp.wave(30, waved_joints=[12,13,14,15, 1,2,3,0, 5,6,7,4 ,9,10,11,8], wave_period=3)
        print("Executing pattern two...")
        mp.wave(30, waved_joints=[0,1,2])
        print("Executing pattern three...")
        mp.wave(30, waved_joints=[12,1,5,9], offsets=[0, -0.2, -0.3, -0.4])