#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Giacomo Picardi

import os, sys, time, signal
import numpy as np
import matplotlib.pyplot as plt

fileDir = os.path.dirname(os.path.abspath(__file__))
parentDir = os.path.dirname(fileDir)
path = sys.path
path.append(parentDir)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import robot

should_quit = False
def handler(signum, frame):
    global should_quit
    should_quit = True


# -retrieve arguments from command line (omnidirectional_gait parameters + number of leg cycles)
if len(sys.argv) != 12:
        print("Usage: python3 omnidirectional_gait.py <gait_width> <gait_height> <direction> <step_length> <duty_cycle> <ground_clearance> <phase_lag> <nstep> <period> <rotation> <nleg_cycles>")
        sys.exit(1)
_, gait_width, gait_height, direction, step_length, duty_cycle, ground_clearance, phase_lag_str, nstep, period, rotation, nleg_cycles = sys.argv

class OmnidirectionalGaitController(Node):

    def __init__(self):
        super().__init__('omnidirectional_gait_controller')
         # -convert cmd line arguments from string to the appropriate type
        self.gait_width = float(gait_width)
        self.gait_height = float(gait_height)
        self.direction = float(direction)
        self.step_length = float(step_length)
        self.duty_cycle = float(duty_cycle)
        self.ground_clearance = float(ground_clearance)
        self.phase_lag = np.array([float(x) for x in phase_lag_str.split(',')])
        self.nstep = int(nstep)
        self.period = float(period)
        self.rotation = int(rotation)
        self.nleg_cycles = int(nleg_cycles)

        #instatiate robot object
        self.robot = robot.Robot()

        # -subscribe to joint_states, retrieve current_positions and format it for robot.change_configuration function
        # this is needed because for some reason /joint_state and /dynamic_joint_state change the order of joints and they must be remapped
        self.joint_order = [
            'coxa_joint_0', 'femur_joint_0', 'tibia_joint_0',
            'coxa_joint_1', 'femur_joint_1', 'tibia_joint_1',
            'coxa_joint_2', 'femur_joint_2', 'tibia_joint_2',
            'coxa_joint_3', 'femur_joint_3', 'tibia_joint_3',
            'coxa_joint_4', 'femur_joint_4', 'tibia_joint_4',
            'coxa_joint_5', 'femur_joint_5', 'tibia_joint_5'
        ]
        self.Q_current = robot.static_poses_pos['dragon']
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_subscriber_callback, 10)

        # -create publisher to send angular positions to the topic of the controller
        #forward_pos_publisher = rclpy.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pid_pos_publisher = self.create_publisher(Float64MultiArray, '/pid_position_controller/commands', 10)

        # -calculate the omnidirectional_gait_trajectories for the parameters
        self.T_omni = np.zeros([18,int(nstep)])
        self.Q_omni = np.zeros([18,int(nstep)])
        self.Admiss_omni = [False]*6
        for leg_id in range(0,6):
            _, self.Q_omni[3*leg_id:3*leg_id+3,:], _, self.Admiss_omni[leg_id] = \
                self.robot.trj_gen(self.gait_width, self.gait_height, self.direction, self.step_length, self.duty_cycle, \
                                   self.ground_clearance, self.phase_lag[leg_id], self.nstep, self.period, leg_id, self.rotation)
        feasible_omni = all(self.Admiss_omni)
        if feasible_omni == 0:
             print("Specified trajectory for omnidirectional gait is outside of the leg workspace")
             raise ValueError("Specified trajectory for omnidirectional gait is outside of the leg workspace")


    def joint_state_subscriber_callback(self, msg):
        #self.Q_current = np.array(msg.position)
        #remap the joint to correct order
        joint_position_dict = dict(zip(msg.name, msg.position))

        # Update the NumPy array according to the desired joint order
        for i, joint_name in enumerate(self.joint_order):
            if joint_name in joint_position_dict:
                self.Q_current[i] = joint_position_dict[joint_name]
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in message")

    # -robot.change_configuration_loop
    def change_configuration_loop(self):
        # -calculate the trajectories for the robot.change_configuration to transition to first pose of omnidirectional_gait.
        # This (unlike the omnidirectional gait trajectory calculation) is done inside the loop to allow the subscriber to get Q_cur
        self.Q_cc, _, self.Admiss_cc, self.nstep_cc, ctrl_timestep = self.robot.change_configuration(self.Q_omni[:,0], self.Q_current)
        feasible_cc = all(self.Admiss_cc)
        if feasible_cc == 0:
            print("Specified trajectory for configuration change is outside of the leg workspace")
            raise ValueError("Specified trajectory for configuration change is outside of the leg workspace")
        
        for i in range(self.nstep_cc):
            if should_quit:
                break
            msg = Float64MultiArray()
            tmp = self.Q_cc[:,i]
            tmp = tmp.tolist()
            msg.data = tmp
            #forward_pos_publisher.publish(msg) #hardware
            self.pid_pos_publisher.publish(msg) #sim
            time.sleep(ctrl_timestep)
         

    # -omnidirectional_gait loop
    def omni_loop(self):
        ctrl_timestep = self.period/self.nstep
        for i in range(self.nstep*self.nleg_cycles):
            if should_quit:
                break
            msg = Float64MultiArray()
            tmp = self.Q_omni[:,i%self.nstep]
            tmp = tmp.tolist()
            msg.data = tmp
            #forward_pos_publisher.publish(msg) #hardware
            self.pid_pos_publisher.publish(msg) #sim
            time.sleep(ctrl_timestep)

    def plot_joint_trajectories(self):
        """
        Plot the joint trajectories from self.Q_cc, where each row represents
        a joint and each column represents a time step.
        """
        num_joints, num_time_steps = self.Q_cc.shape

        # Create a time array (assuming evenly spaced time steps)
        time = np.arange(num_time_steps)

        # Create the plot
        plt.figure(figsize=(10, 6))
        
        # Plot each joint trajectory
        for joint_idx in range(3): #num_joints
            plt.plot(time, self.Q_cc[joint_idx, :], label=f'Joint {joint_idx + 1}')
        
        # Add labels and title
        plt.xlabel('Time Steps')
        plt.ylabel('Joint Position')
        plt.title('Joint Trajectories of the Robot')
        plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
        plt.grid(True)

        # Show the plot
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    rclpy.init()

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    try:
        gait_controller = OmnidirectionalGaitController()
    except ValueError:
        sys.exit(1)

    rclpy.spin_once(gait_controller)

    gait_controller.change_configuration_loop()


    gait_controller.omni_loop()

    rclpy.shutdown()