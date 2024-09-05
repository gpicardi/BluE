#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Giacomo Picardi

import threading
import time
import signal
import os, sys
import numpy as np

fileDir = os.path.dirname(os.path.abspath(__file__))
parentDir = os.path.dirname(fileDir)
path = sys.path
path.append(parentDir)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from std_msgs.msg import Float64MultiArray

from silver_interfaces.srv import GaitSelector

import robot

should_quit = False
def handler(signum, frame):
    global should_quit
    should_quit = True

V0 = 9*np.ones([18], dtype='int')

Vslow = 1*np.ones([18], dtype='int')
dT = 2.5

class LocomotionController(Node):

    def __init__(self):

        # init robot
        self.robot = robot.Robot()

        #state variables
        self.gait_type = None
        self.gait_go = False

        self.cc = 0 #ctrl loop iteration counter (use it to take action at multiples of loop frequency)
        # gait parameters
        self.ctrl_timestep = 0.03
        # static gait parameters
        self.static_gait_params = None
        # dynamic gait parameters
        self.dynamic_gait_params = None
        self.dynamic_gait_cmd = None
        # rotate gait parameters
        self.rotate_gait_params = None
        self.Rdirection = None
        # maximum time to complete pushing or pulling action, set it to Infinity to eliminate the open loop base layer
        self.t_max_pull = 10 #maximum allowed duration of swimming phase
        self.t_max_push = 10 #maximum allowed duration of punting phase
        #minimum time to complete pushing or pulling action, set it non zero in case of piezo multipeaks
        self.t_min_pull = 5 # if 0 condition for td event is always true
        self.t_min_push = 5 # if 0 condition for lo event is always true
        ####################

        # create services
        super().__init__('locomotion_node')
        self.set_gait_proxy = self.create_service(GaitSelector, 'silver_locomotion/set_gait', self.set_gait)
        self.gait_start_proxy = self.create_service(Empty, 'silver_locomotion/gait_start', self.gait_start)
        self.gait_stop_proxy = self.create_service(Empty, 'silver_locomotion/gait_stop', self.gait_stop)

        #create publishers
        #self.forward_pos_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.forward_pos_publisher = self.create_publisher(Float64MultiArray, '/pid_position_controller/commands', 10)

        # static shared variables
        self.leg_ids = np.array([1,2,3,4,5,6])
        self.iteration_counter = 0
        self.T = np.zeros([18,120])
        self.Q = np.zeros([18,120])
        self.Qdot = np.zeros([18,120])
        # dynamic shared variable
        self.joint_vel_push = 0
        self.joint_vel_pull = 70*np.ones(6,dtype="int")
        self.q_femur_pull = np.zeros(6)
        self.q_femur_push = np.zeros(6)
        self.q_tibia_pull = np.zeros(6)
        self.q_tibia_push = np.zeros(6)
        self.q_coxa_pull = np.zeros(6)
        self.ts= 0
        self.vl_no = 1
        self.underactuated = 0
        self.rotation = 0
        self.feet_policy = "none"

        #go to base pose
        msg = Float64MultiArray()
        tmp =  robot.static_poses_pos["dragon"]
        print(tmp)
        tmp = tmp.tolist()
        msg.data = tmp
        self.forward_pos_publisher.publish(msg)

    def loop(self):
        rosthread = threading.Thread(target=self.ros_spin)
        rosthread.deamon = True
        rosthread.start()
        while True:

            if should_quit:
                break
            if self.gait_go:

                if self.gait_type == 'static':
                    msg = Float64MultiArray()
                    tmp =  self.Q[:,self.iteration_counter]
                    print(tmp)
                    tmp = tmp.tolist()
                    msg.data = tmp
                    self.forward_pos_publisher.publish(msg)
                    self.iteration_counter = (self.iteration_counter+1)%int(self.static_gait_params['nstep'])

                elif self.gait_type == 'dynamic':
                    print('dynamic gaits have not been implemented yet')
                else:
                    print('no gait has been selected')
            time.sleep(self.ctrl_timestep)
        if self.gait_go:
            self.gait_stop(None, None)

        self.robot.__del__()

    def ros_spin(self):
        rclpy.spin(self)

    def set_gait(self, req, response):

        print('set_gait')

        if self.gait_go:
            response.success = False
            response.status_message = 'cannot set gait while moving'
            return response

        if req.type == 'static':
            self.gait_type = 'static'
            self.static_gait_params = {
                'rotation': req.rotation,
                'gait_width': req.gait_width,
                'gait_height': req.gait_height,
                'direction': req.direction,
                'step_length': req.step_length,
                'duty_cycle': req.duty_cycle,
                'ground_clearance': req.ground_clearance,
                'phase_lag': np.array(req.phase_lag),
                'nstep': req.nstep,
                'period': req.period
                }
            # check params
            if self.static_gait_params['rotation']  < -1 or self.static_gait_params['rotation'] >1:
                self.static_gait_params['rotation'] = 0
            if self.static_gait_params['gait_width']<= 25 or self.static_gait_params['gait_width']> 60:
                self.static_gait_params['gait_width'] = 40
            if self.static_gait_params['gait_height'] <= 0:
                self.static_gait_params['gait_height'] = 40
            if self.static_gait_params['direction'] < -180 or self.static_gait_params['direction']>180:
                self.static_gait_params['direction'] = 90
            if self.static_gait_params['step_length'] <= 0:
                self.static_gait_params['step_length'] = 20
            if self.static_gait_params['duty_cycle'] <= 0 or self.static_gait_params['duty_cycle'] >= 1:
                self.static_gait_params['duty_cycle'] = 0.8
            if self.static_gait_params['ground_clearance'] <= 0:
                self.static_gait_params['ground_clearance'] = 10
            if self.static_gait_params['phase_lag'].size < 6:
                self.static_gait_params['phase_lag'] = np.array([90, 0, 90, 0, 90, 0])
            if self.static_gait_params['nstep'] <= 0:
                self.static_gait_params['nstep'] = 120
            if self.static_gait_params['period'] <= 0:
                self.static_gait_params['period'] = 10

            try:
                self.ctrl_timestep = self.static_gait_params['period'] / self.static_gait_params['nstep']
            except ZeroDivisionError:
                self.ctrl_timestep = 0.03
            # compute trajectory, check feasibility and apply inverse kinematics
            self.T = np.zeros([18,int(self.static_gait_params['nstep'])])
            self.Q = np.zeros([18,int(self.static_gait_params['nstep'])])
            self.Qdot = np.zeros([18,int(self.static_gait_params['nstep'])])
            Admiss = [False]*6
            for leg_id in range(0,6):
                self.T[3*leg_id:3*leg_id+3,:], self.Q[3*leg_id:3*leg_id+3,:], self.Qdot[3*leg_id:3*leg_id+3,:], Admiss[leg_id] = self.robot.trj_gen(self.static_gait_params['gait_width'], self.static_gait_params['gait_height'],self.static_gait_params['direction'],self.static_gait_params['step_length'],\
                                                                                                                          self.static_gait_params['duty_cycle'],self.static_gait_params['ground_clearance'],self.static_gait_params['phase_lag'][leg_id], int(self.static_gait_params['nstep']),self.static_gait_params['period'], leg_id, self.static_gait_params['rotation'])
            feasible = all(Admiss)
            if feasible:
                # goto initial position
                #self.robot.move_all_legs(Vslow, self.Q[:,0])
                #self.robot.change_configuration(self.Q[:,0])
                msg = Float64MultiArray()
                tmp =  self.Q[:,0]
                tmp = tmp.tolist()
                msg.data = tmp
                self.forward_pos_publisher.publish(msg)
                response.success = True
                response.status_message = 'Static gait successfully set.'
                return response
            else:
                self.gait_type = None
                response.success = False
                response.status_message = 'Failed to set static gait. Non feasible leg trajectory.'
                return response

        elif req.type == 'dynamic':
            self.gait_type = 'dynamic'
            #retrieve params from req
            self.dynamic_gait_params = {
            'joint_vel': req.joint_vel,
            'alpha': req.alpha,
            'td_height': req.td_height,
            'underactuated': req.underactuated,
            'vl_no': req.vl_no,
            'rotation': req.rotation,
            'feet_policy': req.feet_policy
            }
            #check and possibly assign to default
            if self.dynamic_gait_params['joint_vel']<= 0 or self.dynamic_gait_params['joint_vel']> 4000:
                self.dynamic_gait_params['joint_vel'] = 200
            if self.dynamic_gait_params['alpha']<=0 or self.dynamic_gait_params['alpha']>180 or self.dynamic_gait_params['alpha'] == 90:
                self.dynamic_gait_params['alpha'] = 89.9
            if self.dynamic_gait_params['td_height']<=0 or self.dynamic_gait_params['td_height']>60:
                self.dynamic_gait_params['td_height'] = 50
            if self.dynamic_gait_params['vl_no'] < 1 or self.dynamic_gait_params['vl_no'] >2:
                self.dynamic_gait_params['vl_no'] = 1
            if self.dynamic_gait_params['rotation']  < -1 or self.dynamic_gait_params['rotation'] >1:
                self.dynamic_gait_params['rotation'] =0

            self.dynamic_gait_params['alpha'] = self.dynamic_gait_params['alpha']*np.pi/180 #conversions of alpha angle and setting of speed
            self.joint_vel_push = self.dynamic_gait_params['joint_vel']*np.ones(6, dtype="int") # create array of joint velocities (all equal) per il caso equal
            self.joint_vel_pull = 0.5*self.dynamic_gait_params['joint_vel']*np.ones(6, dtype="int") # create array of joint velocities (all equal) per il caso equal

            #Handle combinations of underactuated | vl_no | rotation
            if self.dynamic_gait_params['rotation'] != 0: # ROTATIONS - ROTATIONS - ROTATIONS
                delta_s = 10 # feet horizontal displacement wrt coxa axis
                try:
                    [q2, q3] = self.robot.leg_inv_kine_coxa_plane([delta_s, self.dynamic_gait_params['td_height']])*180/np.pi
                except:
                    response.success = False
                    response.status_message = 'Failed to set dynamic rotation. Not admissible td_height'
                    return response
                self.q_femur_pull = q2*np.ones(6)
                self.q_tibia_pull = q3*np.ones(6)
                self.q_femur_push = self.q_femur_pull
                self.q_tibia_push = np.zeros(6)
                if self.dynamic_gait_params['rotation'] == -1: #clockwise
                    self.q_coxa_pull = np.array([90,45,0,0,45,90])
                else:#counter clockwise
                    self.q_coxa_pull = np.array([0,-45,-90,-90,-45,0])
            else: # FORWARD - FORWARD - FORWARD
                self.q_coxa_pull = np.array([0,0,0,0,0,0])
                if self.dynamic_gait_params['underactuated'] !=1: # ACTIVE FEMUR IN PUSHING
                    if self.dynamic_gait_params['feet_policy']=='equal':
                        dx = self.dynamic_gait_params['td_height']/np.tan(self.dynamic_gait_params['alpha'])# feet horizontal displacement wrt coxa axis
                        dl = 10 #leg linear elongation in direction alpha
                        L_ext = np.sqrt(dx**2+self.dynamic_gait_params['td_height']**2)+dl
                        if L_ext > self.robot.l2 + self.robot.l3:
                            L_ext = self.robot.l2 + self.robot.l3
                        try:
                            [q2_r, q3_r] = self.robot.leg_inv_kine_coxa_plane([-dx, self.dynamic_gait_params['td_height']])*180/np.pi #right side legs (1,2,3), retracted position
                            [q2_l, q3_l] = self.robot.leg_inv_kine_coxa_plane([dx, self.dynamic_gait_params['td_height']])*180/np.pi #left side legs (4,5,6), retracted position
                            [q2_r_ext, q3_r_ext] = self.robot.leg_inv_kine_coxa_plane([L_ext*np.cos(np.pi-self.dynamic_gait_params['alpha']), L_ext*np.sin(np.pi-self.dynamic_gait_params['alpha'])])*180/np.pi #right side legs (1,2,3), extended position
                            [q2_l_ext, q3_l_ext] = self.robot.leg_inv_kine_coxa_plane([L_ext*np.cos(self.dynamic_gait_params['alpha']), L_ext*np.sin(self.dynamic_gait_params['alpha'])])*180/np.pi #left side legs (1,2,3), extended position
                        except:
                            response.success = False
                            response.status_message = 'Failed to set dynamic gait. Not admissible td_height-alpha combination'
                            return response
                        self.q_femur_pull = np.array([q2_r, q2_r, q2_r, q2_l, q2_l, q2_l])
                        self.q_tibia_pull = np.array([q3_r, q3_r, q3_r, q3_l, q3_l, q3_l])
                        self.q_femur_push = np.array([q2_r_ext, q2_r_ext, q2_r_ext, q2_l_ext, q2_l_ext, q2_l_ext])
                        self.q_tibia_push = np.array([q3_r_ext, q3_r_ext, q3_r_ext, q3_l_ext, q3_l_ext, q3_l_ext])
                    elif self.dynamic_gait_params['feet_policy']=='old': #OLD PUSHING, as in SciRo paper
                        delta_s = 20
                        leg_angles = np.array([1,1,1,-1,-1,-1])*45.0
                        q2 = 50
                        q3 =130
                        self.q_femur_pull = np.array([q2, q2, q2, q2, q2, q2])
                        self.q_tibia_pull = np.array([q3, q3, q3, q3, q3, q3])
                        p_pull = self.robot.kine_coxa_plane(np.array([q2, q3])*np.pi/180)
                        p_push = np.zeros([6,2])
                        for i in range(0,6):
                            p_push[i,:] = p_pull + np.array([delta_s*np.sin(leg_angles[i]),delta_s*np.cos(leg_angles[i])])
                            [self.q_femur_push[i], self.q_tibia_push[i]] = self.robot.leg_inv_kine_coxa_plane(p_push[i,:])*180/np.pi
            response.success = True
            response.status_message = 'Dynamic gait successfully set.'
            return response
        else:
            self.gait_go = False
            self.gait_type = None
            self.ctrl_timestep = 0.03
            response.success = False
            response.status_message = 'Failed to set gait. Invalid gait type selected.'
            return response

    def gait_start(self, req, response):
        if self.gait_type is not None:
            if self.gait_type == "static":
                self.iteration_counter = 1
            elif self.gait_type == "dynamic":
                #inizializzare contatore per palleggio tripodi
                #primo salto? controllare integrità con set_gait
                print("start dynamic gait")
            elif self.gait_type =='crab':
                print("start crab gait")
            else:
                print("bad gait")
        else:
            print("non va bene")

        self.gait_go = True
        return response

    def gait_stop(self, req, response):
        self.gait_go = False
        self.ctrl_timestep = 0.03
        print('gait stopped')
        # Go to idle position
        if self.gait_type == 'static':
            msg = Float64MultiArray()
            tmp = robot.static_poses_pos["dragon"]
            print(tmp)
            tmp = tmp.tolist()
            msg.data = tmp
            self.forward_pos_publisher.publish(msg)
        self.gait_type = None
        return response

if __name__ == '__main__':
    rclpy.init()

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    controller = LocomotionController()

    controller.loop()
    #rclpy.spin(controller)
    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None)
    rclpy.shutdown()
