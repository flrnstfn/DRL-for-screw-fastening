#! /usr/bin/env python3

import gymnasium as gym
import rospy
import os
import numpy as np
import sys
import time

curr_file_dir = os.path.dirname(os.path.abspath(__file__))

rel_path_robot_control = os.path.join(curr_file_dir, '..', '..', 'robot_control', 'src')
rel_path_robot_sim = os.path.join(curr_file_dir, '..', '..', 'robot_sim', 'src')

sys.path.append(rel_path_robot_sim)
sys.path.append(rel_path_robot_control)

from gymnasium import utils, spaces
from gymnasium.utils import seeding
from gymnasium.envs.registration import register
from gazebo_connection import Gazebo
from controller_manager_connection import ControllerManager
from robot_sim.simulation_model_setup import Model
from robot_connection import Robot


register(
    id="ScrewFastening-v1",
    entry_point="training_environment:ScrewFastEnv",
    max_episode_steps=200
)

class ScrewFastEnv(gym.Env):

    def __init__(self):

        # Assume that a ROS node has already been created before initialising the environment
        #Place to extract all parameters from ros parameter server

        #Parameters for evaluate_state function
        self.insert_depth_target            =rospy.get_param("/insert_depth_target")     
        self.force_threshold                =rospy.get_param("/force_threshold")        
        self.terminal_x_offset              =rospy.get_param("/terminal_x_offset")
        self.terminal_y_offset              =rospy.get_param("/terminal_y_offset")
        self.terminal_z_offset              =rospy.get_param("/terminal_z_offset")   
        self.lambda1                        =rospy.get_param("/lambda1")     
        self.lambda2                        =rospy.get_param("/lambda2")     
        self.lambda3                        =rospy.get_param("/lambda3")     
        self.lambda4                        =rospy.get_param("/lambda4")
        self.lambda5                        =rospy.get_param("/lambda5")
        self.success_reward                 =rospy.get_param("/success_reward")   

        #Parameters for action and observation spaces
        self.action_low_bound               =rospy.get_param("/action_low_bound") 
        self.action_high_bound              =rospy.get_param("/action_high_bound")     
        self.observation_low_bound          =rospy.get_param("/observation_low_bound")
        self.observation_high_bound         =rospy.get_param("/observation_high_bound")

        #Parameters to delete models
        self.model_1                        =rospy.get_param("/model_1")
        self.model_2                        =rospy.get_param("/model_2")
        self.model_3                        =rospy.get_param("/model_3")
        
        #Coordinates for origin of search frame 
        self.x0_search                      =rospy.get_param("/x0_search") 
        self.y0_search                      =rospy.get_param("/y0_search")
        self.z0_search                      =rospy.get_param("/z0_search")

        #Parameters to spawn models
        self.spawn_lower_lim                =rospy.get_param("/spawn_lower_lim")       
        self.spawn_upper_lim                =rospy.get_param("/spawn_upper_lim")       
        self.model_roll                     =rospy.get_param("/model_roll")            
        self.model_pitch                    =rospy.get_param("/model_pitch")          
        self.model_yaw                      =rospy.get_param("/model_yaw")     
        self.spawn_std_dev_x                =rospy.get_param("/spawn_std_dev_x")
        self.spawn_std_dev_y                =rospy.get_param("/spawn_std_dev_y")

        #parameters to perform action
        self.action_scale                   =rospy.get_param("/action_scale")
        self.scope_boundary                 =rospy.get_param("/scope_boundary")
        self.delta_t_obs                    =rospy.get_param("/delta_t_obs")

        #stablishing connection with simulation via object of class GazeboConnection that provides pause/unpause etc functions
        self.gazebo = Gazebo()
        #stablishing connection to ros_control controller manager
        self.controller_manager = ControllerManager()
        #stablishing class for model interaction in gazebo
        self.model = Model()
        #stablishing connection to robot object
        self.robot = Robot()

        #Call seed function to seed the environment
        self.seed()
        self.model.init_seed()

        #Init action space and observation space
        self.action_space = spaces.Box(low=np.array(self.action_low_bound), high=np.array(self.action_high_bound), dtype=np.float64)
        self.observation_space = spaces.Box(low=np.array(self.observation_low_bound), high=np.array(self.observation_high_bound), dtype=np.float64)

    # A function to seed the random generator 
    def seed(self, seed=20):
        self.np_random, seed = seeding.np_random(seed)
        np.random.seed(seed)
        rospy.set_param("/random_seed", seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def reset(self, seed = None):
        
        # Check if restarting the gazbeo simulation is neccessary
        if self.robot.get_sim_failure() == True:
            self.robot.pub_sim_failure()
            time.sleep(30)
            
        # 0 lift screw driver
        rospy.sleep(.5)
        self.robot.incremental_movement(0, 0, 0.02)
        rospy.sleep(2)

        # 1st stop the existing controllers
        rospy.logdebug("Stopping controller")
        self.controller_manager.stop_controller(controller_name = 'CartesianImpedance_trajectory_controller')

        # 2nd reset
        rospy.logdebug("Reset Sim")
        self.gazebo.resetSim()

        # 3rd restart controller
        rospy.logdebug("Starting controller")
        self.controller_manager.start_controller(controller_name = 'CartesianImpedance_trajectory_controller')

        # 4th Move pos
        rospy.sleep(0.1)
        self.robot.move_init_pose(0.2, 0, 1.3)
        rospy.sleep(1)
        self.robot.move_init_pose(0.5, 0, 0.8)
        rospy.sleep(1.5)
        self.robot.move_init_pose(0.8, 0, 0.3)
        rospy.sleep(1.5)
        self.robot.move_init_pose(0.8, 0, 0.1)
        rospy.sleep(1.5)
        self.robot.move_init_pose(0.8, 0, 0.052)

        # 5th delete models
        rospy.logdebug("Deleting models")
        self.model.delete_multiple_models([self.model_1, self.model_2, self.model_3])
        rospy.sleep(2.)

        # 6th spawn models within searching area with a guassian distribution
        rospy.logdebug("Spawning models")
        #Spawn with probability distribution
        self.pos_x, self.pos_y = self.model.spawn_assembly_group_random(self.x0_search, self.y0_search, self.spawn_lower_lim, self.spawn_upper_lim, self.model_1, self.model_2, self.model_3, self.model_roll, self.model_pitch, self.model_yaw, self.spawn_std_dev_x, self.spawn_std_dev_y)
        #Spawn in fixed position
        #self.model.spawn_assembly_group_fixed(0.8, 0, self.model_1, self.model_2, self.model_3, self.model_roll, self.model_pitch, self.model_yaw)
        rospy.sleep(2.)

        # 7th get inital observation
        rospy.logdebug("Get initial observation")
        self.observation = self.robot.get_observation(x0_search=self.x0_search, y0_search=self.y0_search, z0_search=self.z0_search)
        obs = np.array(self.observation).astype(np.float64)

        #info
        info = {'pos_x' : self.pos_x, 'pos_y': self.pos_y}
        #info = {}

        return obs, info

    
    def step(self, action):

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot
        
        # 1st execute the action; truncate if action would move robot out of scope
        truncated_scope_space = self.robot.perform_action(action, self.action_scale, self.x0_search, self.y0_search, self.z0_search, self.scope_boundary)

        # 2nd wait for action to finish
        rospy.sleep(self.delta_t_obs)

        # 3rd get new observation
        self.observation = self.robot.get_observation(x0_search=self.x0_search, y0_search=self.y0_search, z0_search=self.z0_search)

         # 4th evaluate current state
        self.screw_pose = self.gazebo.get_screw_pose()
        reward, terminated, truncated_simulation_failure, successful_episodes, contact_force = self.robot.evaluate_state(self.screw_pose, self.insert_depth_target, self.force_threshold,  self.terminal_x_offset, self.terminal_y_offset, self.terminal_z_offset, self.lambda1, self.lambda2, self.lambda3, self.lambda4, self.lambda5, self.success_reward)

        # 5th evaluate truncated
        if truncated_simulation_failure or truncated_scope_space:
            truncated = True
        else:
            truncated = False

        # 6th info dict
        info = {'success': successful_episodes, 'contact_force': contact_force}

        return np.array(self.observation).astype(np.float64), reward, terminated, truncated, info