#! /usr/bin/env python3

import rospy
import os
import subprocess
import signal
import roslaunch
import time
from std_srvs.srv import Empty
from gazebo_msgs.msg import ODEPhysics, ModelStates
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest 
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

class Gazebo():
    
    def __init__(self):
        
        self.model_states = ModelStates()

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb)

    def pauseSim(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")
        
    def unpauseSim(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print ("/gazebo/unpause_physics service call failed")
        
    def resetSim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")

    def resetWorld(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_world()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_world service call failed")
    
    def model_states_cb(self, msg):
        self.model_states = msg

    def get_screw_pose(self):
        self.index = self.model_states.name.index('screw')
        self.screw_pose = self.model_states.pose[self.index]

        return self.screw_pose

    def init_values(self):

        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            # reset_proxy.call()
            self.reset()
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")

        self._time_step = Float64(0.001)
        self._max_update_rate = Float64(1000.0)

        self._ode_config = ODEPhysics()
        self._ode_config.auto_disable_bodies = False
        self._ode_config.sor_pgs_precon_iters = 0
        self._ode_config.sor_pgs_iters = 50
        self._ode_config.sor_pgs_w = 1.3
        self._ode_config.sor_pgs_rms_error_tol = 0.0
        self._ode_config.contact_surface_layer = 0.001
        self._ode_config.contact_max_correcting_vel = 0.0
        self._ode_config.cfm = 0.0
        self._ode_config.erp = 0.2
        self._ode_config.max_contacts = 20


if __name__ =="__main__":
    rospy.init_node('bla')
    gazebo = GazeboConnection()
    # gazebo.resetWorld()
    # gazebo.unpauseSim()
    rospy.sleep(0.01)
    gazebo.resetSim()
    