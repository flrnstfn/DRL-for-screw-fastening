#! /usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import WrenchStamped, Vector3, PoseStamped, Pose, Wrench
from sensor_msgs.msg import JointState
from cartesian_impedance_controller.msg import ControllerState
from std_msgs.msg import Bool

class Robot():

    def __init__(self, ns="dsr01"):

        self.tool_force = Vector3()
        self.ref_pose = Pose()
        self.current_pose = Pose()
        self.cartesian_stiffness = Wrench()
        self.cartesian_damping = Wrench()
        self.joint_state = JointState()
        self.nullspace_stiffness = np.float64
        self.nullspace_damping = np.float64
        self.sim_failure = False
        self.successful_episodes = 0
    
        self.force_topic_name = '/'+ns+'/ft_sensor_topic'
        self.ref_pose_topic_name = '/'+ns+'/CartesianImpedance_trajectory_controller/reference_pose'
        self.controller_state_topic_name = '/'+ns+'/CartesianImpedance_trajectory_controller/controller_state'
        self.sim_failure_topic_name = '/gazebo_sim_failure'

        #Subscriber
        self.force_subscriber = rospy.Subscriber(self.force_topic_name, WrenchStamped, self.ft_sensor_cb)
        self.controller_state_sub = rospy.Subscriber(self.controller_state_topic_name, ControllerState, self.controller_state_cb)
        
        #Publisher
        self.ref_pose_pub = rospy.Publisher(self.ref_pose_topic_name, PoseStamped, queue_size=10)
        self.sim_failure_pub = rospy.Publisher(self.sim_failure_topic_name, Bool, queue_size=10)

    def get_observation(self, x0_search, y0_search, z0_search):

        #Calculate position in search coordinate system in mm
        self.pos_x_search = 100 * (self.current_pose.position.x - x0_search)
        self.pos_y_search = 100 * (self.current_pose.position.y - y0_search)
        self.pos_z_search = 100 * (-self.current_pose.position.z + z0_search)

        self.observation_temp = [self.tool_force.x, self.tool_force.y, self.tool_force.z, self.pos_x_search, self.pos_y_search, self.pos_z_search]

        #Check for any NaN values and replace them with 0
        self.observation = self.replace_nan_values(self.observation_temp)

        return self.observation

    def replace_nan_values(self, array):
        return [0 if math.isnan(x) else x for x in array]

    def ft_sensor_cb(self, msg):
        self.tool_force = msg.wrench.force

    def pub_initial_pose(self):
        self.initial_pose = PoseStamped()

        self.initial_pose.pose.position.x = 0
        self.initial_pose.pose.position.y = 0
        self.initial_pose.pose.position.z = 1.55
        
        self.initial_pose.pose.orientation.x = 0
        self.initial_pose.pose.orientation.y = 0.3
        self.initial_pose.pose.orientation.z = 0
        self.initial_pose.pose.orientation.w = 0.93

        self.ref_pose_pub.publish()

    def incremental_movement(self, dx, dy, dz):

        self.new_ref_pose = PoseStamped()
        self.new_ref_pose.pose.position.x    = self.ref_pose.position.x + dx
        self.new_ref_pose.pose.position.y    = self.ref_pose.position.y + dy
        self.new_ref_pose.pose.position.z    = self.ref_pose.position.z + dz
        
        # has to be made variable to extend to rotation
        self.new_ref_pose.pose.orientation.x = 0
        self.new_ref_pose.pose.orientation.y = 1
        self.new_ref_pose.pose.orientation.z = 0
        self.new_ref_pose.pose.orientation.w = 0

        self.ref_pose_pub.publish(self.new_ref_pose)

    def move_init_pose(self, x0_search, y0_search, z0_search):

        self.init_pose = PoseStamped()
        self.init_pose.pose.position.x = x0_search
        self.init_pose.pose.position.y = y0_search
        self.init_pose.pose.position.z = z0_search

        self.init_pose.pose.orientation.x = 0
        self.init_pose.pose.orientation.y = 1
        self.init_pose.pose.orientation.z = 0
        self.init_pose.pose.orientation.w = 0

        self.ref_pose_pub.publish(self.init_pose)

    def perform_action(self, action, action_scale, x0_search, y0_search, z0_search, scope_boundary):
        
        #rescaling of action to action space
        self.scld_action = [action_scale * action[0], action_scale * action[1], action_scale * action[2]]

        self.new_ref_pose = PoseStamped()
        self.new_ref_pose.pose.position.x    = self.ref_pose.position.x + self.scld_action[0]
        self.new_ref_pose.pose.position.y    = self.ref_pose.position.y + self.scld_action[1]
        self.new_ref_pose.pose.position.z    = self.ref_pose.position.z + self.scld_action[2]
        
        # has to be made variable to extend to rotation
        self.new_ref_pose.pose.orientation.x = 0
        self.new_ref_pose.pose.orientation.y = 1
        self.new_ref_pose.pose.orientation.z = 0
        self.new_ref_pose.pose.orientation.w = 0

        # Calculating the distance between the starting point and the new ref pose
        self.distance_x0 = abs(self.new_ref_pose.pose.position.x - x0_search) 
        self.distance_y0 = abs(self.new_ref_pose.pose.position.y - y0_search)
        self.distance_z0 = abs(self.new_ref_pose.pose.position.z - z0_search)
        rospy.logdebug(f"distance_x0: {self.distance_x0}")
        rospy.logdebug(f"distance_y0: {self.distance_y0}")
        rospy.logdebug(f"distance_z0: {self.distance_z0}")

        #Checking if new ref_pose is within search space
        if self.distance_x0 <= scope_boundary:
            self.check_x = True
        else:
            self.check_x = False

        if self.distance_y0 <= scope_boundary:
            self.check_y = True
        else:
            self.check_y = False

        if self.distance_z0 <= scope_boundary:
            self.check_z = True
        else:
            self.check_z = False

        if self.check_x and self.check_y and self.check_z:
            rospy.logdebug(f'New pose within scope')
            self.ref_pose_pub.publish(self.new_ref_pose)
            truncated = False
            return truncated
        else: 
            truncated = True
            rospy.logdebug(f"Pose exceeds scope limits")
            return truncated

    def get_ref_pose(self):
        return self.ref_pose
    
    def get_current_pose(self):
        return self.current_pose
    
    def get_cart_stiffness(self):
        return self.cartesian_stiffness
    
    def get_cart_damping(self):
        return self.cartesian_damping
    
    def get_nullspace_stiffness(self):
        return self.nullspace_stiffness
        
    def get_nullspace_damping(self):
        return self.nullspace_damping
 
    def controller_state_cb(self, msg):
        self.ref_pose = msg.reference_pose
        self.current_pose = msg.current_pose
        self.cartesian_stiffness = msg.cartesian_stiffness
        self.cartesian_damping = msg.cartesian_damping
        self.nullspace_stiffness = msg.nullspace_stiffness
        self.nullspace_damping = msg.nullspace_damping
        self.joint_state = msg.joint_state

    def get_sim_failure(self):
        return self.sim_failure
    
    def pub_sim_failure(self):
        self.sim_failure_pub.publish(self.sim_failure)

    def evaluate_state(self, screw_pose, insert_target, force_threshold, terminal_x_offset, terminal_y_offset, terminal_z_offset, lambda1, lambda2, lambda3, lambda4, lambda5, r5):
        
        self.screw_position = [screw_pose.position.x, screw_pose.position.y, screw_pose.position.z]
        rospy.logdebug(f"screw_position: {self.screw_position}")
        
        self.current_sd_position = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z]
        rospy.logdebug(f"current_sd_position: {self.current_sd_position}")

        # Set target position
        self.target_position = [self.screw_position[0], self.screw_position[1], self.screw_position[2] - insert_target]
        rospy.logdebug(f"target_position: {self.target_position}")

        # Calculate distances in x, y, z direction between current position of screwdriver (sd) and target position
        self.distance_x = self.current_sd_position[0] - self.target_position[0]
        rospy.logdebug(f"distance_x: {self.distance_x}")
        
        self.distance_y = self.current_sd_position[1] - self.target_position[1]
        rospy.logdebug(f"distance_y: {self.distance_y}")
        
        self.distance_z = self.current_sd_position[2] - self.target_position[2]
        rospy.logdebug(f"distance_z: {self.distance_z}")

        # Calculate Euclidean distance between current position and target position
        self.r1 = math.sqrt(math.pow(self.distance_x, 2) + math.pow(self.distance_y, 2) + math.pow(self.distance_z, 2))
        rospy.logdebug(f"r1 (Euclidean distance): {self.r1}")

        # Calculate absolute position error in x-y plane
        self.r2 = abs(self.distance_x) + abs(self.distance_y)
        rospy.logdebug(f"r2 (absolute position error x-y): {self.r2}")

        # Calculate z direction reward
        if self.distance_z > 0:
            self.r3 = self.distance_z
        else: 
            self.r3 = 5 * abs(self.distance_z)
        
        rospy.logdebug(f"r3 (absolute position error z): {self.r3}")

        # Calculate force based reward with check for NaN values
        self.contact_force = math.sqrt(math.pow(self.tool_force.x, 2) + math.pow(self.tool_force.y, 2) + math.pow(self.tool_force.z, 2))
        if math.isnan(self.contact_force):
            self.contact_force = 0
        rospy.logdebug(f"contact_force: {self.contact_force}")
        if self.contact_force <= force_threshold:
            self.r4 = 0
        else:
            self.r4 = self.contact_force
        rospy.logdebug(f"r4 (force based reward): {self.r4}")

        # Calculate reward without r5 for reaching target
        self.r_temp = -lambda1 * self.r1 - lambda2 * self.r2 - lambda3 * self.r3 - lambda4 * self.r4
        rospy.logdebug(f"r_temp: {self.r_temp}")

        # Check if current state is terminal state
        if abs(self.distance_x) <= terminal_x_offset and abs(self.distance_y) <= terminal_y_offset and self.distance_z <= terminal_z_offset:
            reward = self.r_temp + lambda5 * r5
            terminated = True
            self.successful_episodes += 1
            rospy.logdebug(f"terminal_state_reached")
        else:
            reward = self.r_temp
            terminated = False
        rospy.logdebug(f"reward: {reward}, terminated: {terminated}")

        # Check if simulation has failed via NaN value in joint_state velocity
        self.vel_joint_1 = self.joint_state.velocity[0]
        self.nan_value = math.isnan(self.vel_joint_1)
        if self.nan_value == True:
            truncated_simulation_failure = True
            self.sim_failure = True
            rospy.logdebug(f"Simulation_failure_detected")
        else:
            truncated_simulation_failure = False
            self.sim_failure = False

        return reward, terminated, truncated_simulation_failure, self.successful_episodes, self.contact_force


if __name__ =="__main__":

    rospy.init_node('sub_node_force')
    robot = Robot()
    rospy.sleep(1)
    # screw = Pose()
    # screw.position.x= 0.81
    # screw.position.y= 0.015
    # screw.position.z= 0.03
    # reward, terminated = robot.evaluate_state(screw, insert_target=0.01, force_threshold=10, terminal_radius=0.1, lambda1=1000, lambda2=1, lambda3=1, lambda4=1, r5=10)
    robot.incremental_movement(0, 0, 0.02)
 
    