#!/usr/bin/env python3

import rospy
import os
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, SetModelStateRequest, DeleteModel, DeleteModelRequest
import numpy as np
import matplotlib.pyplot as plt

class Model():

    def __init__(self):
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def spawn_single_model(self, model_name, pos_x, pos_y, pos_z, roll, pitch, yaw, ref_frame="world"):
        
        rospy.wait_for_service('/gazebo/spawn_sdf_model')

        #paths to sdf files
        self.current_file_dir = os.path.dirname(os.path.abspath(__file__))
        #print(self.current_file_dir)

        self.rel_path_lamp = os.path.join(self.current_file_dir, '..', '..', 'models', 'lamp_model', 'model.sdf')
        self.rel_path_screw = os.path.join(self.current_file_dir, '..', '..', 'models', 'screw_model', 'model.sdf') 

        #self.path_check = os.path.abspath(self.rel_path_lamp)
        #print(self.path_check)
        
        #Chech which model to load
        if model_name == "lamp" or model_name == "lamp1":
            self.model_path = self.rel_path_lamp
        elif model_name =="screw":
            self.model_path = self.rel_path_screw
        else:
            rospy.logerr("incorrect model name")

        #Calculate quarternion from euler
        self.qx, self.qy, self.qz, self.qw = self.get_quaternion_from_euler(roll, pitch, yaw)

        #Call spawn service
        try:
    
            self.spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            self.request_object = SpawnModelRequest()

            #Read XML file to string for service request
            with open(self.model_path, 'r') as file:
                self.sdf=file.read()
        
            #Set XML
            self.request_object.model_xml = self.sdf
        
            #Set name and namespace / equals global namespace
            self.request_object.model_name = model_name
            self.request_object.robot_namespace = "/"

            #Set pose
            self.request_object.initial_pose.position.x = pos_x
            self.request_object.initial_pose.position.y = pos_y
            self.request_object.initial_pose.position.z = pos_z
            self.request_object.initial_pose.orientation.x = self.qx
            self.request_object.initial_pose.orientation.y = self.qy
            self.request_object.initial_pose.orientation.z = self.qz
            self.request_object.initial_pose.orientation.w = self.qw

            #Set reference frame
            self.request_object.reference_frame = ref_frame

            #Call the service
            self.resp = self.spawn_model_srv(self.request_object)

            # Check the result
            if self.resp.success:
                rospy.loginfo("Model spawned successfully: %s", self.resp.status_message)
            else:
                rospy.logerr("Failed to spawn model: %s", self.resp.status_message)
    
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):

        # Convert an Euler angle to a quaternion.
   
        # Input
        # :param roll: The roll (rotation around x-axis) angle in radians.
        # :param pitch: The pitch (rotation around y-axis) angle in radians.
        # :param yaw: The yaw (rotation around z-axis) angle in radians.
 
        # Output
        # :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
 
        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [self.qx, self.qy, self.qz, self.qw]
    
    def spawn_assembly_group_random(self, x0_search, y0_search, spawn_lower_lim, spawn_upper_lim, model_1, model_2, model_3, roll, pitch, yaw, std_dev_x, std_dev_y, ref_frame="world"):
        
        #Generate x and y position offset for spawning the assembly group 
        self.dx, self.dy = self.generate_2D_position(spawn_lower_lim, spawn_upper_lim, std_dev_x, std_dev_y)
        #rospy.loginfo('Generated position offset for models:'+ self.dx, self.dy)
        #Calculating absolut position for middlepoint of assembly group in x-y plane
        self.pos_x = x0_search + self.dx
        self.pos_y = y0_search + self.dy
        #print(self.pos_x, self.pos_y)
        #Spawning the models of the assembly group
        self.spawn_single_model(model_1, self.pos_x, self.pos_y, 0, roll, pitch, yaw, ref_frame)
        self.spawn_single_model(model_2, self.pos_x, self.pos_y, 0.017, roll, pitch, yaw, ref_frame)
        self.spawn_single_model(model_3, self.pos_x, self.pos_y, 0.043, roll, pitch, yaw, ref_frame)

        return self.pos_x, self.pos_y

    def spawn_assembly_group_fixed(self, pos_x, pos_y, model_1, model_2, model_3, roll, pitch, yaw, ref_frame="world"):
        
        #Spawning the models of the assembly group
        self.spawn_single_model(model_1, pos_x, pos_y, 0, roll, pitch, yaw, ref_frame)
        self.spawn_single_model(model_2, pos_x, pos_y, 0.017, roll, pitch, yaw, ref_frame)
        self.spawn_single_model(model_3, pos_x, pos_y, 0.043, roll, pitch, yaw, ref_frame)
    
    def init_seed(self):
        #Seeding the random number generator
        self.seed = rospy.get_param("/random_seed", None)
        rospy.logdebug("Seed initialized")
        if self.seed is not None:
            np.random.seed(self.seed)

    def generate_2D_position(self, lower_lim, upper_lim, std_dev_x, std_dev_y):

        # Define the number of points to generate
        self.num_points = 1

        # Define the mean and standard deviation for x and y
        self.mean_x, self.mean_y = 0, 0  # Center of the range [-0.02,0.02]

        # Generate x and y values based on a normal distribution
        self.x_value = np.random.normal(loc=self.mean_x, scale=std_dev_x, size=self.num_points)
        self.y_value = np.random.normal(loc=self.mean_y, scale=std_dev_y, size=self.num_points)
        #print(self.x_value, self.y_value)
        # Ensure the values are within the range [lower_lim, upper_lim]
        self.x_value = np.clip(self.x_value, lower_lim, upper_lim)
        self.y_value = np.clip(self.y_value, lower_lim, upper_lim)
        #print(self.x_value, self.y_value)
        #print(lower_lim, upper_lim)
        # #Plot the generated coordinates
        # plt.scatter(x_value, y_value, alpha=0.6)
        # plt.xlim(0, 100)
        # plt.ylim(0, 100)
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.title('Coordinates generated with normal distribution')
        # #plt.show()

        return self.x_value[0], self.y_value[0]

    def move_model(self, pos_x, pos_y, pos_z, roll, pitch, yaw, model_name, ref_frame = 'world'):
        
        rospy.wait_for_service('/gazebo/set_model_state')

        try: 

            self.move_model_srv=rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.move_request=SetModelStateRequest()

            #Set model to move
            self.move_request.model_state.model_name = model_name

            #Calc quaternions
            self.qx, self.qy, self.qz, self.qw = self.get_quaternion_from_euler(roll, pitch, yaw)

            #Set pose 
            self.move_request.model_state.pose.position.x = float(pos_x)
            self.move_request.model_state.pose.position.y = float(pos_y)
            self.move_request.model_state.pose.position.z = float(pos_z)
            self.move_request.model_state.pose.orientation.x = float(self.qx)
            self.move_request.model_state.pose.orientation.y = float(self.qy)
            self.move_request.model_state.pose.orientation.z = float(self.qz)
            self.move_request.model_state.pose.orientation.z = float(self.qw)

            #Set reference frame 
            self.move_request.model_state.reference_frame = ref_frame

            #Publish request to service server
            self.resp = self.move_model_srv(self.move_request)

            if self.resp.success:
                rospy.loginfo("Model moved successfully: %s", self.resp.status_message)
            else:
                rospy.logerr("Failed to move model: %s", self.resp.status_message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def delete_single_model(self, model_name):

        rospy.wait_for_service('/gazebo/delete_model')

        try: 

            self.delete_model_srv=rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            self.delete_request=DeleteModelRequest()

            #Set model to delete
            self.delete_request.model_name = model_name

            #Publish request to service server
            self.resp = self.delete_model_srv(self.delete_request)

            if self.resp.success:
                rospy.loginfo("Model deleted successfully: %s", self.resp.status_message)
            else:
                rospy.logerr("Failed to delete model: %s", self.resp.status_message)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def delete_multiple_models(self, model_names):

        for string in model_names:
            self.delete_single_model(model_name = string)
            rospy.loginfo('Deleted: '+string+' successful')

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True
    
if __name__ == '__main__':
    rospy.init_node('sim_setup_node')
    sim_setup = GazeboModel()
    # for i in range(10):
    #     x, y = sim_setup.generate_2D_position(-0.02, 0.02)
    #     print(x, y)
    # sim_setup.init_seed()
    # for i in range(10):
    #     sim_setup.spawn_assembly_group_random(0.8, 0.0, -0.02, 0.02, 'd', 'v', 'ds')

    #sim_setup.delete_multiple_models(['lamp', 'lamp1', 'screw'])
    #sim_setup.spawn_assembly_group(0, 0, 0 , 0, 0, 0)
    sim_setup.spawn_single_model('lamp', 0.8, 0, 0, 0, 0, 0)
    sim_setup.spawn_single_model('lamp1',0.8, 0, 0.017, 0, 0, 0)
    sim_setup.spawn_single_model('screw', 0.8, 0, 0.043, 0, 0, 0)
    #rospy.sleep(3.)
    # sim_setup.move_model(1,1, 0.04, 0, 0 , 0,'screw')
    # sim_setup.move_model(1,1, 0, 0, 0 , 0,'lamp')
    # sim_setup.move_model(1,1, 0.017, 0, 0 , 0,'lamp1')
    # rospy.sleep(3.)
    # sim_setup.delete_model('lamp')
    # rospy.sleep(3.)
    # sim_setup.spawn_single_model('lamp', 0.3, 0.3, 0, 0, 0, 0)