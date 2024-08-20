#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, UnloadController, UnloadControllerRequest, LoadController, LoadControllerRequest

class ControllerManager():

    def __init__(self, ns="dsr01"):
        self.reset_service_name = '/'+ns+'/controller_manager/switch_controller'
        self.unload_service_name = '/'+ns+'/controller_manager/unload_controller'
        self.load_service_name = '/'+ns+'/controller_manager/load_controller'
        self.reset_service = rospy.ServiceProxy(self.reset_service_name, SwitchController)
        self.unload_service = rospy.ServiceProxy(self.unload_service_name, UnloadController)
        self.load_service = rospy.ServiceProxy(self.load_service_name, LoadController)

    def stop_controller(self, controller_name, strictness=2):
        rospy.wait_for_service(self.reset_service_name)

        try:
            self.reset_request = SwitchControllerRequest()
            self.reset_request.stop_controllers = [controller_name]
            self.reset_request.start_controllers = []
            self.reset_request.strictness = strictness

            stop_result = self.reset_service(self.reset_request)

            rospy.logdebug("Stop Result==>"+str(stop_result.ok))

            return stop_result.ok
        
        except rospy.ServiceException as e:
            print (self.reset_service_name+" service stop call failed")

            return None
        
    def start_controller(self, controller_name, strictness=1):
        rospy.wait_for_service(self.reset_service_name)

        try:
            self.reset_request = SwitchControllerRequest()
            self.reset_request.stop_controllers = []
            self.reset_request.start_controllers = [controller_name]
            self.reset_request.strictness = strictness

            start_result = self.reset_service(self.reset_request)

            rospy.logdebug("Stop Result==>"+str(start_result.ok))

            return start_result.ok
        
        except rospy.ServiceException as e:
            print (self.reset_service_name+" service stop call failed")

            return None
        
    def reset_controller(self, controller_name, strictness=2):
        if self.stop_controller(controller_name, strictness):
            rospy.sleep(1.0)
            self.reset_result = self.start_controller(controller_name, strictness)
            if self.reset_result:
                rospy.loginfo('Reset successful')

    def unload_controller(self, controller_name):
        rospy.wait_for_service(self.unload_service_name)

        try:
            self.unload_request = UnloadControllerRequest()
            self.unload_request.name = controller_name
            unload_result = self.unload_service(self.unload_request)

            rospy.logdebug("Unload Result==>"+str(unload_result.ok))

            return unload_result.ok
         
        except rospy.ServiceException as e:
            print (self.unload_service_name+" service unload call failed")

            return None

    def load_controller(self, controller_name):
        rospy.wait_for_service(self.load_service_name)

        try:
            self.load_request = LoadControllerRequest()
            self.load_request.name = controller_name
            load_result = self.load_service(self.load_request)

            rospy.logdebug("Unload Result==>"+str(load_result.ok))

            return load_result.ok
         
        except rospy.ServiceException as e:
            print (self.load_service_name+" service load call failed")

            return None


