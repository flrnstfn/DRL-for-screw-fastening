#!/usr/bin/env python3

import os
import signal
import subprocess
import roslaunch
import rospy
import time
from std_msgs.msg import Bool

class GazeboSimulation():

    def __init__(self):

        self.sim_failure = False

        rospy.init_node('gazebo_master_node')
        self.sim_failure_pub = rospy.Publisher('/gazebo_sim_failure', Bool, queue_size=10)
        self.sim_failure_sub = rospy.Subscriber('/gazebo_sim_failure', Bool, self.state_callback)    

    def state_callback(self, msg):
        self.sim_failure = msg.data

    def get_sim_failure(self):
        return self.sim_failure
    
    def set_sim_failure(self, value):
        self.sim_failure = value
        
    def source_ros(self):
        self.command = "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash"
        os.system(self.command)

    def close_processes(self, process_names):
        # Function to terminate processes by name
        def terminate_process(process_name):
            try:
                self.pids = subprocess.check_output(['pgrep', '-f', process_name]).decode().strip().split('\n')
                print(self.pids)
                for pid in self.pids:
                    os.kill(int(pid), signal.SIGTERM)
                    rospy.loginfo(f"{process_name} process {pid} terminated.")
            except subprocess.CalledProcessError:
                rospy.loginfo(f"{process_name} is not running.")
            except Exception as e:
                rospy.logerr(f"Failed to terminate {process_name} process: {e}")

        for process_name in process_names:
            terminate_process(process_name)

    def start_robot_launch(self):

        # Specify the package and launch file name
        self.package = 'robot_sim'
        self.launch_file = 'm1013_gazebo_rviz.launch'
        self.curr_file_dir = os.path.dirname(os.path.abspath(__file__))
        self.launch_file_path = os.path.join(self.curr_file_dir, '..', '..', self.package, 'launch', self.launch_file)

        # Create a roslaunch parent object
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.launch_file_path])

        # Start the launch file
        self.launch.start()
        rospy.loginfo("Gazebo simulation restarted.")
        self.set_sim_failure(value=False)


    def restart_gazebo_simulation(self):
        self.process_names = [
            'controller_manager',
            'gzserver',
            'gzclient',
            'joint_state_publisher',
            'joint_state_publisher_gui',
            'robot_state_publisher',
            'rviz',
            'controller_manager/spawner',
            'gazebo_ros/spawn_model',
            'tf/static_transform_publisher'
        ]
        self.close_processes(self.process_names)
        #Wait a few seconds to ensure the processes are fully terminated
        time.sleep(5)
        self.start_robot_launch()
        

if __name__ == '__main__':

    simulation = GazeboSimulation()
    simulation.start_robot_launch()

    while not rospy.is_shutdown():
        
        time.sleep(0.1)
        if simulation.get_sim_failure() == True:
            simulation.restart_gazebo_simulation()
            time.sleep(10)