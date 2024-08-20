#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, Wrench
from cartesian_impedance_controller.msg import ControllerConfig
# from robot_sim.euler_quaternion import get_quaternion_from_euler

class Configuration():

    def __init__(self):

        self.stiff_pub = rospy.Publisher('/dsr01/CartesianImpedance_trajectory_controller/set_cartesian_stiffness', WrenchStamped, queue_size=10)
        self.damp_pub = rospy.Publisher('/dsr01/CartesianImpedance_trajectory_controller/set_damping_factors', WrenchStamped, queue_size=10)
        self.pose_pub = rospy.Publisher('/dsr01/CartesianImpedance_trajectory_controller/reference_pose', PoseStamped, queue_size=10)
        self.config_pub = rospy.Publisher('/dsr01/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
        self.wrench_pub = rospy.Publisher('dsr01/CartesianImpedance_trajectory_controller/set_cartesian_wrench', WrenchStamped, queue_size=10)

    def set_stiffness(self, x = 400.0, y = 300.0, z = 300.0, xt = 30.0, yt = 30.0, zt= 30.0 ):
        
        self.stiffness = WrenchStamped()

        self.stiffness.wrench.force.x = x
        self.stiffness.wrench.force.y = y
        self.stiffness.wrench.force.z = z
        self.stiffness.wrench.torque.x = xt
        self.stiffness.wrench.torque.y = yt
        self.stiffness.wrench.torque.z = zt

        for self.i in range(2):
            self.stiff_pub.publish(self.stiffness)
            rospy.sleep(0.5)

    def set_damping(self, x = 1.0, y = 1.0, z = 1.0, xt = 1.0, yt = 1.0, zt = 1.0):
        
        self.damping = WrenchStamped()

        self.damping.wrench.force.x = x
        self.damping.wrench.force.y = y
        self.damping.wrench.force.z = z
        self.damping.wrench.torque.x = xt
        self.damping.wrench.torque.y = yt
        self.damping.wrench.torque.z = zt

        for self.i in range(2):
            self.damp_pub.publish(self.damping)
            rospy.sleep(0.5)


    def set_config(self, xs, ys, zs, xst, yst, zst, xd, yd, zd, xdt, ydt, zdt, nullspace_stiff, nullspace_damp, f_x, f_y, f_z, tau_x, tau_y, tau_z):
        
        self.config_object = ControllerConfig()
        self.stiffness = Wrench()
        self.damping_factors = Wrench()
        self.wrench_object = WrenchStamped()

        self.stiffness.force.x = xs
        self.stiffness.force.y = ys
        self.stiffness.force.z = zs
        self.stiffness.torque.x = xst
        self.stiffness.torque.y = yst
        self.stiffness.torque.z = zst

        self.damping_factors.force.x = xd
        self.damping_factors.force.y = yd
        self.damping_factors.force.z = zd
        self.damping_factors.torque.x = xdt
        self.damping_factors.torque.y = ydt
        self.damping_factors.torque.z = zdt

        self.config_object.cartesian_stiffness = self.stiffness
        self.config_object.cartesian_damping_factors = self.damping_factors
        self.config_object.nullspace_stiffness = nullspace_stiff
        self.config_object.nullspace_damping_factor = nullspace_damp
        self.config_object.q_d_nullspace = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.wrench_object.wrench.force.x = f_x
        self.wrench_object.wrench.force.y = f_y
        self.wrench_object.wrench.force.z = f_z

        self.wrench_object.wrench.torque.x = tau_x
        self.wrench_object.wrench.torque.y = tau_y
        self.wrench_object.wrench.torque.z = tau_z

        #needed as a work around for unexplained msg type mismatch 
        for self.i in range(3):
            self.config_pub.publish(self.config_object)
            self.wrench_pub.publish(self.wrench_object)
            rospy.sleep(0.5)

    # def set_pose(self, x, y, z, roll, pitch, yaw):

    #     self.pose = PoseStamped()

    #     self.qx, self.qy, self.qz, self.qw = get_quaternion_from_euler(roll, pitch, yaw)

    #     self.pose.pose.position.x = x
    #     self.pose.pose.position.y = y
    #     self.pose.pose.position.z = z
    #     self.pose.pose.orientation.x = self.qx
    #     self.pose.pose.orientation.y = self.qy
    #     self.pose.pose.orientation.z = self.qz
    #     self.pose.pose.orientation.w = self.qw

    #     self.pose_pub.publish(self.pose)

if __name__ == "__main__":
    
    rospy.init_node('config_impedance_controller')
    config_controller = Configuration()

    #Parameters to configure controller
    xs                             =rospy.get_param("/xs")
    ys                             =rospy.get_param("/ys")
    zs                             =rospy.get_param("/zs")
    xst                            =rospy.get_param("/xst")
    yst                            =rospy.get_param("/yst")
    zst                            =rospy.get_param("/zst")
    xd                             =rospy.get_param("/xd")
    yd                             =rospy.get_param("/yd")
    zd                             =rospy.get_param("/zd")
    xdt                            =rospy.get_param("/xdt")
    ydt                            =rospy.get_param("/ydt")
    zdt                            =rospy.get_param("/zdt")
    nullspace_stiff                =rospy.get_param("/nullspace_stiff")
    nullspace_damp                 =rospy.get_param("/nullspace_damp")
    wrench_fx                      =rospy.get_param("/wrench_fx")
    wrench_fy                      =rospy.get_param("/wrench_fy")
    wrench_fz                      =rospy.get_param("/wrench_fz")
    wrench_tx                      =rospy.get_param("/wrench_tx")
    wrench_ty                      =rospy.get_param("/wrench_ty")
    wrench_tz                      =rospy.get_param("/wrench_tz")


    config_controller.set_config(xs, ys, zs, xst, yst, zst, xd, yd, zd, xdt, ydt, zdt, nullspace_stiff, nullspace_damp, wrench_fx, wrench_fy, wrench_fz, wrench_tx, wrench_ty, wrench_tz)
