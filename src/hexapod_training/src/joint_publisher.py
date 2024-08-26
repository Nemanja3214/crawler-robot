#!/usr/bin/python3.8

import rospy
import math
import time
import copy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

   

def make_name(part, side, num):
    return "/hexapod/" + part + "_joint_" + side + str(num) +"_position_controller/command"


class JointPub(object):
    def __init__(self):

        self.publishers_array = []

        parts = [ "coxa", "tibia", "femur"]
        sides = ["l", "r"]
        nums = [1, 2, 3]
        rospy.logdebug("PUBLISHERSSSS:\n\n")
        for part in parts:
            for side in sides:
                for num in nums:
                    name = make_name(part, side, num)
                    rospy.logdebug(name)
                    pub = rospy.Publisher(name, Float64, queue_size=1)
                    self.publishers_array.append(pub)


    def set_init_pose(self, init_pose):
        """
        Sets joints to initial position [0,0,0]
        :return: The init Pose
        """
        rospy.logdebug(init_pose)
        self.check_publishers_connection()
        self.move_joints(init_pose)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(50)  # 10hz
        # rospy.loginfo(self.publishers_array)
        for pub in self.publishers_array:
            while (pub.get_num_connections() == 0):
                rospy.logdebug("No susbribers to JOINT" + pub.name +" yet so we wait and try again")
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    # This is to avoid error when world is rested, time when backwards.
                    pass
            rospy.logdebug("Publisher "+ pub.name +" Connected")

        rospy.logdebug("All Joint Publishers READY")

    def joint_mono_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))

        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):
        rospy.logdebug("Moving joints>>>>" +str(joints_array))

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.logdebug("JointsPos>>"+str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


if __name__=="__main__":
    rospy.init_node('joint_publisher_node', log_level=rospy.WARN)
    joint_publisher = JointPub()
    rate_value = 8.0
    #joint_publisher.start_loop(rate_value)
    #joint_publisher.start_sinus_loop(rate_value)
