#!/usr/bin/env python3
import rospy


if __name__ == "__main__":
    rospy.init_node("test_node")

    rospy.loginfo("Hello from test node")

    rospy.sleep(5)
    rospy.loginfo("SHutting down")