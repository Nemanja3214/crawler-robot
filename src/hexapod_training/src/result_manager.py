#!/usr/bin/python3.8

import json
import rospy
import numpy
import rospkg

from hexapod_training.msg import ResultMsg
import os


def save(msg):
    try:
        dir = rospy.get_param("result_dir")
        with open(dir + "/result"+ str(msg.order)+ ".json", "w+") as file:
            rospy.loginfo("DUMPING")
            ob = {
                "order": msg.order,
                "reward": msg.reward,
                "actions": msg.actions
            }
            rospy.loginfo(ob)
            # rospy.loginfo(os.access(result_path, os.W_OK))
            json.dump(ob, file)
    except Exception as e:
        rospy.logerr(e)


if __name__ == "__main__":
    rospy.loginfo("STARTED LOADER")
    rospy.init_node("result_loader", anonymous=True)
    rospy.loginfo("Choose number from 1 to 10")
    while not rospy.is_shutdown():
        option = input("Choose option: ")
        try:
            dir = rospy.get_param("result_dir")
            with open(dir + "/result"+ option + ".json", "r") as file:
                rospy.loginfo("LOADING")
                
                # rospy.loginfo(os.access(result_path, os.W_OK))
                ob = json.load(file)
                rospy.loginfo(ob)
        except Exception as e:
            rospy.logerr(e)
