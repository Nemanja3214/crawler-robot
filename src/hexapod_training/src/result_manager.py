#!/usr/bin/python3.8

import json
import rospy
import numpy
import rospkg
import gym
from hexapod_training.msg import ResultMsg
import os
import hexapod_env


if __name__ == "__main__":
    rospy.loginfo("STARTED LOADER")
    rospy.init_node("result_loader", anonymous=True)
    rospy.loginfo("Choose number from 1 to 10")
    env = gym.make('Hexapod-v0')
    while not rospy.is_shutdown():
        option = input("Choose option: ")
        env.reset()
        try:
            dir = rospy.get_param("result_dir")
            with open(dir + "/result"+ option + ".json", "r") as file:
                rospy.loginfo("LOADING")
                
                # rospy.loginfo(os.access(result_path, os.W_OK))
                ob = json.load(file)
                rospy.loginfo(ob)
        except Exception as e:
            rospy.logerr(e)
        for action in ob["actions"]:
            nextState, reward, done, info = env.step(action)
