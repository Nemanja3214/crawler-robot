#!/usr/bin/python3.8

import json
import rospy
import numpy
import rospkg
import gym
import os
import hexapod_env


if __name__ == "__main__":
    rospy.loginfo("STARTED LOADER")
    rospy.init_node("result_loader", anonymous=True)
    rospy.loginfo("Choose order number of result ")
    env = gym.make('Hexapod-v0')
    dir = rospy.get_param("result_dir")
    with open(dir + "/results.json", "r") as file:
        rospy.loginfo("LOADING")
        
        # rospy.loginfo(os.access(result_path, os.W_OK))
        results = json.load(file)
        # rospy.loginfo(results)
    for i, ob in enumerate(results):
        print(str(i) + " " + str(ob["reward"]))
    while not rospy.is_shutdown():
        option = input("Choose option: ")
        env.reset()
        try:
            ob = results[int(option)]
        except Exception as e:
            rospy.logerr(e)
        for action in ob["actions"]:
            nextState, reward, done, info = env.step(action)
