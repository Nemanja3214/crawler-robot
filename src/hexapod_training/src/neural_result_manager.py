#!/usr/bin/python3.8

import json
import rospy
import numpy
import rospkg
import gym
import os
import hexapod_env

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from sb3_contrib import MaskablePPO
from sb3_contrib.common.maskable.evaluation import evaluate_policy
from sb3_contrib.common.maskable.utils import get_action_masks
# This is a drop-in replacement for EvalCallback
# from sb3_contrib.common.maskable.callbacks import MaskableEvalCallback
from stable_baselines3.common.callbacks import BaseCallback
if __name__ == "__main__":
    rospy.loginfo("STARTED LOADER")
    rospy.init_node("neural_result_loader", anonymous=True)
    env = gym.make('Hexapod-v0')
    option = input("Type name of file to load model: ")
    dir = rospy.get_param("result_dir")
    model = MaskablePPO.load(dir + "/" + option + ".zip")
  
    while not rospy.is_shutdown():
        state = env.reset()
        done = False
        while not done:
            action, _ = model.predict(state)
            _, _, done, _ = env.step(action)
        
