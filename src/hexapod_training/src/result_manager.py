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

def load():
    with open(result_path, "r") as file:
        result = json.load(file)
        for reward, actions in result:
            print("REWARDS>>>>>>" + str(reward))
            print("ACTIONS>>>>>>" + str(actions))

    return result

if __name__ == "__main__":
    rospy.loginfo("STARTED SAVER")
    # action_sequence1 = numpy.array([1.0, 2.0])
    # action_sequence2 = numpy.array([3.0, 4.0])
    # top_episodes_rewards = numpy.array([22.0, 33.0])
    # top_episodes_actions = numpy.array([action_sequence1, action_sequence2])

    # save(top_episodes_rewards, top_episodes_actions)
    rospy.init_node("result_saver", anonymous=True)
    for i in range(1, 11):
        rospy.Subscriber("/result" + str(i), ResultMsg, save)
        rospy.loginfo("SUB FOR" + "/result" + str(i))
    rospy.spin()