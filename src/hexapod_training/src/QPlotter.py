#!/usr/bin/python3.8
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time
from hexapod_training.msg import QMatrix, QMatrixElement, StateActionPair

class Plotter(object):
    def __init__(self):

        self.fig, self.ax = plt.subplots()
        self.states = []
        self.actions = []

        q_matrix_message = None
        while q_matrix_message is None and not rospy.is_shutdown():
            try:
                q_matrix_message = rospy.wait_for_message("/q_matrix", QMatrix, timeout=3.0)
                self.q_matrix = self.get_matrix_from_msg(q_matrix_message)
                self.ax.set_aspect('auto')
                rospy.loginfo("/q_matrix READY")
            except:
                rospy.loginfo("Current /q_matrix not ready yet, retrying...")

        rospy.on_shutdown(self.clean_up_plots)
        rospy.Subscriber("/q_matrix", QMatrix, self.update_callback)

        states_axis = len(self.states)
        action_axis = len(self.actions)

        self.q_matrix = np.zeros((states_axis, action_axis))
        self.matrice = self.ax.matshow(self.q_matrix)

        new_cbar = plt.colorbar(self.matrice)
        new_cbar.ax.set_autoscale_on(True)

        ani = animation.FuncAnimation(self.fig, self.update, frames=19, interval=100)
        print("Matrix Ready")
        plt.show()

    def update(self,i):
        self.matrice.remove()
        self.matrice = self.ax.matshow(self.q_matrix)
        self.ax.set_aspect('auto')

        self.ax.set_xticks(np.arange(len(self.actions)))
        self.ax.set_yticks(np.arange(len(self.states)))
        self.ax.set_xticklabels(self.actions)
        self.ax.set_yticklabels(self.states)

        plt.setp(self.ax.get_xticklabels(), rotation=45, ha="right",
                 rotation_mode="anchor")

        for i in range(len(self.states)):
            for j in range(len(self.actions)):
                self.text = self.ax.text(j, i, self.q_matrix[i, j],
                                    ha="center", va="center", color="w")


    def update_callback(self,msg):
        self.q_matrix = self.get_matrix_from_msg(msg)

    def clean_up_plots(self):
        dir = rospy.get_param("result_dir")
        # with open(dir + "QMatrix.png", "w+") as file:
        plt.savefig('QMatrix.png', dpi=600)
        plt.close('all')

    def get_matrix_from_msg(self, msg):
        for q_matrix_elem in msg.elements:
            state = q_matrix_elem.pair.state
            action = q_matrix_elem.pair.action

            if not state in self.states:
                self.states.append(state)

            if not action in self.actions:
                self.actions.append(action)

        states_axis = len(self.states)
        actions_axis = len(self.actions)


        qlearn_matrix = np.zeros((states_axis, actions_axis))

        for q_elem in msg.elements:
            state = q_elem.pair.state
            action = q_elem.pair.action
            reward_value = q_elem.reward.data

            state_index = self.states.index(state)

            action_index = self.actions.index(action)
            qlearn_matrix[state_index][action_index] = str(reward_value * 100)

        return qlearn_matrix

if __name__ == '__main__':
    rospy.init_node('q_plotter', anonymous=True, log_level=rospy.INFO)
    plotter = Plotter()