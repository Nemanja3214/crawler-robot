#!/usr/bin/python3.8

from collections import deque
import random
import rospy
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

class ReplayMemory:
    def __init__(self, size):
        self.memory = deque(maxlen=size)
    
    def add(self, experience):
        self.memory.append(experience)
    
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    
    def size(self):
        return len(self.memory)

class DDQNAgent:
    def __init__(self, state_size, action_size, memory_size, gamma, epsilon_start, epsilon_end, epsilon_decay, learning_rate, batch_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = ReplayMemory(memory_size)
        self.gamma = gamma
        self.epsilon = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.learning_rate = learning_rate
        self.batch_size = batch_size
        self.loss_history = [] 
        
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()
    
    def build_model(self):
        model = tf.keras.Sequential([
            layers.Dense(24, input_dim=self.state_size, activation='relu'),
            layers.Dense(24, activation='relu'),
            layers.Dense(self.action_size, activation='linear')
        ])
        model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=self.learning_rate), loss='mse')
        return model
    
    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())
    
    def act(self, state, i):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        q_values = self.model.predict(np.expand_dims(state, axis=0))
        # rospy.loginfo(q_values)
        return np.argmax(q_values[0])
    
    def replay(self):
        if self.memory.size() < self.batch_size:
            rospy.loginfo("NOT REPLAYING: memory size" + str(self.memory.size()) + ", batch size  " + str(self.batch_size))
            return
        rospy.loginfo("REPLAYING")
        batch = self.memory.sample(self.batch_size)
        for state, action, reward, next_state, done in batch:
            target = reward
            if not done:
                next_action = np.argmax(self.model.predict(np.expand_dims(next_state, axis=0))[0])
                target = reward + self.gamma * self.target_model.predict(np.expand_dims(next_state, axis=0))[0][next_action]
            
            target_f = self.model.predict(np.expand_dims(state, axis=0))
            target_f[0][action] = target
            history = self.model.fit(np.expand_dims(state, axis=0), target_f, epochs=1, verbose=0)
            loss = history.history['loss'][0]
            rospy.loginfo("LOSS>>>>>>>>>>"  + str(loss))
            self.loss_history.append(loss)
        
        if self.epsilon > self.epsilon_end:
            self.epsilon *= self.epsilon_decay

    def plot_statistics(self):
        # Plot rewards and MSE loss
        plt.figure(figsize=(14, 6))

        # plt.subplot(1, 2, 1)
        # plt.plot(self.episode_rewards)
        # plt.title('Rewards per Episode')
        # plt.xlabel('Episode')
        # plt.ylabel('Total Reward')

        plt.subplot(1, 2, 2)
        plt.plot(self.loss_history)
        rospy.loginfo(self.loss_history)
        plt.title('Model Loss During Training')
        plt.xlabel('Training Steps')
        plt.ylabel('Loss')

        plt.tight_layout()
        plt.show()
        import os
        directory = os.path.expanduser('~/catkin_ws/src/hexapod_training')
        file_path = os.path.join(directory, 'foo.png')
        plt.savefig(file_path)
        plt.close()
