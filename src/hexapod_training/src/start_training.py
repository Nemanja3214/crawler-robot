#!/usr/bin/python3.8

from functools import reduce
import rospy
from std_msgs.msg import String
'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
'''
import gym
# import gymnasium as gym
import time
import json
import numpy
import random
import qlearn
# import deep_qlearn
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg
from hexapod_training.msg import QMatrix, QMatrixElement, StateActionPair

from stable_baselines3 import PPO


# import our training environment
import hexapod_env
# import hexapod_env_gymnasium
def replace_if_greater(numeric_arr, side_list, numeric_val, side_val):
    if numpy.size(numeric_arr, 0) < 100:
        numeric_arr = numpy.append(numeric_arr, numeric_val)
        side_list.append(side_val)
        return numeric_arr

    # Find the index of the tuple to replace
    replace_index = numpy.where(numeric_arr < numeric_val)[0]

    # Check if there are any valid indices
    if replace_index.size > 0:
        # Find the index of the tuple with the minimum first value that's less than the new tuple's first value
        min_index = replace_index[numpy.argmin(numeric_arr[replace_index])]
        
        # Replace the tuple at that index
        numeric_arr[min_index] = numeric_val
        side_list[min_index] = side_val
    return numeric_arr


# top 10 episodes (reward, action sequence)
top_episodes_rewards = numpy.array([])
top_episodes_actions = []
def save():
    global model
    results = []
    for i, _ in enumerate(top_episodes_rewards):
        ob = {
            "reward": top_episodes_rewards[i],
            "actions": top_episodes_actions[i].tolist()
        }
        results.append(ob)
        rospy.loginfo(ob)
    results = sorted(results, key=lambda ob: ob["reward"], reverse=True)
    dir = rospy.get_param("result_dir")
    with open(dir + "/results.json", "w+") as file:
        rospy.loginfo("DUMPING")
        try:
            json.dump(results, file)
        except Exception as e:
            rospy.logerr(e)
    model.save(dir + '/Hexapod-v0')

def make_msg(q_matrix):
    q_msg = QMatrix()
    for pair, reward in q_matrix.items():
        q_element = QMatrixElement()
        q_element.pair.state = pair[0]
        q_element.pair.action = pair[1]
        q_element.reward = Float64(reward)
        q_msg.elements.append(q_element)
    return q_msg


# def qlearn_main():
#     global top_episodes_rewards, top_episodes_actions
    
#     rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.INFO)
#     # rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.DEBUG)
#     rospy.on_shutdown(save)

#     # Create the Gym environment
#     env = gym.make('Hexapod-v0')

#     rospy.logdebug ( "Gym environment done")
#     reward_pub = rospy.Publisher('/hexapod/reward', Float64, queue_size=1)
#     episode_reward_pub = rospy.Publisher('/hexapod/episode_reward', Float64, queue_size=1)

#     # Set the logging system
#     rospack = rospkg.RosPack()
#     pkg_path = rospack.get_path('hexapod_training')
#     outdir = pkg_path + '/training_results'
    
#     last_time_steps = numpy.ndarray(0)

#     # Loads parameters from the ROS param server
#     # Parameters are stored in a yaml file inside the config directory
#     # They are loaded at runtime by the launch file
#     Alpha = rospy.get_param("/alpha")
#     Epsilon = rospy.get_param("/epsilon")
#     Gamma = rospy.get_param("/gamma")
#     epsilon_discount = rospy.get_param("/epsilon_discount")
#     nepisodes = rospy.get_param("/nepisodes")
#     nsteps = rospy.get_param("/nsteps")
#     q_pub = rospy.Publisher("/q_matrix", QMatrix, queue_size=1)

#     # Initialises the algorithm that we are going to use for learning
#     q = qlearn.QLearn(actions=range(env.action_space.n),
#                         alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
#     initial_epsilon = q.epsilon

#     start_time = time.time()
#     highest_reward = 0

    
#     # Starts the main training loop: the one about the episodes to do
#     for x in range(nepisodes):
#         rospy.loginfo ("STARTING Episode #"+str(x))
        
#         cumulated_reward = 0
#         cumulated_reward_msg = Float64()
#         episode_reward_msg = Float64()
#         done = False
#         if q.epsilon > 0.05:
#             q.epsilon *= epsilon_discount
        
#         # Initialize the environment and get first state of the robot
#         rospy.logdebug("env.reset...")
#         # Now We return directly the stringuified observations called state
#         state = env.reset()

#         rospy.logdebug("env.get_state...==>"+str(state))

#         action_sequence = numpy.array([])
        
#         # for each episode, we test the robot for nsteps
#         for i in range(nsteps):
#             # q_matrix = q.q
#             # q_matrix_msg = make_msg(q_matrix)
#             # q_pub.publish(q_matrix_msg)

#             # Pick an action based on the current state
#             action = q.chooseAction(state)
#             action_sequence = numpy.append(action_sequence, action)
            
#             # Execute the action in the environment and get feedback
#             rospy.logdebug("###################### Start Step...["+str(i)+"]")
#             rospy.logdebug("Action to Perform >> "+str(action))
#             nextState, reward, done, info = env.step(action)
#             rospy.logdebug("END Step...")
#             rospy.logdebug("Reward ==> " + str(reward))
#             cumulated_reward += reward
#             if highest_reward < cumulated_reward:
#                 highest_reward = cumulated_reward

#             rospy.logdebug("Next state==>" + str(nextState))

#             # Make the algorithm learn based on the results
#             q.learn(state, action, reward, nextState)
#             # print("Q: " + str(q.q))


#             # We publish the cumulated reward
#             cumulated_reward_msg.data = cumulated_reward
#             reward_pub.publish(cumulated_reward_msg)

#             # rospy.loginfo(env.hexapod_state_object.get_joint_states())

#             if not(done):
#                 state = nextState
#             else:
#                 rospy.logdebug ("DONE")
#                 last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
#                 break

#             rospy.logdebug("###################### END Step...["+str(i)+"]")

#         m, s = divmod(int(time.time() - start_time), 60)
#         h, m = divmod(m, 60)
#         episode_reward_msg.data = cumulated_reward
#         episode_reward_pub.publish(episode_reward_msg)

#         top_episodes_rewards = replace_if_greater(top_episodes_rewards, top_episodes_actions, cumulated_reward, action_sequence)
#         rospy.loginfo( ("EP: "+str(x+1)+" - [alpha: "+str(round(q.alpha,2))+" - gamma: "+str(round(q.gamma,2))+" - epsilon: "+str(round(q.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))
#         rospy.loginfo("TOP EPISODES REWARDS>>>" + str(top_episodes_rewards))
#         # rospy.loginfo("TOP EPISODES ACTIONS>>>" + str(top_episodes_actions))
#     rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(q.alpha)+"|"+str(q.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

#     l = last_time_steps.tolist()
#     l.sort()

#     if len(l) != 0:
#         rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
#         rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
#         # rate = rospy.Rate(50)

#     else:
#         rospy.loginfo("No episode has reached solution")
#     env.close()

# def scale_state(state):
#     return (state + 100) / 200

# def deep_qlearn_main():
#     global top_episodes_rewards, top_episodes_actions
#     rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.INFO)
#     # rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.DEBUG)

 

#     # Create the Gym environment
#     env = gym.make('Hexapod-v0')

#     rospy.logdebug ( "Gym environment done")
#     reward_pub = rospy.Publisher('/hexapod/reward', Float64, queue_size=1)
#     episode_reward_pub = rospy.Publisher('/hexapod/episode_reward', Float64, queue_size=1)

#     # Set the logging system
#     rospack = rospkg.RosPack()
#     pkg_path = rospack.get_path('hexapod_training')
#     outdir = pkg_path + '/training_results'
    
#     last_time_steps = numpy.ndarray(0)

#     # Loads parameters from the ROS param server
#     # Parameters are stored in a yaml file inside the config directory
#     # They are loaded at runtime by the launch file
#     Alpha = rospy.get_param("/alpha")
#     Epsilon = rospy.get_param("/epsilon")
#     Gamma = rospy.get_param("/gamma")
#     epsilon_discount = rospy.get_param("/epsilon_discount")
#     nepisodes = rospy.get_param("/nepisodes")
#     nsteps = rospy.get_param("/nsteps")
#     # q_pub = rospy.Publisher("/q_matrix", QMatrix, queue_size=1)

#     state_size = len(env.reset())

#     # Initialises the algorithm that we are going to use for learning
#     deep_qlearn_agent = deep_qlearn.DDQNAgent(
#             state_size=state_size,  # Adjust as needed
#             action_size=36,  # Adjust as needed
#             memory_size=10000,
#             gamma=0.99,
#             epsilon_start=Epsilon,
#             epsilon_end=0.01,
#             epsilon_decay=0.995,
#             learning_rate=0.01,
#             batch_size=64
#         )
#     sync_rate = 6
#     initial_epsilon = Epsilon

#     start_time = time.time()
#     highest_reward = 0

#     def save_with_plot():
#         save()
#         deep_qlearn_agent.plot_statistics()
        
#     rospy.on_shutdown(save_with_plot)
    
#     # Starts the main training loop: the one about the episodes to do
#     for x in range(nepisodes):
#         rospy.loginfo ("STARTING Episode #"+str(x))
        
#         cumulated_reward = 0
#         cumulated_reward_msg = Float64()
#         episode_reward_msg = Float64()
#         done = False
        
#         # Initialize the environment and get first state of the robot
#         rospy.logdebug("env.reset...")
#         # Now We return directly the stringuified observations called state
#         state = env.reset()
#         state = scale_state(state)


#         rospy.logdebug("env.get_state...==>"+str(state))

#         action_sequence = numpy.array([])
        
#         # for each episode, we test the robot for nsteps
#         for i in range(nsteps):
#             # q_matrix = qlearn.q
#             # q_matrix_msg = make_msg(q_matrix)
#             # q_pub.publish(q_matrix_msg)

#             # Pick an action based on the current state
#             invalid_action_counter = 1
#             action  = deep_qlearn_agent.act(state, invalid_action_counter)
#             # pick new action until it is valid
#             while not env.is_valid_action(action):
#                 action  = deep_qlearn_agent.act(state, invalid_action_counter)
#                 invalid_action_counter += 1
#             # rospy.loginfo("STATE>>>>>>" + str(state))
#             # rospy.loginfo("SCALED STATE>>>>>>" + str(scale_state(state)))
#             action_sequence = numpy.append(action_sequence, action)
            
#             # Execute the action in the environment and get feedback
#             rospy.logdebug("###################### Start Step...["+str(i)+"]")
#             rospy.logdebug("Action to Perform >> "+str(action))
#             nextState, reward, done, info = env.step(action)
#             nextState = scale_state(state)
#             rospy.logdebug("END Step...")
#             rospy.logdebug("Reward ==> " + str(reward))
#             cumulated_reward += reward
#             if highest_reward < cumulated_reward:
#                 highest_reward = cumulated_reward

#             rospy.logdebug("Next state==>" + str(nextState))

#             # We publish the cumulated reward
#             cumulated_reward_msg.data = cumulated_reward
#             reward_pub.publish(cumulated_reward_msg)

#             # rospy.loginfo(env.hexapod_state_object.get_joint_states())

#             if not(done):
#                 state = nextState
#             else:
#                 rospy.logdebug ("DONE")
#                 deep_qlearn_agent.memory.add((state, action, cumulated_reward, nextState, done))
#                 if i % sync_rate == 0 and x > 10:
#                     deep_qlearn_agent.replay()
#                 last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
#                 break

#             rospy.logdebug("###################### END Step...["+str(i)+"]")

#         m, s = divmod(int(time.time() - start_time), 60)
#         h, m = divmod(m, 60)
#         episode_reward_msg.data = cumulated_reward
#         episode_reward_pub.publish(episode_reward_msg)

#         top_episodes_rewards = replace_if_greater(top_episodes_rewards, top_episodes_actions, cumulated_reward, action_sequence)
#         rospy.loginfo( ("EP: "+str(x+1)+" - [alpha: "+str(round(Alpha,2))+" - gamma: "+str(round(Gamma,2))+" - epsilon: "+str(round(Epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))
#         rospy.loginfo("TOP EPISODES REWARDS>>>" + str(top_episodes_rewards))
#         # rospy.loginfo("TOP EPISODES ACTIONS>>>" + str(top_episodes_actions))
#     rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(Alpha)+"|"+str(Gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

#     deep_qlearn_agent.plot_statistics()
#     l = last_time_steps.tolist()
#     l.sort()

#     if len(l) != 0:
#         rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
#         rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
#         # rate = rospy.Rate(50)

#     else:
#         rospy.loginfo("No episode has reached solution")
#     env.close()
from stable_baselines3.common.env_checker import check_env
from sb3_contrib import MaskablePPO
from sb3_contrib.common.maskable.evaluation import evaluate_policy
from sb3_contrib.common.maskable.utils import get_action_masks
# This is a drop-in replacement for EvalCallback

from sb3_contrib.common.wrappers import ActionMasker
# from sb3_contrib.common.maskable.callbacks import MaskableEvalCallback
from stable_baselines3.common.callbacks import BaseCallback
# from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
import os

# class SaveOnBestTrainingRewardCallback(BaseCallback):
#     """
#     Callback for saving a model (the check is done every ``check_freq`` steps)
#     based on the training reward (in practice, we recommend using ``EvalCallback``).

#     :param check_freq:
#     :param log_dir: Path to the folder where the model will be saved.
#       It must contains the file created by the ``Monitor`` wrapper.
#     :param verbose: Verbosity level.
#     """
#     def __init__(self, check_freq: int, log_dir: str, verbose: int = 1):
#         super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
#         self.check_freq = check_freq
#         self.log_dir = log_dir
#         self.save_path = os.path.join(log_dir, 'best_model')
#         self.best_mean_reward = -numpy.inf

#     def _init_callback(self) -> None:
#         # Create folder if needed
#         if self.save_path is not None:
#             os.makedirs(self.save_path, exist_ok=True)

#     def _on_step(self) -> bool:
#         if self.n_calls % self.check_freq == 0:

#           # Retrieve training reward
#           x, y = ts2xy(load_results(self.log_dir), 'timesteps')
#           if len(x) > 0:
#               # Mean training reward over the last 100 episodes
#               mean_reward = numpy.mean(y[-100:])
#               if self.verbose > 0:
#                 print(f"Num timesteps: {self.num_timesteps}")
#                 print(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}")

#               # New best model, you could save the agent here
#               if mean_reward > self.best_mean_reward:
#                   self.best_mean_reward = mean_reward
#                   # Example for saving best model
#                   if self.verbose > 0:
#                     print(f"Saving new best model to {self.save_path}")
#                   self.model.save(self.save_path)

#         return True
import datetime
class TensorBoardCallback(BaseCallback):
    def __init__(self, log_dir: str, n_eval_freq: int, name: str, *args, **kwargs):
        super(TensorBoardCallback, self).__init__(*args, **kwargs)
        self.log_dir = log_dir
        self.n_eval_freq = n_eval_freq
        self.name = name

    def _on_training_start(self) -> None:
        from tensorboardX import SummaryWriter
        run_name = self.name
        self.log_dir = os.path.join(self.log_dir, run_name)
        self.writer = SummaryWriter(log_dir=self.log_dir)
        
    def _on_step(self) -> bool:
        if self.num_timesteps % self.n_eval_freq == 0:
            self.writer.add_scalar('train/episode_reward', self.locals['rewards'], self.num_timesteps)
        return True

    def _on_training_end(self) -> None:
        self.writer.close()

global model
def mask_fn(env: gym.Env) -> numpy.ndarray:
    return env.action_masks()

def ppo_main():
    global top_episodes_rewards, top_episodes_actions, model
    
    rospy.on_shutdown(save)

    # Create the Gym environment
    env = gym.make('Hexapod-v0')
    env = ActionMasker(env, mask_fn)
    check_env(env)

    # rospy.logdebug ( "Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('hexapod_training')

    # Create log dir
    log_dir = pkg_path + "/tmp/"

    os.makedirs(log_dir, exist_ok=True)
    name = str(datetime.datetime.now())
    tensorboard_callback = TensorBoardCallback(log_dir=log_dir, name=name, n_eval_freq=1000)
    rospy.loginfo("BEFORE CREATING MODEL")
    
    model = MaskablePPO("MlpPolicy", env, gamma=0.99, seed=32, verbose=1, learning_rate=0.0003)
    # callback = SaveOnBestTrainingRewardCallback(check_freq=1000, log_dir=log_dir)

    model.learn(total_timesteps=250000, callback=tensorboard_callback)
    model.save(name)

import continous_ppo

if __name__ == '__main__':
    rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.INFO)
    # rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.DEBUG)
    # ppo_main()
    # qlearn_main()
    # rospy.on_shutdown(save)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('hexapod_training')
    log_dir = pkg_path + "/tmp/"
    os.makedirs(log_dir, exist_ok=True)
    name = str(datetime.datetime.now())
    tensorboard_callback = TensorBoardCallback(log_dir=log_dir, name=name, n_eval_freq=1000)

    # Create log dir
    log_dir = pkg_path + "/tmp/"
    continous_ppo.run(log_dir)
