#!/usr/bin/python3.8

from functools import reduce
import rospy
from std_msgs.msg import String
'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
'''
import gym
import time
import json
import numpy
import random
import qlearn
from gym import wrappers
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg
from hexapod_training.msg import QMatrix, QMatrixElement, StateActionPair


# import our training environment
import hexapod_env
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

def make_msg(q_matrix):
    q_msg = QMatrix()
    for pair, reward in q_matrix.items():
        q_element = QMatrixElement()
        q_element.pair.state = pair[0]
        q_element.pair.action = pair[1]
        q_element.reward = Float64(reward)
        q_msg.elements.append(q_element)
    return q_msg



if __name__ == '__main__':
    
    rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.INFO)
    # rospy.init_node('hexapod_gym', anonymous=True, log_level=rospy.DEBUG)
    rospy.on_shutdown(save)

    # Create the Gym environment
    env = gym.make('Hexapod-v0')

    rospy.logdebug ( "Gym environment done")
    reward_pub = rospy.Publisher('/hexapod/reward', Float64, queue_size=1)
    episode_reward_pub = rospy.Publisher('/hexapod/episode_reward', Float64, queue_size=1)

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('hexapod_training')
    outdir = pkg_path + '/training_results'
    env = gym.wrappers.RecordEpisodeStatistics(env)
    rospy.logdebug("Record Wrapper started")
    
    last_time_steps = numpy.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    Alpha = rospy.get_param("/alpha")
    Epsilon = rospy.get_param("/epsilon")
    Gamma = rospy.get_param("/gamma")
    epsilon_discount = rospy.get_param("/epsilon_discount")
    nepisodes = rospy.get_param("/nepisodes")
    nsteps = rospy.get_param("/nsteps")
    q_pub = rospy.Publisher("/q_matrix", QMatrix, queue_size=1)

    # Initialises the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

    
    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.loginfo ("STARTING Episode #"+str(x))
        
        cumulated_reward = 0
        cumulated_reward_msg = Float64()
        episode_reward_msg = Float64()
        done = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount
        
        # Initialize the environment and get first state of the robot
        rospy.logdebug("env.reset...")
        # Now We return directly the stringuified observations called state
        state = env.reset()

        rospy.logdebug("env.get_state...==>"+str(state))

        action_sequence = numpy.array([])
        
        # for each episode, we test the robot for nsteps
        for i in range(nsteps):
            q_matrix = qlearn.q
            q_matrix_msg = make_msg(q_matrix)
            q_pub.publish(q_matrix_msg)

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            action_sequence = numpy.append(action_sequence, action)
            
            # Execute the action in the environment and get feedback
            rospy.logdebug("###################### Start Step...["+str(i)+"]")
            rospy.logdebug("Action to Perform >> "+str(action))
            nextState, reward, done, info = env.step(action)
            rospy.logdebug("END Step...")
            rospy.logdebug("Reward ==> " + str(reward))
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logdebug("Next state==>" + str(nextState))

            # Make the algorithm learn based on the results
            qlearn.learn(state, action, reward, nextState)
            # print("Q: " + str(qlearn.q))


            # We publish the cumulated reward
            cumulated_reward_msg.data = cumulated_reward
            reward_pub.publish(cumulated_reward_msg)

            # rospy.loginfo(env.hexapod_state_object.get_joint_states())

            if not(done):
                state = nextState
            else:
                rospy.logdebug ("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logdebug("###################### END Step...["+str(i)+"]")

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        episode_reward_msg.data = cumulated_reward
        episode_reward_pub.publish(episode_reward_msg)

        top_episodes_rewards = replace_if_greater(top_episodes_rewards, top_episodes_actions, cumulated_reward, action_sequence)
        rospy.loginfo( ("EP: "+str(x+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))
        rospy.loginfo("TOP EPISODES REWARDS>>>" + str(top_episodes_rewards))
        # rospy.loginfo("TOP EPISODES ACTIONS>>>" + str(top_episodes_actions))
    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    if len(l) != 0:
        rospy.loginfo("Overall score: {:0.2f}".format(last_time_steps.mean()))
        rospy.loginfo("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))
        # rate = rospy.Rate(50)

    else:
        rospy.loginfo("No episode has reached solution")
    env.close()
