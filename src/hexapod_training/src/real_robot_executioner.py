#!/usr/bin/python3.8

import math
import gym
import serial
import time
import rospy
import torch
import hexapod_env
import numpy as np

# ssc32 = serial.Serial(
#   port='/dev/ttyUSB0',
#   baudrate=9600,
#   write_timeout=1,
#   timeout=1
# )

servo_pins = [31, 27, 23, 15, 11, 7, 30, 26, 22, 14, 10, 6, 29, 25, 21, 13, 9, 5]
is_mirrored = [False, False, False, True, True, True, False, False, False, True, True, True, False, False, False, True, True, True]

def angle_rad_to_pwm(angle_rad, mirrored=False):
    """
    Convert an angle in radians (from -1.5 to 1.5 radians) to PWM pulse width for the MG90S servo, with optional mirroring.
    This version maps -1.5 radians to 1000 µs (1.0 ms) and 1.5 radians to 2000 µs (2.0 ms).
    
    Parameters:
        angle_rad (float): Angle in radians (from -1.5 to 1.5 radians).
        mirrored (bool): If True, mirror the angle mapping (0 radians becomes ±1.5 radians, etc.).

    Returns:
        float: Pulse width in milliseconds (ms) corresponding to the angle.
    """
    # Ensure angle is within valid range
    if angle_rad < -1.5:
        angle_rad = -1.4999
    if angle_rad > 1.5:
        angle_rad = 1.4999

    # If mirrored, reverse the angle mapping
    if mirrored:
        angle_rad = -angle_rad

    # Define pulse width range
    min_pulse_width = 1.0  # Pulse width in ms for -1.5 radians
    max_pulse_width = 2.0  # Pulse width in ms for 1.5 radians
    
    # Calculate pulse width based on angle
    # Linear interpolation between min_pulse_width and max_pulse_width
    pulse_width = min_pulse_width + ((max_pulse_width - min_pulse_width) * (angle_rad + 1.5) / 3.0)

    if pulse_width < 1:
        pulse_width = 1
    if pulse_width > 2:
        pulse_width = 2
    return pulse_width * 1000

def set_joint_states(joints_states):
    command = ""
    for i in range(18):
        if i != 0:
            command += " "
        command += "#{0} P{1}".format(servo_pins[i], angle_rad_to_pwm(joints_states[i], is_mirrored[i]))
    command += "\r"
    command_bytes = str.encode(command)
    # rospy.loginfo(command)
    # try:
    #     # ssc32.write(command_bytes)
    # except Exception as e:
    #     rospy.logerror("Failure>>"+str(e))
    #     exit()

# def cleanup():
#     rospy.loginfo("CLEANUP")

#     try:
#         ssc32.reset_input_buffer()
#         rospy.loginfo("CLEANUP INPUT")
#         ssc32.reset_output_buffer()
#         rospy.loginfo("CLEANUP OUTPUT")
#         rospy.loginfo("CLOSING")
#         ssc32.close()
#     except Exception as e:
#         rospy.loginfo(e)

def test():
    pos = 0.0
    rate = rospy.Rate(1)
    addition = 0.01
    rospy.loginfo("STARTING")
    while not rospy.is_shutdown():
        if abs(pos + addition) > 1.5:
            addition = addition * -1
        pos += addition
    #     joint_states = 18*[pos]

    #     set_joint_states(joint_states)
    #     rospy.loginfo(joint_states)
    #     rate.sleep()


from continous_ppo import Agent, make_env

if __name__ == "__main__":

    rospy.loginfo("STARTED REAL ROBOT EXECUTIONER")
    rospy.init_node("real_robot_executioner", anonymous=True)
    # rospy.on_shutdown(cleanup)
    # test()
    dir = rospy.get_param("result_dir")
    device = torch.device("cpu")
    gym_id = 'Hexapod-v0'
    seed = 3214111
    envs = gym.vector.SyncVectorEnv(
        [make_env(gym_id, seed)]
    )
    model = Agent(envs).to(device)
    model.load_state_dict(torch.load(dir + '/bigger_punishment.pth'))

    # Set the model to evaluation mode
    model.eval()

    state = torch.Tensor(envs.reset()).to(device)
    done = False
    rew = 0
    success_counter = 0
    step_counter = 0
    num_of_episodes = 100
    steps_sum = 0

    for i in range(num_of_episodes):
        while not done:
            state = torch.Tensor(state).to(device)
            with torch.no_grad():
                # WARNING action may be normalized
                action, _, _, _ = model.get_action_and_value(state)
                # set_joint_states(action)
            state, rew, done, info = envs.step(action)
            
            if info[0]["success"] and done:
                success_counter+=1
                rospy.loginfo("SUCCESS")
            elif done:
                rospy.loginfo("FAIL")

            if done:
                steps_sum += step_counter
                state = torch.Tensor(envs.reset()).to(device)
                step_counter = 0
            # rospy.loginfo(env.hexapod_state_object.current_joint_pose)
            # rospy.loginfo("REWARD>>>>" + str(rew))
            step_counter += 1
        done = False
        rospy.loginfo("EPISODE>>>>" + str(i))
    rospy.loginfo("AVG STEPS OF EPISODES>>>" + str(steps_sum/1000))
    rospy.loginfo(rew)

        


