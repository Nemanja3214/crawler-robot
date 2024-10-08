#!/usr/bin/python3.8

import math
import serial
import time
import torch

import rospy
import numpy as np
ssc32 = serial.Serial('/dev/ttyS0', 9600, timeout=1)


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
        # rospy.loginfo(joints_states)
        if i != 0:
            command += " "
        command += "#{0} P{1}".format(servo_pins[i], angle_rad_to_pwm(float(joints_states[i])), is_mirrored[i])
    command += "\r"
    command_bytes = str.encode(command)
    # rospy.loginfo(command)
    try:
        ssc32.write(command_bytes)
    except Exception as e:
        rospy.logerr("Failure>>"+str(e))
        exit()

def cleanup():
    rospy.loginfo("CLEANUP")
    set_joint_states(18*[0.0])

    try:
        ssc32.reset_input_buffer()
        rospy.loginfo("CLEANUP INPUT")
        ssc32.reset_output_buffer()
        rospy.loginfo("CLEANUP OUTPUT")
        rospy.loginfo("CLOSING")
        ssc32.close()
    except Exception as e:
        rospy.loginfo(e)

from continous_ppo import Agent, make_env, gym

if __name__ == "__main__":

    rospy.loginfo("STARTED REAL ROBOT EXECUTIONER")
    rospy.init_node("real_robot_executioner", anonymous=True)
    rospy.on_shutdown(cleanup)

    dir = rospy.get_param("result_dir")
    device = torch.device("cpu")
    gym_id = 'Hexapod-v0'
    seed = 3214111
    envs = gym.vector.SyncVectorEnv(
        [make_env(gym_id, seed)]
    )
    model = Agent(envs).to(device)
    model.load_state_dict(torch.load(dir + '/670k_duzi_uspeh.pth'))

    # Set the model to evaluation mode
    model.eval()

    state = torch.Tensor(envs.reset()).to(device)
    done = False
    rew = 0
    success_counter = 0
    step_counter = 0
    num_of_episodes = 100
    steps_sum = 0
    set_joint_states(18*[0.0])

    for i in range(num_of_episodes):
        while not done:

            with torch.no_grad():
                action, _, _, _ = model.get_action_and_value(state)
                set_joint_states(torch.flatten(action))
                state, rew, done, info = envs.step(action)
                state = torch.Tensor(state).to(device)
                
                if info[0]["success"] and done:
                    success_counter+=1
                    rospy.loginfo("SUCCESS")
                elif done:
                    rospy.loginfo("FAIL")

                if done:
                    steps_sum += step_counter
                    rospy.loginfo("STEPS>>>>" + str(step_counter))
                    state = torch.Tensor(envs.reset()).to(device)
                    step_counter = 0
                # rospy.loginfo(env.hexapod_state_object.current_joint_pose)
                # rospy.loginfo("REWARD>>>>" + str(rew))
                if step_counter % 10 == 0:
                    rospy.loginfo("STEP>>>>" + str(step_counter))
                step_counter += 1
        done = False
      
        rospy.loginfo("EPISODE>>>>" + str(i))
    rospy.loginfo("AVG STEPS OF EPISODES>>>" + str(steps_sum/1000))
    rospy.loginfo(rew)

        


