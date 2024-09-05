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
#   port='/dev/ttyUSB',
#   baudrate=9600
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
    if angle_rad < -1.5 or angle_rad > 1.5:
        raise ValueError("Angle must be between -1.5 and 1.5 radians")

    # If mirrored, reverse the angle mapping
    if mirrored:
        angle_rad = -angle_rad

    # Define pulse width range
    min_pulse_width = 1.0  # Pulse width in ms for -1.5 radians
    max_pulse_width = 2.0  # Pulse width in ms for 1.5 radians
    
    # Calculate pulse width based on angle
    # Linear interpolation between min_pulse_width and max_pulse_width
    pulse_width = min_pulse_width + ((max_pulse_width - min_pulse_width) * (angle_rad + 1.5) / 3.0)

    return pulse_width

def set_joint_states(joints_states):
    command = b""
    for i in range(joints_states):
        if i != 0:
            command += b" "
        command += b"#{0} P{1}".format(servo_pins[i], angle_rad_to_pwm(joints_states[i], is_mirrored[i]))
    command += "\r"
    rospy.loginfo(command)
    # ssc32.write(command)


from continous_ppo import Agent, make_env

if __name__ == "__main__":
    rospy.loginfo("STARTED REAL ROBOT EXECUTIONER")
    rospy.init_node("real_robot_executioner", anonymous=True)
    dir = rospy.get_param("result_dir")
    device = torch.device("cpu")
    gym_id = 'Hexapod-v0'
    seed = int(time.time())
    envs = gym.vector.SyncVectorEnv(
        [make_env(gym_id, seed)]
    )
    model = Agent(envs).to(device)
    model.load_state_dict(torch.load(dir + '/Hexapod-v0.pth'))

    # Set the model to evaluation mode
    model.eval()

    state = torch.Tensor(envs.reset()).to(device)
    done = False
    rew = 0

      
    while not done:
        state = torch.Tensor(state).to(device)
        with torch.no_grad():
            # WARNING action may be normalized
            action, _, _, _ = model.get_action_and_value(state)
            # set_joint_states(action)
        state, rew, done, _ = envs.step(action)
        # rospy.loginfo(env.hexapod_state_object.current_joint_pose)
        rospy.loginfo("REWARD>>>>" + str(rew))
    rospy.loginfo(rew)
        


    # ssc32.close()
