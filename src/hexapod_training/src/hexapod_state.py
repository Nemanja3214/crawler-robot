#!/usr/bin/python3.8

import rospy
import copy
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
import tf
import numpy
import math


class HexapodState(object):

    def __init__(self, max_height, min_height, abs_max_roll, abs_max_pitch, list_of_observations, joint_limits, episode_done_criteria, joint_increment_value = 0.05, done_reward = -1000.0, alive_reward=10.0, desired_force=7.08, desired_roll=0.0, desired_pitch=0.0, desired_yaw=0.0, weight_r1=1.0, weight_r2=1.0, weight_r3=1.0, weight_r4=1.0, weight_r5=1.0, weight_r6=1.0,weight_r7=1.0, discrete_division=10, maximum_base_linear_acceleration=3000.0, maximum_base_angular_velocity=20.0, maximum_joint_effort=10.0):
        rospy.logdebug("Starting HexapodState Class object...")
       
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw
        self._desired_pitch = desired_pitch
        self._desired_roll = desired_roll

        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3
        self._weight_r4 = weight_r4
        self._weight_r5 = weight_r5
        self._weight_r6 = weight_r6
        self._weight_r7 = weight_r7

        self._list_of_observations = list_of_observations

        # Dictionary with the max and min of each of the joints
        self._joint_limits = joint_limits
        self.touching = False

        # Maximum base linear acceleration values
        self.maximum_base_linear_acceleration = maximum_base_linear_acceleration

        # Maximum Angular Velocity value
        # By maximum means the value that we consider relevant maximum, the sensor might pick up higher
        # But its an equilibrium between precission and number of divisions of the sensors data.
        self.maximum_base_angular_velocity = maximum_base_angular_velocity

        self.maximum_joint_effort = maximum_joint_effort

        # List of all the Done Episode Criteria
        self._episode_done_criteria = episode_done_criteria
        assert len(self._episode_done_criteria) != 0, "Episode_done_criteria list is empty. Minimum one value"

        self._discrete_division = discrete_division

        # We init the observation ranges and We create the bins now for all the observations
        self.init_bins()

        self.base_position = Point()
        self.base_orientation = Quaternion()
        self.base_angular_velocity = Vector3()
        self.base_linear_acceleration = Vector3()
        self.contact_force = Vector3()
        self.joints_state = JointState()

        # Odom we only use it for the height detection and planar position ,
        #  because in real robots this data is not trivial.
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # We use the IMU for orientation and linearacceleration detection
        rospy.Subscriber("/hexapod/imu/data", Imu, self.imu_callback)
        # We use it to get the contact force, to know if its in the air or stumping too hard.
        rospy.Subscriber("/contactsensor_state", ContactsState, self.contact_callback)
        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/hexapod/joint_states", JointState, self.joints_state_callback)

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
                self.base_position = data_pose.pose.pose.position
                rospy.logdebug("Current odom READY")
            except:
                rospy.logdebug("Current odom pose not ready yet, retrying for getting robot base_position")

        imu_data = None
        while imu_data is None and not rospy.is_shutdown():
            try:
                imu_data = rospy.wait_for_message("/hexapod/imu/data", Imu, timeout=0.1)
                self.base_orientation = imu_data.orientation
                self.base_angular_velocity = imu_data.angular_velocity
                self.base_linear_acceleration = imu_data.linear_acceleration
                rospy.logdebug("Current imu_data READY")
            except:
                rospy.logdebug("Current imu_data not ready yet, retrying for getting robot base_orientation, and base_linear_acceleration")

        contacts_data = None
        while contacts_data is None and not rospy.is_shutdown():
            try:
                contacts_data = rospy.wait_for_message("/contactsensor_state", ContactsState, timeout=0.1)
                for state in contacts_data.states:
                    self.contact_force = state.total_wrench.force
                rospy.logdebug("Current contacts_data READY")
            except:
                rospy.logdebug("Current contacts_data not ready yet, retrying")

        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/hexapod/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.logdebug("Current joint_states READY")
            except Exception as e:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")

    def set_desired_world_point(self, x, y, z):
        """
        Point where you want the hexapod to be
        :return:
        """
        self.desired_world_point.x = x
        self.desired_world_point.y = y
        self.desired_world_point.z = z


    def get_base_height(self):
        height = self.base_position.z
        rospy.logdebug("BASE-HEIGHT="+str(height))
        return height

    def get_base_rpy(self):
        euler_rpy = Vector3()
        euler = tf.transformations.euler_from_quaternion(
            [self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])

        euler_rpy.x = euler[0]
        euler_rpy.y = euler[1]
        euler_rpy.z = euler[2]
        return euler_rpy

    def get_base_angular_velocity(self):
        return self.base_angular_velocity

    def get_base_linear_acceleration(self):
        return self.base_linear_acceleration

    def get_distance_from_point(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((self.base_position.x, self.base_position.y, self.base_position.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_contact_force_magnitude(self):
        """
        You will see that because the X axis is the one pointing downwards, it will be the one with
        higher value when touching the floor
        For a Robot of total mas of 0.55Kg, a gravity of 9.81 m/sec**2, Weight = 0.55*9.81=5.39 N
        Falling from around 5centimetres ( negligible height ), we register peaks around
        Fx = 7.08 N
        :return:
        """
        contact_force = self.contact_force
        contact_force_np = numpy.array((contact_force.x, contact_force.y, contact_force.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude

    def get_joint_states(self):
        return self.joints_state

    def odom_callback(self,msg):
        self.base_position = msg.pose.pose.position

    def imu_callback(self,msg):
        self.base_orientation = msg.orientation
        self.base_angular_velocity = msg.angular_velocity
        self.base_linear_acceleration = msg.linear_acceleration

    def is_straight(self):
        unit_orientation = self.base_orientation / numpy.linalg.norm(self.base_orientation)
        similar = numpy.dot(unit_orientation, numpy.array(0.0, 0.0, -1.0))
        rospy.logdebug("cos of z and unit is>>" + str(similar))
        rospy.logdebug("angle is >>" + str(numpy.arccos(similar)))
        return similar >= 0.75

    def contact_callback(self,msg):
        """
        /lowerleg_contactsensor_state/states[0]/contact_positions ==> PointContact in World
        /lowerleg_contactsensor_state/states[0]/contact_normals ==> NormalContact in World

        ==> One is an array of all the forces, the other total,
         and are relative to the contact link referred to in the sensor.
        /lowerleg_contactsensor_state/states[0]/wrenches[]
        /lowerleg_contactsensor_state/states[0]/total_wrench
        :param msg:
        :return:
        """
        # rospy.logdebug(msg)
        if len(msg.states) != 0:
            self.touching = True 
        else:
            self.touching = False
        for state in msg.states:
            self.contact_force = state.total_wrench.force
        

    def joints_state_callback(self,msg):
        self.joints_state = msg

    def hexapod_height_ok(self):

        height_ok = self._min_height <= self.get_base_height() < self._max_height
        return height_ok

    def is_stand_up(self):
        orientation_rpy = self.get_base_rpy()
        pitch_ok = abs(orientation_rpy.y ) < 0.5 #~30 deg
        rospy.logdebug("DISTANCE FROM Z >>>>" + str(abs(self.get_base_height() - self.desired_world_point.z)))
        is_standing = self.get_base_height() > self.desired_world_point.z and pitch_ok
        return is_standing

    def hexapod_orientation_ok(self):

        orientation_rpy = self.get_base_rpy()
        roll_ok = self._abs_max_roll > abs(orientation_rpy.x)
        pitch_ok = self._abs_max_pitch > abs(orientation_rpy.y)
        orientation_ok = roll_ok and pitch_ok
        return orientation_ok

    def calculate_reward_joint_position(self, weight=1.0):
        """
        We calculate reward base on the joints configuration. The more near 0 the better.
        :return:
        """
        acumulated_joint_pos = 0.0
        for joint_pos in self.joints_state.position:
            # Abs to remove sign influence, it doesnt matter the direction of turn.
            acumulated_joint_pos += abs(joint_pos)
            rospy.logdebug("calculate_reward_joint_position>>acumulated_joint_pos=" + str(acumulated_joint_pos))
        reward = weight * acumulated_joint_pos
        rospy.logdebug("calculate_reward_joint_position>>reward=" + str(reward))
        return reward

    def calculate_reward_joint_effort(self, weight=1.0):
        """
        We calculate reward base on the joints effort readings. The more near 0 the better.
        :return:
        """
        acumulated_joint_effort = 0.0
        for joint_effort in self.joints_state.effort:
            # Abs to remove sign influence, it doesnt matter the direction of the effort.
            acumulated_joint_effort += abs(joint_effort)
            rospy.logdebug("calculate_reward_joint_effort>>joint_effort=" + str(joint_effort))
            rospy.logdebug("calculate_reward_joint_effort>>acumulated_joint_effort=" + str(acumulated_joint_effort))
        reward = weight * acumulated_joint_effort
        rospy.logdebug("calculate_reward_joint_effort>>reward=" + str(reward))
        return reward

    def calculate_reward_contact_force(self, weight=1.0):
        """
        We calculate reward base on the contact force.
        The nearest to the desired contact force the better.
        We use exponential to magnify big departures from the desired force.
        Default ( 7.08 N ) desired force was taken from reading of the robot touching
        the ground from a negligible height of 5cm.
        :return:
        """
        force_magnitude = self.get_contact_force_magnitude()
        force_displacement = force_magnitude - self._desired_force

        rospy.logdebug("calculate_reward_contact_force>>force_magnitude=" + str(force_magnitude))
        rospy.logdebug("calculate_reward_contact_force>>force_displacement=" + str(force_displacement))
        # Abs to remove sign
        reward = weight * abs(force_displacement)
        rospy.logdebug("calculate_reward_contact_force>>reward=" + str(reward))
        return reward

    def calculate_reward_orientation(self, weight=1.0):
        """
        We calculate the reward based on the orientation.
        The more its closser to 0 the better because it means its upright
        desired_yaw is the yaw that we want it to be.
        to praise it to have a certain orientation, here is where to set it.
        :return:
        """
        curren_orientation = self.get_base_rpy()
        # yaw_displacement = curren_orientation.z - self._desired_yaw
        roll_displacement = curren_orientation.x - self._desired_roll
        pitch_displacement = curren_orientation.y - self._desired_pitch

        yaw_displacement = 0
        # roll_displacement = 0
        # pitch_displacement = 0
        
        rospy.logdebug("calculate_reward_orientation>>[R,P,Y]=" + str(curren_orientation))
        acumulated_orientation_displacement = abs(roll_displacement) + abs(pitch_displacement) + abs(yaw_displacement)
        reward = weight * acumulated_orientation_displacement
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward

    def calculate_reward_distance_from_des_point(self, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = self.get_distance_from_point(self.desired_world_point)
        reward = weight * distance
        rospy.logdebug("calculate_reward_orientation>>reward=" + str(reward))
        return reward
    
    def calculate_touching_reward(self, weight=1.0):
        """
        We calculate the reward based on thorax touching floor
        If not touching that is better.
        :param weight:
        :return:reward
        """
        if self.touching:
            return weight
        return -100.0 * weight

    def calculate_synchro_reward(self, weight=1.0):
        """
        We calculate the reward based synchronisation of joins
        on one side.
        :param weight:
        :return:reward
        """
        total_sum = 0

        # tibia_l1, tibia_l2, tibia_l3
        # then tibia_r1, tibia_r2, tibia_r3
        # then same for other joints
        for side_part_i in range(0, len(self.joints_state.position), 3):
            total_sum += abs(self.joints_state.position[side_part_i] - self.joints_state.position[side_part_i + 1]) + \
            abs(self.joints_state.position[side_part_i] - self.joints_state.position[side_part_i + 2]) + \
            abs(self.joints_state.position[side_part_i + 1] - self.joints_state.position[side_part_i + 2])

        return weight * total_sum
            

    def calculate_total_reward(self):
        r1 = self.calculate_reward_joint_position(self._weight_r1)
        r2 = self.calculate_reward_joint_effort(self._weight_r2)
        # Desired Force in Newtons, taken form idle contact with 9.81 gravity.
        r3 = self.calculate_reward_contact_force(self._weight_r3)
        # r3 = 0
        r4 = self.calculate_reward_orientation(self._weight_r4)
        r5 = self.calculate_reward_distance_from_des_point(self._weight_r5)
        r6 = self.calculate_touching_reward(self._weight_r6)
        r7 = self.calculate_synchro_reward(self._weight_r7)

        # The sign depend on its function.
        total_reward = self._alive_reward - r1 - r2 - r3 - r4 - r5 - r6
    
        rospy.logdebug("###############")
        rospy.logdebug("alive_bonus=" + str(self._alive_reward))
        rospy.logdebug("r1 joint_position=" + str(r1))
        rospy.logdebug("r2 joint_effort=" + str(r2))
        rospy.logdebug("r3 contact_force=" + str(r3))
        rospy.logdebug("r4 orientation=" + str(r4))
        rospy.logdebug("r5 distance=" + str(r5))
        rospy.logdebug("r6 touching=" + str(r6))
        rospy.logdebug("total_reward=" + str(total_reward))
        rospy.logdebug("###############")

        return total_reward

    def get_observations(self):
        """
        Returns the state of the robot needed for OpenAI QLearn Algorithm
        The state will be defined by an array of the:
        1) distance from desired point in meters
        2) The pitch orientation in radians
        3) the Roll orientation in radians
        4) the Yaw orientation in radians
        5) Force in contact sensor in Newtons
        6-7-8) State of the 3 joints in radians

        observation = [distance_from_desired_point,
                 base_roll,
                 base_pitch,
                 base_yaw,
                 base_angular_vel_x,
                 base_angular_vel_y,
                 base_angular_vel_z,
                 base_linear_acceleration_x,
                 base_linear_acceleration_y,
                 base_linear_acceleration_z,
                 contact_force,
                 joint_states,
                 joint_effort]

        :return: observation
        """

        distance_from_desired_point = self.get_distance_from_point(self.desired_world_point)

        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z

        base_angular_velocity = self.get_base_angular_velocity()
        base_angular_vel_x = base_angular_velocity.x
        base_angular_vel_y = base_angular_velocity.y
        base_angular_vel_z = base_angular_velocity.z

        base_linear_acceleration = self.get_base_linear_acceleration()
        base_linear_acceleration_x = base_linear_acceleration.x
        base_linear_acceleration_y = base_linear_acceleration.y
        base_linear_acceleration_z = base_linear_acceleration.z

        contact_force = self.get_contact_force_magnitude()

        joint_states = self.get_joint_states()

        joint_state_positions = [position for position in joint_states.position]
        efforts = [effort for effort in joint_states.effort]
        touching_ground = 1 if self.touching else 0

        observation = []
        rospy.logdebug("List of Observations==>"+str(self._list_of_observations))
        # if scalar simply append, if list extend so that list is flat
        for obs_name in self._list_of_observations:
            if obs_name == "distance_from_desired_point":
                observation.append(distance_from_desired_point)
            elif obs_name == "base_roll":
                observation.append(base_roll)
            elif obs_name == "base_pitch":
                observation.append(base_pitch)
            elif obs_name == "base_yaw":
                observation.append(base_yaw)
            elif obs_name == "contact_force":
                observation.append(contact_force)
            elif obs_name == "joint_states":
                observation.extend(joint_state_positions)
            elif obs_name == "joint_effort":
                observation.extend(efforts)
            elif obs_name == "base_angular_vel_x":
                observation.append(base_angular_vel_x)
            elif obs_name == "base_angular_vel_y":
                observation.append(base_angular_vel_y)
            elif obs_name == "base_angular_vel_z":
                observation.append(base_angular_vel_z)
            elif obs_name == "base_linear_acceleration_x":
                observation.append(base_linear_acceleration_x)
            elif obs_name == "base_linear_acceleration_y":
                observation.append(base_linear_acceleration_y)
            elif obs_name == "base_linear_acceleration_z":
                observation.append(base_linear_acceleration_z)
            elif obs_name == "touching_ground":
                observation.append(touching_ground)
            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

        return observation

    def get_state_as_string(self, observation):
        """
        This function will do two things:
        1) It will make discrete the observations
        2) Will convert the discrete observations in to state tags strings
        :param observation:
        :return: state
        """
        observations_discrete = self.assign_bins(observation)
        string_state = ''.join(map(str, observations_discrete))
        rospy.logdebug("STATE==>"+str(string_state))
        return string_state

    def get_state_as_bins(self, observation):
        """
        This function will do two things:
        1) It will make discrete the observations
        2) Will convert the discrete observations in to state tags strings
        :param observation:
        :return: state
        """
        observations_discrete = self.assign_bins(observation)
        return observations_discrete

    def assign_bins(self, observation):
        """
        Will make observations discrete by placing each value into its corresponding bin
        :param observation:
        :return:
        """
        rospy.logdebug("Observations>>"+str(observation))
        rospy.logdebug("Observations list>>"+str(self._list_of_observations))
        state_discrete = numpy.zeros(len(self._list_of_observations), dtype=numpy.int32)
        for i in range(len(self._list_of_observations)):
            # We convert to int because anyway it will be round floats. We add Right True to include limits
            # Ex: [-20, 0, 20], value=-20 ==> index=0, In right = False, would be index=1
            state_discrete[i] = int(numpy.digitize(observation[i], self._bins[i], right=True))
            rospy.logdebug("bin="+str(self._bins[i])+"obs="+str(observation[i])+",end_val="+str(state_discrete[i]))

        rospy.logdebug(str(state_discrete))
        return state_discrete

    def init_bins(self):
        """
        We initalise all related to the bins
        :return:
        """
        self.fill_observations_ranges()
        self.create_bins()

    def fill_observations_ranges(self):
        """
        We create the dictionary for the ranges of the data related to each observation
        :return:
        """
        self._obs_range_dict = {}
        for obs_name in self._list_of_observations:

            if obs_name == "distance_from_desired_point":
                # We consider the range as based on the range of distance allowed in height
                delta = self._max_height - self._min_height
                max_value = delta
                min_value = -delta
            elif obs_name == "base_roll":
                max_value = self._abs_max_roll
                min_value = -self._abs_max_roll
            elif obs_name == "base_pitch":
                max_value = self._abs_max_pitch
                min_value = -self._abs_max_pitch
            elif obs_name == "base_yaw":
                # We consider that 360 degrees is max range
                max_value = 2*math.pi
                min_value = -2*math.pi
            elif obs_name == "contact_force":
                # We consider that no force is the minimum, and the maximum is 2 times the desired
                # We dont want to make a very big range because we might loose the desired force
                # in the middle.
                max_value = 2*self._desired_force
                min_value = 0.0

            elif obs_name == "joint_states":
                # We consider the URDF maximum values
                max_value = self._joint_limits["max"]
                min_value = self._joint_limits["min"]
            elif obs_name == "joint_effort":
                # We consider the URDF maximum values
                max_value = self.maximum_joint_effort
                min_value = -self.maximum_joint_effort
            elif obs_name == "base_angular_vel_x":
                max_value = self.maximum_base_angular_velocity
                min_value = -self.maximum_base_angular_velocity
            elif obs_name == "base_angular_vel_y":
                max_value = self.maximum_base_angular_velocity
                min_value = -self.maximum_base_angular_velocity
            elif obs_name == "base_angular_vel_z":
                max_value = self.maximum_base_angular_velocity
                min_value = -self.maximum_base_angular_velocity

            elif obs_name == "base_linear_acceleration_x":
                max_value = self.maximum_base_linear_acceleration
                min_value = -self.maximum_base_linear_acceleration
            elif obs_name == "base_linear_acceleration_y":
                max_value = self.maximum_base_linear_acceleration
                min_value = -self.maximum_base_linear_acceleration
            elif obs_name == "base_linear_acceleration_z":
                max_value = self.maximum_base_linear_acceleration
                min_value = -self.maximum_base_linear_acceleration
            elif obs_name == "touching_ground":
                max_value = 1
                min_value = 0

            else:
                raise NameError('Observation Asked does not exist=='+str(obs_name))

            self._obs_range_dict[obs_name] = [min_value,max_value]

    def create_bins(self):
        """
        We create the Bins for the discretization of the observations
        self.desired_world_point = Vector3(0.0, 0.0, 0.0)
        self._min_height = min_height
        self._max_height = max_height
        self._abs_max_roll = abs_max_roll
        self._abs_max_pitch = abs_max_pitch
        self._joint_increment_value = joint_increment_value
        self._done_reward = done_reward
        self._alive_reward = alive_reward
        self._desired_force = desired_force
        self._desired_yaw = desired_yaw


        :return:bins
        """

        number_of_observations = len(self._list_of_observations)
        parts_we_disrcetize = self._discrete_division
        rospy.logdebug("Parts to discretise==>"+str(parts_we_disrcetize))
        self._bins = numpy.zeros((number_of_observations, parts_we_disrcetize))
        for counter in range(number_of_observations):
            obs_name = self._list_of_observations[counter]
            min_value = self._obs_range_dict[obs_name][0]
            max_value = self._obs_range_dict[obs_name][1]
            # if obs_name == "touching_ground":
            #     self._bins[counter] = numpy.linspace(min_value, max_value, 2)
            # else:
            self._bins[counter] = numpy.linspace(min_value, max_value, parts_we_disrcetize)

            rospy.logdebug("bins==>" + str(self._bins[counter]))

    def init_joints_pose(self, des_init_pos):
        """
        We initialise the Position variable that saves the desired position where we want our
        joints to be
        :param init_pos:
        :return:
        """
        self.current_joint_pose =[]
        self.current_joint_pose = copy.deepcopy(des_init_pos)
        return self.current_joint_pose

    def get_action_to_position(self, action):
        """
        Here we have the ACtions number to real joint movement correspondance.
        :param action: Integer that goes from 0 to 6, because we have 7 actions.
        :return:
        """

        rospy.logdebug("current joint pose>>>"+str(self.current_joint_pose))
        rospy.logdebug("Action Number>>>"+str(action))

# COXA
        if action == 0: #Increment coxa_l1
            rospy.logdebug("Action Decided:Increment coxa_l1_joint>>>")
            self.current_joint_pose[0] += self._joint_increment_value
        elif action == 1: #Decrement coxa_l1
            rospy.logdebug("Action Decided:Decrement coxa_l1_joint>>>")
            self.current_joint_pose[0] -= self._joint_increment_value
        elif action == 2: #Increment coxa_l2
            rospy.logdebug("Action Decided:Increment coxa_l2_joint>>>")
            self.current_joint_pose[1] += self._joint_increment_value
        elif action == 3: #Decrement coxa_l2
            rospy.logdebug("Action Decided:Decrement coxa_l2_joint>>>")
            self.current_joint_pose[1] -= self._joint_increment_value
        elif action == 4: #Increment coxa_l3
            rospy.logdebug("Action Decided:Increment coxa_l3_joint>>>")
            self.current_joint_pose[2] += self._joint_increment_value
        elif action == 5: #Decrement coxa_l3
            rospy.logdebug("Action Decided:Decrement coxa_l3_joint>>>")
            self.current_joint_pose[2] -= self._joint_increment_value
        elif action == 6: #Increment coxa_r1
            rospy.logdebug("Action Decided:Increment coxa_r1_joint>>>")
            self.current_joint_pose[3] += self._joint_increment_value
        elif action == 7: #Decrement coxa_r1
            rospy.logdebug("Action Decided:Decrement coxa_r1_joint>>>")
            self.current_joint_pose[3] -= self._joint_increment_value
        elif action == 8: #Increment coxa_r2
            rospy.logdebug("Action Decided:Increment coxa_r2_joint>>>")
            self.current_joint_pose[4] += self._joint_increment_value
        elif action == 9: #Decrement coxa_r2
            rospy.logdebug("Action Decided:Decrement coxa_r2_joint>>>")
            self.current_joint_pose[4] -= self._joint_increment_value
        elif action == 10: #Decrement coxa_r3
            rospy.logdebug("Action Decided:Decrement coxa_r3_joint>>>")
            self.current_joint_pose[5] += self._joint_increment_value
        elif action == 11: #Decrement coxa_r3
            rospy.logdebug("Action Decided:Decrement coxa_r3_joint>>>")
            self.current_joint_pose[5] -= self._joint_increment_value

# TIBIA
        if action == 12: #Increment tibia_l1
            rospy.logdebug("Action Decided:Increment haa_joint>>>")
            self.current_joint_pose[6] += self._joint_increment_value
        elif action == 13: #Decrement tibia_l1
            rospy.logdebug("Action Decided:Decrement haa_joint>>>")
            self.current_joint_pose[6] -= self._joint_increment_value
        elif action == 14: #Increment tibia_l2
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[7] += self._joint_increment_value
        elif action == 15: #Decrement tibia_l2
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[7] -= self._joint_increment_value
        elif action == 16: #Increment tibia_l3
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[8] += self._joint_increment_value
        elif action == 17: #Decrement tibia_l3
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[8] -= self._joint_increment_value
        elif action == 18: #Increment tibia_r1
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[9] += self._joint_increment_value
        elif action == 19: #Decrement tibia_r1
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[9] -= self._joint_increment_value
        elif action == 20: #Increment tibia_r2
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[10] += self._joint_increment_value
        elif action == 21: #Decrement tibia_r2
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[10] -= self._joint_increment_value
        elif action == 22: #Decrement tibia_r3
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[11] += self._joint_increment_value
        elif action == 23: #Decrement tibia_r3
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[11] -= self._joint_increment_value

# FEMUR
        if action == 24: #Increment femur_l1
            rospy.logdebug("Action Decided:Increment haa_joint>>>")
            self.current_joint_pose[12] += self._joint_increment_value
        elif action == 25: #Decrement femur_l1
            rospy.logdebug("Action Decided:Decrement haa_joint>>>")
            self.current_joint_pose[12] -= self._joint_increment_value
        elif action == 26: #Increment femur_l2
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[13] += self._joint_increment_value
        elif action == 27: #Decrement femur_l2
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[13] -= self._joint_increment_value
        elif action == 28: #Increment femur_l3
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[14] += self._joint_increment_value
        elif action == 29: #Decrement femur_l3
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[14] -= self._joint_increment_value
        elif action == 30: #Increment femur_r1
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[15] += self._joint_increment_value
        elif action == 31: #Decrement femur_r1
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[15] -= self._joint_increment_value
        elif action == 32: #Increment femur_r2
            rospy.logdebug("Action Decided:Increment hfe_joint>>>")
            self.current_joint_pose[16] += self._joint_increment_value
        elif action == 33: #Decrement femur_r2
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[16] -= self._joint_increment_value
        elif action == 34: #Decrement femur_r3
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[17] += self._joint_increment_value
        elif action == 35: #Decrement femur_r3
            rospy.logdebug("Action Decided:Decrement hfe_joint>>>")
            self.current_joint_pose[17] -= self._joint_increment_value


        # elif action == 36:  # Dont Move
        #     rospy.logdebug("Action Decided:Dont Move>>>")

        rospy.logdebug("action to move joint states>>>" + str(self.current_joint_pose))

        self.clamp_to_joint_limits()

        return self.current_joint_pose

    def clamp_to_joint_limits(self):
        """
        clamps self.current_joint_pose based on the joint limits
        self._joint_limits
        {"haa_max": haa_max,
         "haa_min": haa_min,
         "hfe_max": hfe_max,
         "hfe_min": hfe_min,
         "kfe_max": kfe_max,
         "kfe_min": kfe_min
         }
        :return:
        """

        rospy.logdebug("Clamping current_joint_pose>>>" + str(self.current_joint_pose))
        for i, joint_value in enumerate(self.current_joint_pose):
            self.current_joint_pose[i] = max(min(joint_value, self._joint_limits["max"]),
                                         self._joint_limits["min"])

        rospy.logdebug("DONE Clamping current_joint_pose>>>" + str(self.current_joint_pose))

    def is_joints_less_exceeded(self):
        for pos in self.get_joint_states().position:
            if pos < self._joint_limits["min"] or pos > self._joint_limits["max"]:
                return True
        return False


    def process_data(self):
        """
        We return the total reward based on the state in which we are in and if its done or not
        ( it fell basically )
        :return: reward, done
        """

        # if "hexapod_minimum_height" in self._episode_done_criteria:
        #     hexapod_height_ok = self.hexapod_height_ok()
        # else:
        #     rospy.logdebug("hexapod_height_ok NOT TAKEN INTO ACCOUNT")
        #     hexapod_height_ok = True

        if "hexapod_vertical_orientation" in self._episode_done_criteria:
            hexapod_orientation_ok = self.hexapod_orientation_ok()
        else:
            rospy.logdebug("hexapod_orientation_ok NOT TAKEN INTO ACCOUNT")
            hexapod_orientation_ok = True
        
        if "less_exceeded_joint_position" in self._episode_done_criteria:
            less_exceeded_joint_position = self.is_joints_less_exceeded()
        else:
            rospy.logdebug("exceeded_joint_position NOT TAKEN INTO ACCOUNT")
            less_exceeded_joint_position = False

        is_standing_up = False
        if "stand_up" in self._episode_done_criteria:
            is_standing_up = self.is_stand_up()

        # rospy.logdebug("hexapod_height_ok="+str(hexapod_height_ok))
        rospy.logdebug("hexapod_orientation_ok=" + str(hexapod_orientation_ok))

        done = False
        if (not hexapod_orientation_ok) or less_exceeded_joint_position or is_standing_up:
            done = True
        
        if done:
            if is_standing_up:
                # TODO add to config done reward
                total_reward = 100000000000
            else:
                rospy.logerr("It fell, so the reward has to be very low")
                total_reward = -100000000000
        else:
            rospy.logdebug("Calculate normal reward because it didn't fall.")
            total_reward = self.calculate_total_reward()

        return total_reward, done

