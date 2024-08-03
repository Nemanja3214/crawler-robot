#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def talker():
    parts = ["tibia", "coxa", "femur"]
    sides = ["l", "r"]
    nums = [1, 2, 3]
    rospy.init_node('dummy', anonymous=True)
    rate = rospy.Rate(50)
    publishers = []
    for part in parts:
        for side in sides:
            for num in nums:
                name = make_name(part, side, num)
                print(name)
                publishers.append(rospy.Publisher(name, Float64, queue_size=10, latch=True))
                pos = 0
                if part == parts[0]:
                    pos = 1.4
                elif part == parts[1]:
                    pos = 0
                elif part == parts[2]:
                    pos = -1.4
                publishers[-1].publish(pos)
                rate.sleep()

def make_name(part, side, num):
    return "/hexapod/" + part + "_joint_" + side + str(num) +"_position_controller/command"

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass