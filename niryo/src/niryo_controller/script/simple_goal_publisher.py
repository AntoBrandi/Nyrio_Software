#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray
    

if __name__ == '__main__':
    rospy.init_node('simple_goal_publisher', anonymous=True)
    pub = rospy.Publisher('niryo/stepper_actuate', UInt16MultiArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(data=[0,0,0])
        rate.sleep()
