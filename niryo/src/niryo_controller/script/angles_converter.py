#!/usr/bin/env python
import rospy
import math
from niryo_controller.srv import AnglesConverter, AnglesConverterResponse

def convert_radians_to_steps(req):
    # Function that is called every time the service radians_to_steps is called
    # It receives the Request message as input with the angles in radians
    # and returns the Result message as output with the angles in steps
    res = AnglesConverterResponse()
    res.joint_1 = (req.joint_1 * 600)/math.pi
    res.joint_2 = (req.joint_2 * 160)/0.628
    res.joint_3 = (((math.pi/2)-req.joint_3)*720)/2.826
    res.joint_4 = 0
    res.joint_5 = 0
    res.joint_6 = 0
    return res

def convert_steps_to_radians(req):
    # Function that is called every time the service steps_to_radians is called
    # It receives the Request message as input with the angles in steps
    # and returns the Result message as output with the angles in radians
    res = AnglesConverterResponse()
    res.joint_1 = (req.joint_1 * math.pi)/600
    res.joint_2 = (req.joint_2 * 0.628)/160
    res.joint_3 = (math.pi/2)-((2.826*req.joint_3)/720)
    res.joint_4 = 0
    res.joint_5 = 0
    res.joint_6 = 0
    return res


if __name__ == "__main__":
    # Inizialize a ROS node called angles_converter
    rospy.init_node('angles_converter')

    # Inizialize two services for the angle conversions 
    radians_to_steps = rospy.Service('radians_to_steps', AnglesConverter, convert_radians_to_steps)
    steps_to_radians = rospy.Service('steps_to_radians', AnglesConverter, convert_steps_to_radians)

    # keeps the node up and running
    rospy.spin()