#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# define the robot group names created with the MoveIt! Setup Assistant
ARM_GROUP_NAME = 'arm'

class MoveitInterface(object):

    def __init__(self):
        super(MoveitInterface, self).__init__()
        
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
                                                    

    def get_robot_info(self):
        # Function that prints the robot informations obtained via
        # MoveIt! APIs
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""
    

    def go_to_joint_state(self, joint_goal):

        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

    
    def set_max_velocity(self, scaling_factor):
        # This function sets the the max velocity of the ARM and GRIPPER move group as percentage
        # of the max speed of the joint defined in the joint_linits.yaml file
        # it requires a float number as input >= 0 and <= 1
        self.group.set_max_velocity_scaling_factor(scaling_factor)


    def set_max_acceleration(self, scaling_factor):
        # This function sets the the max acceleration of the ARM and GRIPPER move group as percentage
        # of the max speed of the joint defined in the joint_linits.yaml file
        # it requires a float number as input >= 0 and <= 1
        self.group.set_max_acceleration_scaling_factor(scaling_factor)

