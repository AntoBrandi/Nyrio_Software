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


    def plan_cartesian_path(self, points):

        waypoints = []

        for point in points:
            wpose = self.group.get_current_pose().pose
            wpose.position.x += point.position.x
            wpose.position.y += point.position.y  
            wpose.position.z += point.position.z
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def display_trajectory(self, plan):

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    
    def execute_plan(self, plan):

        self.group.execute(plan, wait=True)


    def go_to_pose_goal(self):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        self.group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()
        # It is always good to clear your targets after planning with poses.
        self.group.clear_pose_targets()


