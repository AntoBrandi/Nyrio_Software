#! /usr/bin/env python
import rospy
import actionlib
import control_msgs.msg
import math
from std_msgs.msg import Int16MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from niryo_controller.srv import AnglesConverter


# constant list of the joint names for the arm
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3','joint_4', 'joint_5', 'joint_6']

# constant that defines the start pose of the robot
START_POSE = [0.0,0.0,0.0,0.0,0.0,0.0]

# current pose of the robot
CURRENT_POSE = [0.0,0.0,0.0,0.0,0.0,0.0]


class TrajectoryControllerAction(object):
    # create messages that are used to publish feedback/result during the action execution
    # and after its completion
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        # Constructor, called when an instance of this class is created
        # It init and starts an action server with the FollowJointTrajectoryAction interface
        # It sends and executes the start pose of the robot so that the start configuration of 
        # the robot is known
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.goal_cb, auto_start = False)
        self._as.start()
        self.execute(START_POSE)

      
    def goal_cb(self, goal):
        # This function is called when the action server receives a new goal
        # The received goal is a FollowJointTrajectoryActionGoal message and contains
        # the list of position that each joint should follow. Each of this consequent poses of the 
        # joints are separated by a given time interval. 
        success = True
        rospy.loginfo('%s: Goal Received' % self._action_name)
        
        # Loop that executes each pose of the robot in the given goal trajectory
        # delaying each consequent pose by a given delay 
        for i, point in enumerate(goal.trajectory.points):

            # At each execution of the loop, check if the action server received a cancelation request
            # of the ongoing goal. If so, interrupt the execution of the goal and 
            # return a result message with status failed
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            # wait before starting the execution of the next goal
            delay = point.time_from_start - goal.trajectory.points[i-1].time_from_start if i>0 else 0
            rospy.sleep(delay)

            # reach the current loop pose
            self.execute(point.positions)

            # Fill out a FollowJointTrajectoryFeedback message to provide
            # a feedback about the current goal execution
            self._feedback.joint_names = JOINT_NAMES
            self._feedback.actual = point
            self._feedback.desired = point
            self._as.publish_feedback(self._feedback)

        # if during its execution the goal hasn't received any cancelation request,
        # return a result message with status succeeded
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)        
        

    def execute(self, angles):
        # This function publishes the target pose on the robot on the matching topic
        rospy.loginfo('Angles Radians : %s' % str(angles))
        # The trajectory controller is moving a real robot controlled by Arduino
        radians_to_steps = rospy.ServiceProxy('/radians_to_steps', AnglesConverter)
        angles_step = radians_to_steps(angles[0],angles[1],angles[2],angles[3],angles[4],angles[5])
        rospy.loginfo('Angles Steps : %s' % str([angles_step.joint_1,angles_step.joint_2,angles_step.joint_3,angles_step.joint_4,angles_step.joint_5,angles_step.joint_6]))

        pub_stepper.publish(data=[int(angles_step.joint_1), int(angles_step.joint_2), int(angles_step.joint_3)])
        pub_servo.publish(data=[int(angles_step.joint_4), int(angles_step.joint_5), int(angles_step.joint_6)])
        CURRENT_POSE = angles     
        

if __name__ == '__main__':
    # Inizialize a ROS node called trajectory_action
    rospy.init_node('arm_controller')

    pub_stepper = rospy.Publisher('/niryo/stepper_actuate', Int16MultiArray, queue_size=10)
    pub_servo = rospy.Publisher('/niryo/servo_actuate', Int16MultiArray, queue_size=10)
    pub_status = rospy.Publisher('/niryo/joint_states', JointState, queue_size=10)

    rospy.wait_for_service('/radians_to_steps')

    # Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    # execute it in the real robot
    server = TrajectoryControllerAction(rospy.get_name())

    rate = rospy.Rate(10) # 10hz
    status = JointState()
    status.name = JOINT_NAMES

    while not rospy.is_shutdown():  
        status.header.stamp = rospy.Time.now()
        status.position = CURRENT_POSE
        pub_status.publish(status)
        rate.sleep()

    # keep this ROS node up and running
    rospy.spin()
