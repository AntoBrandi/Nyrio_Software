#! /usr/bin/env python
import rospy
import actionlib
from niryo_msgs.msg import NiryoTaskAction, NiryoTaskGoal, NiryoTaskFeedback, NiryoTaskResult
from moveit_interface import MoveitInterface


class TaskServer(object):
    # create messages that are used to publish feedback/result
    _feedback = NiryoTaskFeedback()
    _result = NiryoTaskResult()
    _moveit = MoveitInterface()

    def __init__(self, name):
        # Constructor
        # function that inizialize the NiryoTaskAction class and creates 
        # a Simple Action Server from the library actionlib
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, NiryoTaskAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      

    def execute_cb(self, goal):

        success = True     

        # start executing the action
        if goal.goal.isCartesian:
            pass
        else:
            self._moveit.go_to_joint_state(goal.goal.jointAngles)


        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

          
        # If no cancelation requests are received, return a suceeded result 
        if success:
            self._result.success = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':

    # Inizialize a ROS node 
    rospy.init_node('task_server')

    server = TaskServer(rospy.get_name())

    # keeps the node up and running
    rospy.spin()
