import rospy
from niryo_msgs.msg import NiryoTaskAction, NiryoTaskGoal, NiryoGoal
from geometry_msgs.msg import Pose
import actionlib

class Task():

    # static class member
    _client = actionlib.SimpleActionClient('task_server', NiryoTaskAction)

    def __init__(self):
        # Proteted class member    
        self._client.wait_for_server()   
        self._goal = NiryoGoal()
    
    def execute(self):
        self._client.send_goal(NiryoTaskGoal(goal = self._goal))


class Home(Task):

    def __init__(self):
        Task.__init__(self)

    def execute(self):
        self._goal.isCartesian = False
        self._goal.isAbsolutePose = True
        self._goal.isDisplay = False
        self._goal.isExecute = True
        self._goal.jointAngles = [0.0,0.0,0.0,0.0,0.0,0.0]
        Task.execute(self)
        

class Sleep(Task):

    def __init__(self):
        Task.__init__(self)
        
    def execute(self):
        self._goal.isCartesian = False
        self._goal.isAbsolutePose = True
        self._goal.isDisplay = False
        self._goal.isExecute = True
        self._goal.jointAngles = [0.0,0.7065,-1.3345,0.0,0.0,0.0]
        Task.execute(self)


class MoveUp(Task):

    def __init__(self):
        Task.__init__(self)
        
    def execute(self):
        self._goal.isCartesian = True
        self._goal.isAbsolutePose = False
        self._goal.isDisplay = True
        self._goal.isExecute = True
        pose = Pose()
        pose.position.z = 0.1 # meters
        self._goal.waypoints.append(pose)
        Task.execute(self)
