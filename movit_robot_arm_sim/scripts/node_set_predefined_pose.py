



import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi

class MyRobot:

   
    def __init__(self, Group_Name):
     
        print(f"Initializing robot with group: {Group_Name}")
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)

        
        self._robot = moveit_commander.RobotCommander()
        
        self._scene = moveit_commander.PlanningSceneInterface()
        
        
        self._planning_group = Group_Name
       
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
        
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)


        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        
        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        
        self._group.set_named_target(arg_pose_name)


        success, plan, x, y = self._group.plan()

       
        if not success:
            rospy.logerr("Planning failed for pose: {}".format(arg_pose_name))
            return

       
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()

        
        goal.trajectory = plan  


        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()

        
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    
    def __del__(self):
        
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')


def main():

    
    arm = MyRobot("arm_group")
    hand =  MyRobot("hand")

    
    while not rospy.is_shutdown():
      
        arm.set_pose("zero_pose")
      
        rospy.sleep(2)
        
        arm.set_pose("straight_up")
        rospy.sleep(2)
        
       
        hand.set_pose("hand_open")
        rospy.sleep(1)
        
        arm.set_pose("pick_object_pose")
        rospy.sleep(2)
       
        hand.set_pose("hand_closed")
        rospy.sleep(1)
        
        arm.set_pose("lift_object_pose")
        rospy.sleep(2)
        
        arm.set_pose("place_object_opposit_pose")
        rospy.sleep(2)
        
        
        hand.set_pose("hand_open")
        rospy.sleep(1)
        
        arm.set_pose("opposite_pose")
        rospy.sleep(2)
        
        
        hand.set_pose("hand_closed")
        rospy.sleep(1)


    
    del arm
    del hand
	


if __name__== '__main__':
    main()