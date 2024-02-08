import sys

import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion

moveit_commander.roscpp_initialize(sys.argv)


arm = moveit_commander.MoveGroupCommander('manipulator')

arm.set_pose_reference_frame("base_link")

default_planning_frame = arm.get_planning_frame()
default_ref_frame = arm.get_pose_reference_frame()
default_ee_frame = arm.get_end_effector_link()
rospy.loginfo("default_planning_frame: " + default_planning_frame)
rospy.loginfo("default_ref_frame: " + default_ref_frame)
rospy.loginfo("default_ee_frame: " + default_ee_frame)

poses = []
for ii in range(0, 3):
    pose = Pose(
        position=Point(x=0.12, y=-(0.35 + ii*0.14), z=-0.0525), 
        orientation=Quaternion(x = 0.0, y=0.7070, z=0.0, w=0.707)
    )

    print("pose:\n", pose)

    poses.append(pose)


plan, fraction = arm.compute_cartesian_path(
    poses, 
    0.02, 
    0.0, 
    True
)
print(plan)
arm.execute(plan)