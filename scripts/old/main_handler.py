#!/usr/bin/env python
import copy

import rospy

from std_srvs.srv import Empty

from geometry_msgs.msg import Pose

from ur5_pick_place_gazebo.srv import TrackObj, TrackObjRequest, TrackObjResponse
from ur5_pick_place_gazebo.srv import RobotCmd, RobotCmdRequest, RobotCmdResponse


from pkg_utilts import RobotCommanderCmd, RobotCommanderStatus

stack_seq = [
    "red",
    "green",
    "yellow",
    "blue",
]

stack_pose = Pose()

disk_height = 0.025 + 0.005 # a bit offset

stack_pose.position.x = 0.25
stack_pose.position.y = -0.25
stack_pose.position.z = -0.0525

stack_pose.orientation.w = 1



# bring the robot to home pose
print("going to home pose ...")

rob_commander_req = RobotCmdRequest()
rob_commander_res = RobotCmdResponse()

rob_commander_res.status = RobotCommanderStatus.FAILED
while not rob_commander_res.status == RobotCommanderStatus.AT_HOME:
    rob_commander_req.cmd = RobotCommanderCmd.HOME
    rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

    rob_commander_res = rob_commander_srv(rob_commander_req)

print("going to home pose ... [DONE]")



# spawn the world's component
spawn_world = rospy.ServiceProxy("/spawn_world_components", Empty)
res_spawn_world = spawn_world()
print("res_spawn_world: ", res_spawn_world)


# loop to place each disk on the on top of each other
for ii, color in enumerate( stack_seq):
    print("initialising task for \"{}\" cylinder".format(color))

    # trigger obj tracker for desired color
    print("initialising tracking for \"{}\" cylinder ...".format(color))
    track_obj_req = TrackObjRequest()
    track_obj_res = TrackObjResponse()
    track_obj_res.success = False

    while not track_obj_res.success:
        track_obj_req.color = color
        track_obj_req.trigger = True

        track_obj_srv = rospy.ServiceProxy("/track_obj", TrackObj)
        track_obj_res = track_obj_srv(track_obj_req)
    
    print("initialising tracking for \"{}\" cylinder ... [DONE]".format(color))



    # track, pick and place task
    rob_commander_req = RobotCmdRequest()
    rob_commander_res = RobotCmdResponse()


    # bring the robot to home pose
    # print("going to home pose ...")
    # rob_commander_res.status = RobotCommanderStatus.FAILED
    # while not rob_commander_res.status == RobotCommanderStatus.AT_HOME:
    #     rob_commander_req.cmd = RobotCommanderCmd.HOME
    #     rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

    #     rob_commander_res = rob_commander_srv(rob_commander_req)
    
    # print("going to home pose ... [DONE]")




    # align the gripper on the top of obj
    print("initialising aligning for \"{}\" cylinder ... ".format(color))

    rob_commander_res.status = RobotCommanderStatus.FAILED
    while not rob_commander_res.status == RobotCommanderStatus.ALIGNED:
        rob_commander_req.cmd = RobotCommanderCmd.ALIGN
        rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

        rob_commander_res = rob_commander_srv(rob_commander_req)
    
    print("initialising aligning for \"{}\" cylinder ... [DONE]".format(color))


    # pick the obj
    print("initialising picking for \"{}\" cylinder ... ".format(color))

    rob_commander_res.status = RobotCommanderStatus.FAILED
    while not rob_commander_res.status == RobotCommanderStatus.PICKED:
        rob_commander_req.cmd = RobotCommanderCmd.PICK
        rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

        rob_commander_res = rob_commander_srv(rob_commander_req)

    print("initialising picking for \"{}\" cylinder ... [DONE]".format(color))
    

    
    # place the obj
    print("initialising placing for \"{}\" cylinder ... ".format(color))


    rob_commander_res.status = RobotCommanderStatus.FAILED
    while not rob_commander_res.status == RobotCommanderStatus.PLACED:
        rob_commander_req.cmd = RobotCommanderCmd.PLACE
        rob_commander_req.reqPose = copy.deepcopy(stack_pose)

         # as stack is growing
        rob_commander_req.reqPose.position.z += (ii * disk_height) 
        
        rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

        rob_commander_res = rob_commander_srv(rob_commander_req)
    
    print("initialising placing for \"{}\" cylinder ... [DONE]".format(color))


    print("DONE")

   
    


