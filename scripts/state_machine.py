#!/usr/bin/env python3

import copy
from sre_constants import SUCCESS

import rospy
import smach
import smach_ros

from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty
from std_msgs.msg import Int8
import random

import itertools

from ur5_pick_place_gazebo.srv import RobotCmd, RobotCmdRequest, RobotCmdResponse
from ur5_pick_place_gazebo.srv import Scene1StackBuilder, Scene1StackBuilderRequest, Scene1StackBuilderResponse
from ur5_pick_place_gazebo.srv import Scene2ObjSpawner, Scene2ObjSpawnerRequest, Scene2ObjSpawnerResponse

from ur5_pick_place_gazebo.srv import EstimatePose, EstimatePoseRequest, EstimatePoseResponse

from ur5_pick_place_gazebo.srv import DetectMarkerIDAndStack, DetectMarkerIDAndStackRequest, DetectMarkerIDAndStackResponse

from pkg_utilts import RobotCommanderCmd, RobotCommanderStatus, to_m3d_transform, to_geometry_msgs_pose


## DEFINE FLOW
# Initial state (IS)                  --> initialise world
# Home State (HS)                     --> bring the robot at home position  
# Detection State 1 (DS1)             --> detect marker in scene 1
# Detection State 2 (DS2)             --> bring the robot above the workbench & detect the pose of detected marker in DS1
# PICK & PLACE STATE (PPS)            --> perform pick and place task 


TOTAL_OBJS = 8
OBJ_KEYS = [ii for ii in range(1, TOTAL_OBJS+1)]
# shuffle the keys in random order
random.shuffle(OBJ_KEYS)
OBJ_KEYS_ITR = itertools.cycle(OBJ_KEYS)

STACK_KEYS= [1, 3, 1, 2, 2, 1, 3, 2]
random.shuffle(STACK_KEYS)  # shuffle the keys in random order
STACK_KEYS_ITR = itertools.cycle(STACK_KEYS)

#STACK_KEYS = [1, 2, 3 ]
#random.shuffle(STACK_KEYS)  # shuffle the keys in random order
#STACK_KEYS_ITR = itertools.cycle(STACK_KEYS)

CYCLE = 0


# stack poses
stack_pose = Pose()

stack_poses = {
    1 : Pose(position=Point(0.12, -0.63, -0.0525), orientation=Quaternion(0,0,0,1)),
    2 : Pose(position=Point(0.12, -0.49, -0.0525), orientation=Quaternion(0,0,0,1)),
    3 : Pose(position=Point(0.12, -0.35, -0.0525), orientation=Quaternion(0,0,0,1)),
}

stack_heights = [0.0, 0.0, 0.0]
stack_inc = 0.0125




class IS(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['initilisation'],
        )


    def spawn_scene_1(self):
        # scene 1
        # rospy.wait_for_service("/spawn_scene_1", timeout=5)
        spawn_scene_1_srv = rospy.ServiceProxy("/spawn_scene_1", Empty)
        # res_spawn_scene_1 = spawn_scene_1_srv()
        spawn_scene_1_srv()
        print("res_spawn_scene_1: ")#, res_spawn_scene_1)


    def spawn_scene_2(self):
        # scene 1
        # rospy.wait_for_service("/spawn_scene_2_workbench", timeout=5)
        spawn_scene_2_srv = rospy.ServiceProxy("/spawn_scene_2_workbench", Empty)
        # res_spawn_scene_2 = spawn_scene_2_srv()
        spawn_scene_2_srv()
        print("res_spawn_scene_2: ")#, res_spawn_scene_2)

    def spawn_obj_scene_2(self, marker_id):
        # scene 1
        # rospy.wait_for_service("/spawn_scene_2_cylinder", timeout=5)
        req = Scene2ObjSpawnerRequest()
        req.markerID = marker_id
    
        res = Scene2ObjSpawnerResponse()
        res.success = False
        while not res.success:
            spawn_obj_scene_2_srv = rospy.ServiceProxy("/spawn_scene_2_cylinder", Scene2ObjSpawner)
            res = spawn_obj_scene_2_srv(req)
            print("res_spawn_obj_scene_2: ", res)

    def execute(self, userdata):
        rospy.loginfo('Executing state --> Initial state (IS)')
        # rospy.sleep(1)

        # spawn the world's component
        # scene 1
        self.spawn_scene_1()

        # scene 2
        self.spawn_scene_2()

        for i in range(1, TOTAL_OBJS+1):
            self.spawn_obj_scene_2(i) 
            rospy.sleep(4)
        # rospy.sleep(2.5)

        rospy.loginfo("press any key to continue...")
        input()

        return 'initilisation'


class HS(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['robot_at_home_pose', 'tasks_finished'], #_init', 'robot_at_home_pose_after_PPS'])
            input_keys=['TOTAL_OBJS'],
            io_keys=['OBJ_KEYS_ITR', 'STACK_KEYS_ITR', 'CYCLE', 'TOTAL_OBJS'],
            # io_keys=["itr"],
        )

    def go_home(self):
        # bring the robot to home pose
        print("going to home pose ...")

        rob_commander_req = RobotCmdRequest()
        rob_commander_res = RobotCmdResponse()

        rob_commander_res.status = RobotCommanderStatus.FAILED
        while not rob_commander_res.status == RobotCommanderStatus.AT_HOME:
            rob_commander_req.cmd = RobotCommanderCmd.HOME
            # rospy.wait_for_service("/robot_cmd", timeout=5)
            rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

            rob_commander_res = rob_commander_srv(rob_commander_req)

        print("going to home pose ... [DONE]")


    def spawn_obj_scene_1(self, stack_id, marker_id):
        # scene 1
        # rospy.wait_for_service("/scene_1_stack_builder", timeout=5)
        req = Scene1StackBuilderRequest()
        req.stackNum = stack_id
        req.markerID = marker_id

        res = Scene1StackBuilderResponse()
        res.success = False

        while not res.success:
            spawn_obj_scene_1_srv = rospy.ServiceProxy("/scene_1_stack_builder", Scene1StackBuilder)
            res = spawn_obj_scene_1_srv(req)
            print("res_spawn_obj_scene_1: ", res)

    # def spawn_obj_scene_2(self, marker_id):
    #     # scene 1
    #     # rospy.wait_for_service("/spawn_scene_2_cylinder", timeout=5)
    #     req = Scene2ObjSpawnerRequest()
    #     req.markerID = marker_id
    
    #     res = Scene2ObjSpawnerResponse()
    #     res.success = False
    #     while not res.success:
    #         spawn_obj_scene_2_srv = rospy.ServiceProxy("/spawn_scene_2_cylinder", Scene2ObjSpawner)
    #         res = spawn_obj_scene_2_srv(req)
    #         print("res_spawn_obj_scene_2: ", res)


    def execute(self, userdata):
        rospy.loginfo('Executing state --> Home State (HS) ... {} itr'.format(userdata.CYCLE))
        rospy.loginfo("press any key to continue...")
        input()
        # rospy.sleep(2.5)
        # home_pose = Pose().deserialize(userdata.home_pose)
        # print("home_pose:\n", userdata.home_pose)
        # rospy.sleep(1)

        # go home
        self.go_home()

        if userdata.CYCLE < userdata.TOTAL_OBJS:
            userdata.CYCLE += 1    

            # spawn cylinder on both scenes
            stack_id = next(userdata.STACK_KEYS_ITR)
            lucky_num = next(userdata.OBJ_KEYS_ITR)

            # spawn in scene 1
            self.spawn_obj_scene_1(stack_id, lucky_num)

            # spawn in scene 1
            #self.spawn_obj_scene_2(lucky_num)

            return 'robot_at_home_pose'
        else:
            return 'tasks_finished'
            
        



class DS1(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['get_marker_in_scene_1'],
            io_keys=['latest_id', 'placing_stack'])

    def execute(self, userdata):
        rospy.loginfo('Executing state --> Detection State 1 (DS1)')
        rospy.loginfo("press any key to continue...")
        input()
        # rospy.sleep(2.5)
        # rospy.sleep(1)

        # get the latest aruco id in the scene
        res = DetectMarkerIDAndStackResponse()
        res.success = False
        res.stackNum = 0
        while not res.success or (res.stackNum == 0):
            req = DetectMarkerIDAndStackRequest()
            srv = rospy.ServiceProxy("/latest_aruco_id", DetectMarkerIDAndStack)
            res = srv(req)

            userdata.latest_id = res.id
            userdata.placing_stack = res.stackNum
            rospy.sleep(0.25)

            rospy.logerr("stuck in the in the loop ...\nres:\n{}".format(res))

        return 'get_marker_in_scene_1'


class DS2(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['estimate_obj_pose_in_scene_2'],
            io_keys=['latest_id', 'pick_pose', 'placing_stack'])

    def execute(self, userdata):
        rospy.loginfo('Executing state --> Detection State 2 (DS2)')
        rospy.loginfo("press any key to continue...")
        input()
        # rospy.sleep(2.5)
        
        # scene 1
        res = EstimatePoseResponse()
        res.success = False
        while not res.success:
            # rospy.wait_for_service("/estimate_obj_pose_scene_2", timeout=5)
            estimate_obj_pose_scene_2_srv = rospy.ServiceProxy("/estimate_obj_pose_scene_2", EstimatePose)
            req = EstimatePoseRequest()
            req.id = userdata.latest_id
            res = estimate_obj_pose_scene_2_srv(req)
            if res.success:
                userdata.pick_pose = res.pose
                # userdata.placing_stack = res.stackNum
                print("res_estimate_obj_pose_scene_2_srv: ", res)

        return 'estimate_obj_pose_in_scene_2'


class PPS(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['obj_picked_and_placed'],
            io_keys=['pick_pose', 'placing_stack', 'stack_poses', 'stack_heights', 'stack_inc'],
            )

    def pick(self, pose, stack):
        # pick the obj
        print("initialising picking for \"{}\" stack ... ".format(stack))
        rob_commander_req = RobotCmdRequest()
        rob_commander_res = RobotCmdResponse()
        rob_commander_res.status = RobotCommanderStatus.FAILED
        while not rob_commander_res.status == RobotCommanderStatus.PICKED:
            rob_commander_req.cmd = RobotCommanderCmd.PICK
            rob_commander_req.reqPose = pose
            # rospy.wait_for_service("/robot_cmd", timeout=5)
            rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

            rob_commander_res = rob_commander_srv(rob_commander_req)

    def place(self, pose, stack):
        # place the obj
        print("initialising placing for \"{}\" stack ... ".format(stack))

        rob_commander_req = RobotCmdRequest()
        rob_commander_res = RobotCmdResponse()
        rob_commander_res.status = RobotCommanderStatus.FAILED

        while not rob_commander_res.status == RobotCommanderStatus.PLACED:
            rob_commander_req.cmd = RobotCommanderCmd.PLACE
            rob_commander_req.reqPose = copy.deepcopy(pose)

            # as stack is growing
            
            # rospy.wait_for_service("/robot_cmd", timeout=5)
            rob_commander_srv = rospy.ServiceProxy("/robot_cmd", RobotCmd)

            rob_commander_res = rob_commander_srv(rob_commander_req)


    def execute(self, userdata):
        rospy.loginfo('Executing state --> PICK & PLACE STATE (PPS)')
        rospy.loginfo("press any key to continue...")
        input()
        # rospy.sleep(2.5)
        # rospy.loginfo("press any key to proceed ...")
        # input()
        # userdata.pick_pose = Pose(position=Point(1,2,3), orientation=Quaternion(0,0,0,1))
        # print("home_pose:\n", userdata.pick_pose)
        # rospy.sleep(1)

        pick_pose = copy.deepcopy(userdata.pick_pose)
        
        # pick
        self.pick(pick_pose, userdata.placing_stack)
        rospy.sleep(0.5)
        # place
        placing_pose = copy.deepcopy(userdata.stack_poses.get(userdata.placing_stack))

        userdata.stack_heights[userdata.placing_stack-1] += userdata.stack_inc
        placing_pose.position.z += userdata.stack_heights[userdata.placing_stack-1]

        userdata.stack_poses[userdata.placing_stack] = placing_pose

        self.place(placing_pose, userdata.placing_stack)

        return 'obj_picked_and_placed'



def main():
    rospy.init_node('smach_example_state_machine')

    # total_itr = 5
    # itr = 0

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    sm.userdata.OBJ_KEYS_ITR = OBJ_KEYS_ITR
    sm.userdata.STACK_KEYS_ITR = STACK_KEYS_ITR
    sm.userdata.CYCLE = CYCLE
    sm.userdata.TOTAL_OBJS = TOTAL_OBJS
    sm.userdata.latest_id = None

    sm.userdata.pick_pose = None
    sm.userdata.placing_stack = None
    sm.userdata.stack_poses = stack_poses
    sm.userdata.stack_heights = stack_heights
    sm.userdata.stack_inc = stack_inc

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            'IS', IS(),
            transitions={
                "initilisation" : "HS",
            },
        )

        smach.StateMachine.add(
            'HS', HS(),
            transitions={
                "robot_at_home_pose"  : "DS1",
                "tasks_finished"      :  "END"
            },
            remapping={
                "OBJ_KEYS_ITR"      :   "OBJ_KEYS_ITR",
                "STACK_KEYS_ITR"    :   "STACK_KEYS_ITR",
                "CYCLE"             :   "CYCLE",
                "TOTAL_OBJS"        :   "TOTAL_OBJS"
            },
        )

        smach.StateMachine.add(
            'DS1', DS1(),
            transitions={
                "get_marker_in_scene_1" : "DS2",
            },
            remapping={
                "latest_id"     :       "latest_id",
                "placing_stack" :       "placing_stack"

            }
        )

        smach.StateMachine.add(
            'DS2', DS2(),
            transitions={
                "estimate_obj_pose_in_scene_2" : "PPS",
            },
            remapping={
                'latest_id'     :       'latest_id',
                'pick_pose'     :       'pick_pose',
                'placing_stack' :       'placing_stack',
            },
        )

        smach.StateMachine.add(
            'PPS', PPS(),
            transitions={
                "obj_picked_and_placed" : "HS",
            },
            remapping={
                'pick_pose'     :       'pick_pose',
                'placing_stack' :       'placing_stack',
                'stack_poses'   :       'stack_poses',
                'stack_heights' :       'stack_heights',
                'stack_inc'     :       'stack_inc',
            }
        )



    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    # sm.set_initial_state(IS)
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()