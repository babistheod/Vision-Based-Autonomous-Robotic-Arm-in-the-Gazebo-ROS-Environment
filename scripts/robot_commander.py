#!/usr/bin/env python3


import sys
import numpy as np
import copy 
import traceback

import asyncio

import pdb

import math3d

import rospy
import moveit_commander


from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Pose2D, Pose, PoseStamped, Point, Quaternion

from ur5_pick_place_gazebo.srv import RobotCmd, RobotCmdRequest, RobotCmdResponse

from pkg_utilts import RobotCommanderCmd, RobotCommanderStatus, to_m3d_transform, to_geometry_msgs_pose



class GripperCommander:
    def __init__(self, num_grippers=9):# num_grippers=9
        self.num_grippers = num_grippers
        self.grippers = list(range(num_grippers))

    async def actor(self, cmd, num):
        try:
            srv_name = '/ur5/vacuum_gripper{}/{}'.format(num, cmd)
            print(srv_name)
            rospy.wait_for_service(srv_name, timeout=2)
            switch = rospy.ServiceProxy(srv_name, Empty)
            res = switch()
            print(res)
        except Exception as ex:
            print(ex.args)
            traceback.print_exc()


    # async def turn_on(self, num):
    #     self.actor("on", str(num))

    # async def turn_off(self, num):
    #     self.actor("off", str(num))

    async def turn_on_srv_caller(self, num_grippers=9):
        grippers = list(range(num_grippers))
        on_list = num_grippers * ["on"] # make a list with num_grippers times "on"
        await asyncio.gather(*map(self.actor, on_list, grippers))

    async def turn_off_srv_caller(self, num_grippers=9):
        grippers = list(range(num_grippers))
        off_list = num_grippers * ["off"] # make a list with num_grippers times "on"
        await asyncio.gather(*map(self.actor, off_list, grippers))

    def turn_all_grippers_on(self):
        try:
            result = asyncio.run(self.turn_on_srv_caller())
            #print("result: ", result)
            return True
        except Exception as ex:
            print(ex.args)
            traceback.print_exc()
            return False

    def turn_all_grippers_off(self):
        try:
            result = asyncio.run(self.turn_off_srv_caller())
            #print("result: ", result)
            return True
        except Exception as ex:
            print(ex.args)
            traceback.print_exc()
            return False




class RobotCommander(GripperCommander):
    def __init__(self):
        super(GripperCommander, self).__init__()

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        self.arm.set_pose_reference_frame("base_link")

        self.default_planning_frame = self.arm.get_planning_frame()
        self.default_ref_frame = self.arm.get_pose_reference_frame()
        self.default_ee_frame = self.arm.get_end_effector_link()

        rospy.loginfo("self.default_planning_frame: " + self.default_planning_frame)
        rospy.loginfo("self.default_ref_frame: " + self.default_ref_frame)
        rospy.loginfo("self.default_ee_frame: " + self.default_ee_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        
        # self.arm.set_planning_time(10)
        self.default_max_acceleration_scaling_factor = 0.1
        self.default_max_velocity_scaling_factor = 0.1
        
        self.tracking_max_acceleration_scaling_factor = 0.051
        self.tracking_max_velocity_scaling_factor = 0.051

        self.arm.set_max_acceleration_scaling_factor(self.default_max_velocity_scaling_factor)
        self.arm.set_max_velocity_scaling_factor(self.default_max_velocity_scaling_factor)

        self.arm.clear_path_constraints()



        self.home_pose = Pose()
        self.home_pose.position = Point(x=0.540000000, y=0.00000000, z=0.21)
        self.home_pose.orientation = Quaternion(x = 0.0, y=0.7070, z=0.0, w=0.707)

        self.z_pick = -0.05#-0.065#0525


        try:
            cam_pose = self.arm.get_current_pose(end_effector_link = "camera_link_optical" )
            ee_pose = self.arm.get_current_pose(end_effector_link = "ee_link" )
            
            print("cam_pose: ", cam_pose)
            print("ee_pose: ", ee_pose)

            self.arm.clear_pose_targets()

            self.go_home(step=2)

        except Exception as ex:
            rospy.loginfo(ex.args)
            traceback.print_exc()

        self.robot_cmd_srv = rospy.Service('/robot_cmd', RobotCmd, self.robot_cmd_cb)
        rospy.loginfo("/robot_cmd srv is ready ...")

        # self.track_obj = rospy.Subscriber('asdasdasdasdas', Pose2D, self.track_obj_cb, queue_size=1)
        
        # self.commanding_timer = rospy.Timer(rospy.Rate(10), self.alignment_commander)



    def robot_cmd_cb(self, req):
        # ALIGN
        # PICK
        # PLACE
        # HOME
        try:
            if req.cmd == RobotCommanderCmd.ALIGN:
                pass

            elif req.cmd == RobotCommanderCmd.PICK:
                # bring the pose from camera to robot frame
                cam_in_base_pose = self.arm.get_current_pose(end_effector_link = "camera_link_optical" )
                #print("-------------cam in base pose------------:\n", cam_in_base_pose.pose)
                cam_in_base_m3d = to_m3d_transform(cam_in_base_pose.pose)
                #print("-------------cam in base m3D------------:\n", cam_in_base_m3d)
                #print("-------------marker in cam pose------------:\n", req.reqPose)
                marker_in_cam_m3d = to_m3d_transform(req.reqPose)
                #print("-------------marker in cam m3d------------:\n", marker_in_cam_m3d)
                marker_in_base_m3d = cam_in_base_m3d * marker_in_cam_m3d
                #print("-------------cam in base--marker in cam-- marker in base m3d------------:\n",cam_in_base_m3d, marker_in_cam_m3d, marker_in_base_m3d)
                marker_in_base_pose = to_geometry_msgs_pose(marker_in_base_m3d)

                self.arm.set_max_acceleration_scaling_factor(self.default_max_velocity_scaling_factor)
                self.arm.set_max_velocity_scaling_factor(self.default_max_velocity_scaling_factor)
                print("-------------pickpose-------------:\n", marker_in_base_pose)
                return self.pick_obj(marker_in_base_pose)

                # return RobotCommanderStatus.PICKED

            elif req.cmd == RobotCommanderCmd.PLACE:   
                # bring the pose from camera to robot frame
                # cam_in_base_pose = self.arm.get_current_pose(end_effector_link = "camera_link_optical" )
                # cam_in_base_m3d = to_m3d_transform(cam_in_base_pose.pose)
                # marker_in_cam_m3d = to_m3d_transform(req.reqPose)

                # marker_in_base_m3d = cam_in_base_m3d * marker_in_cam_m3d

                # marker_in_base_pose = to_geometry_msgs_pose(marker_in_base_m3d)

                self.arm.set_max_acceleration_scaling_factor(self.default_max_velocity_scaling_factor)
                self.arm.set_max_velocity_scaling_factor(self.default_max_velocity_scaling_factor)

                return self.place_obj(req.reqPose)
                # return self.place_obj(marker_in_base_pose)

                # return RobotCommanderStatus.PLACED

            elif req.cmd == RobotCommanderCmd.HOME:                
                self.arm.set_max_acceleration_scaling_factor(self.default_max_velocity_scaling_factor)
                self.arm.set_max_velocity_scaling_factor(self.default_max_velocity_scaling_factor)

                return self.go_home(step=2)

                # return RobotCommanderStatus.AT_HOME

        except Exception as ex:
                print(ex.args)
                traceback.print_exc()

                return RobotCommanderStatus.FAILED


    def go_home(self, step=1):
        try:
            # bring robot to home position
            print("going to home")

            if step == 2:
                self.arm.go(joints=[
                    0.0,
                    -1.57,
                    2.356125,
                    -2.356125,
                    -1.57,
                    0.0
                ])


            plan, fraction = self.arm.compute_cartesian_path(
                [self.home_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan)

            return RobotCommanderStatus.AT_HOME

        except Exception as ex:
            print(ex.args)
            traceback.print_exc()

            return RobotCommanderStatus.FAILED

    
    def pick_obj(self, reqPose):
        try:
            curr_ee_pose_stamped = self.arm.get_current_pose(end_effector_link = "ee_link" )
            
            # define pick pose
            # use the current orientation
            pick_pose = copy.deepcopy(reqPose)
            pick_pose.orientation = curr_ee_pose_stamped.pose.orientation
            pick_pose.position.z = self.z_pick
            # define pre pick pose
            pre_pick_pose = copy.deepcopy(pick_pose)
            pre_pick_pose.position.z += 0.05


            # NOW EXECUTE THE PLANNED MOTION

            # pre pick pose
            plan, fraction = self.arm.compute_cartesian_path(
                [pre_pick_pose], # [curr_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan, wait = True)

            # pick pose
            plan, fraction = self.arm.compute_cartesian_path(
                [pick_pose], # [curr_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan, wait = True)

            # turn the gripper on
            is_picked = self.turn_all_grippers_on()

            # post pick pose
            plan, fraction = self.arm.compute_cartesian_path(
                [pre_pick_pose], # [curr_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan, wait = True)

            return RobotCommanderStatus.PICKED

        except Exception as ex:
            print(ex.args)
            traceback.print_exc()
            return RobotCommanderStatus.FAILED


    def place_obj(self, reqPose):
        try:
            curr_ee_pose_stamped = self.arm.get_current_pose(end_effector_link = "ee_link" )
            
            # define pick pose
            # use the current orientation
            place_pose = copy.deepcopy(reqPose)
            place_pose.orientation = curr_ee_pose_stamped.pose.orientation

            # define pre pick pose
            pre_place_pose = copy.deepcopy(place_pose)
            pre_place_pose.position.z += 0.05


            # NOW EXECUTE THE PLANNED MOTION

            # pre pick pose
            plan, fraction = self.arm.compute_cartesian_path(
                [pre_place_pose], # [curr_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan, wait = True)

            # pick pose
            plan, fraction = self.arm.compute_cartesian_path(
                [place_pose], # [curr_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan, wait = True)

            # turn the gripper on
            is_picked = self.turn_all_grippers_off()

            # post pick pose
            plan, fraction = self.arm.compute_cartesian_path(
                [pre_place_pose], # [curr_pose], 
                0.02, 
                0.0, 
                True
            )
            #print(plan)
            self.arm.execute(plan, wait = True)

            return RobotCommanderStatus.PLACED

        except Exception as ex:
            print(ex.args)
            traceback.print_exc()
            return RobotCommanderStatus.FAILED



if __name__ == "__main__":
    rospy.init_node("robot_commander", anonymous=False)

    rc = RobotCommander()

    rospy.spin()

