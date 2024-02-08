#!/usr/bin/env python3

import os
import rospy

import rospkg

from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from pkg_utilts import spawn_model

rp = rospkg.RosPack()

urdf_srv_name = '/gazebo/spawn_urdf_model'
sdf_srv_name = '/gazebo/spawn_sdf_model'

wb_name = "workbench_demo"
camera_name = "camera_demo"


def spawn_scene_1_cb(req):
    # setting scene demo stack workbench and camera
    workbench_urdf_path = os.path.join(rp.get_path("ur5_pick_place_gazebo"), "urdf", "workbench.urdf")
    spawn_model(
        urdf_srv_name,
        wb_name, 
        workbench_urdf_path, 
        Pose(
            position=Point(-1, 0.25, 0),
            orientation=Quaternion(0, 0, 0, 1)
        )
    )

    camera_sdf_name = os.path.join(rp.get_path("ur5_pick_place_gazebo"), "sdf", "camera.sdf")
    spawn_model(
        sdf_srv_name,
        camera_name, 
        camera_sdf_name, 
        Pose(
            position=Point(-1, 0.25, 0.625),
            orientation=Quaternion(0.5, -0.5, -0.5, -0.5),
        )
    )

    return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("spawn_world_components_node", anonymous=True)

    s = rospy.Service('/spawn_scene_1', Empty, spawn_scene_1_cb)
    print("Spawn Gazebo World Components Service Ready ...")


    rospy.spin()
