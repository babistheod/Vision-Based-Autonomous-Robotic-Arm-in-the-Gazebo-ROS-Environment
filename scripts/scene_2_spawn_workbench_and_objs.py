#!/usr/bin/env python3

import os
import random
import traceback
import itertools

import rospy

import rospkg

from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Point, Quaternion

from pkg_utilts import spawn_model, delete_model

from ur5_pick_place_gazebo.srv import Scene2ObjSpawner, Scene2ObjSpawnerRequest, Scene2ObjSpawnerResponse 

rp = rospkg.RosPack()

urdf_srv = '/gazebo/spawn_urdf_model'
sdf_srv = '/gazebo/spawn_sdf_model'

workbench_name = "workbench_scattered_object"
workbench_urdf_path = os.path.join(rp.get_path("ur5_pick_place_gazebo"), "urdf", "workbench.urdf")
workbench_pose = Pose(position= Point(0.6,     0,      0.0051), orientation=Quaternion(0,0,0,1))

model_name_prefix = "scene_2_"
urdf_file_path_fmt = os.path.join(
    rp.get_path("ur5_pick_place_gazebo"), 
    "urdf",
    "cylinder{}.urdf"
)

urdf_file_paths = [
    urdf_file_path_fmt.format(id) for id in range(1, 17)
]

urdf_model_names = [
    model_name_prefix+obj_path.split("/")[-1].split(".")[0] for obj_path in urdf_file_paths
]


# possible four places where the new marker will be spawn
# here a dict has int keys
obj_poses = {
   1 : Pose(position= Point(0.5,  0,      0.0251),orientation=Quaternion(0,0,0,1)),
   2 : Pose(position= Point(0.7,  0,      0.0251),orientation=Quaternion(0,0,0,1)),
   3 : Pose(position= Point(0.6,  -0.1,   0.0251),orientation=Quaternion(0,0,0,1)),
   4 : Pose(position= Point(0.6,  0.1,    0.0251),orientation=Quaternion(0,0,0,1)),
   5 : Pose(position= Point(0.74, -0.14,   0.0251),orientation=Quaternion(0,0,0,1)),
   6 : Pose(position= Point(0.74,  0.14,  0.0251),orientation=Quaternion(0,0,0,1)),
   7 : Pose(position= Point(0.46, -0.14,  0.0251),orientation=Quaternion(0,0,0,1)),
   8 : Pose(position= Point(0.46,  0.14,  0.0251),orientation=Quaternion(0,0,0,1)),
}

obj_poses_keys = [1, 2, 3, 4, 5, 6, 7, 8]
# randomise the order
obj_poses_keys_itr = itertools.cycle(obj_poses_keys)



def spawn_scene_2_obj_cb(req):
    try:
        urdf_file_paths = urdf_file_path_fmt.format(req.markerID)
        urdf_model_names = model_name_prefix + urdf_file_paths.split("/")[-1].split(".")[0]

        lucky_pose = next(obj_poses_keys_itr) #random.choice(obj_poses_keys)
        urdf_pose = obj_poses.get(lucky_pose)

        print("spawning \"" , urdf_file_paths , "\" at" )
        print(urdf_pose)

        delete_model(
            urdf_model_names,
        )

        spawn_model(
            urdf_srv,
            urdf_model_names, 
            urdf_file_paths, 
            urdf_pose,
        )

        return Scene2ObjSpawnerResponse(success=True)

    except Exception as ex:
        rospy.logerr("{}".format(ex.args))
        traceback.print_exc()

        return Scene2ObjSpawnerResponse(success=False)



def spawn_scene_2_cb(req):
    spawn_model(
        urdf_srv,
        workbench_name, 
        workbench_urdf_path, 
        workbench_pose,
    )

    return EmptyResponse()


def main():
    # setting scene demo stack workbench and camera

    rospy.init_node("spawn_world_components_node", anonymous=True)

    tmp = [delete_model(name) for name in urdf_model_names]


    ss2w = rospy.Service('/spawn_scene_2_workbench', Empty, spawn_scene_2_cb)
    
    ss2c = rospy.Service('/spawn_scene_2_cylinder', Scene2ObjSpawner, spawn_scene_2_obj_cb)
                          

    print("Spawn Gazebo World Components for Scene 2 Service Ready ...")

    rospy.spin()





if __name__ == "__main__":
    main()