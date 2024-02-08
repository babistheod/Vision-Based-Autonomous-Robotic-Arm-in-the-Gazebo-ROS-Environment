#!/usr/bin/env python3

import os

import traceback
from random import random

import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

from ur5_pick_place_gazebo.srv import Scene1StackBuilder, Scene1StackBuilderRequest, Scene1StackBuilderResponse


rp = rospkg.RosPack()

urdf_srv_name = '/gazebo/spawn_urdf_model'
sdf_srv_name = '/gazebo/spawn_sdf_model'

wb_name = "workbench_demo"
camera_name = "camera_demo"

model_name_prefix = "scene_1_"

stack_inc = 0.0125
stack_heights = [0.0, 0.0, 0.0]

stack_loc = [
    Pose(position = Point(-1,  0.09,  0.025), orientation = Quaternion(0,0,0,1)),
    Pose(position = Point(-1,  0.25,  0.025), orientation = Quaternion(0,0,0,1)),
    Pose(position = Point(-1,  0.41,  0.025), orientation = Quaternion(0,0,0,1)),
]


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


print("stack_loc\n", stack_loc)
print("urdf_file_path_fmt\n", urdf_file_path_fmt)
print("urdf_file_paths\n", urdf_file_paths)
print("urdf_model_names\n", urdf_model_names)

first_spawn_all = True
first_delete_all = True

# async def spawn_model(name, path, pose=Pose()):
def spawn_model(srv_name, name, path, pose=Pose()):
    try:
        # print(urdf_path)

        # rospy.loginfo("spawning \"" + obj_path + "\" at" )
        print("spawning \"" , path , "\" at" )
        print(pose)

        rospy.wait_for_service(srv_name, timeout=5)
        spawn_model_service = rospy.ServiceProxy(srv_name, SpawnModel)
        spawn_model_service(
            model_name=name,  
            model_xml=open(path, 'r').read(),
            # robot_namespace='/foo',
            initial_pose=pose,
            reference_frame='world',
        )
        return True
    except Exception as ex:
        print(ex.args)
        traceback.print_exc()
        return False




# async def delete_model(name):
def delete_model(name):
    try:
        # rospy.loginfo("spawning \"" + name + "\" at" )
        print("deleting \"" , name , "\"" )

        rospy.wait_for_service('/gazebo/delete_model', timeout=5)
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_res = delete_model_service(
            model_name=name,
        )
        print(delete_model_res)
        return True

    except Exception as ex:
        traceback.print_exc()
        print(ex.args)
        return False


# async def asyn_spawn_all_models():
#     await asyncio.gather(*map(spawn_model, urdf_model_names, urdf_file_paths))


# async def asyn_delete_all_models():
#     await asyncio.gather(*map(delete_model, urdf_model_names))


# async def asyn_spawn_and_delete_for_initial_run():
#     await asyncio.gather(*map(spawn_model, urdf_model_names, urdf_file_paths))
#     await asyncio.gather(*map(delete_model, urdf_model_names))



def stack_builder_func(req):
    # global first_delete_all
    # if first_delete_all:
    #     # asyncio.run(asyn_delete_all_models())
    #     tmp = [delete_model(name) for name in urdf_model_names]

    #     first_delete_all = False
    #     rospy.logerr("deleting all models...")
    #     return Scene1StackBuilderResponse(success=False)



    # error checking
    if req.stackNum < 1 or req.stackNum > 3:
        rospy.logerr("stack number is not within limit ...\tstack number: {}".format(req.stackNum)) 
        return Scene1StackBuilderResponse(success=False)

    
    if req.markerID < 1 or req.markerID >16:
        rospy.logerr("maker id is not within limit ...\marker id: {}".format(req.markerID))
        return Scene1StackBuilderResponse(success=False)


    success = spawn_model(
        urdf_srv_name,
        urdf_model_names[req.markerID-1], 
        urdf_file_paths[req.markerID-1], 
        stack_loc[req.stackNum-1],
    )
    if success:
        stack_heights[req.stackNum-1] += stack_inc
        stack_loc[req.stackNum-1].position.z += stack_heights[req.stackNum-1]

        return Scene1StackBuilderResponse(success=True)
    else: 
        return Scene1StackBuilderResponse(success=False)



if __name__ == "__main__":
    rospy.init_node("scene_1_stack_builder_node", anonymous=True)

    # deleting all previous models in scene 1
    # tmp = [delete_model(name) for name in urdf_model_names]
    # asyncio.run(asyn_delete_all_models())
    # asyncio.run(asyn_spawn_and_delete_for_initial_run())
    # asyncio.run(asyn_spawn_all_models())
    # tmp = [
    #     spawn_model(
    #         name, 
    #         path, 
    #         Pose(
    #             position=Point(random(), random(), random()),
    #             orientation=Quaternion(random(), random(), random(), random())
    #         )
    #     ) 
    #     for name, path in zip(urdf_model_names, urdf_file_paths)
    # ]

    # print("tmp:\n", tmp)

    # for name, path in zip(urdf_model_names, urdf_file_paths):
    #     # asyncio.run(spawn_model(
    #     spawn_model(
    #         name, 
    #         path, 
    #         Pose(
    #             position=Point(random(), random(), random()),
    #             orientation=Quaternion(random(), random(), random(), random())
    #         )
    #     )

    #     rospy.loginfo("waiting five seconds...")
    #     rospy.sleep(5)

    #     # asyncio.run(delete_model(name))
    #     delete_model(name)

    # delete models if any other are in the scene
    tmp = [delete_model(name) for name in urdf_model_names]
    # delete_model(wb_name)
    # delete_model(camera_name)



    s = rospy.Service('/scene_1_stack_builder', Scene1StackBuilder, stack_builder_func)
    print("Spawn Gazebo World Components Service Ready ...")


    rospy.spin()