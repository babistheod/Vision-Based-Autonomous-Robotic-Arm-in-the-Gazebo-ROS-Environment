#!/usr/bin/env python3

import os
import traceback
import asyncio

import rospy

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

import rospkg


async def spawn_model(urdf_path, pose):
    try:
        # print(urdf_path)
        # while not urdf_path.empty() and pose.empty():
        obj_path = urdf_path
        obj_pose = pose

        # rospy.loginfo("spawning \"" + obj_path + "\" at" )
        print("spawning \"" , obj_path , "\" at" )
        print(obj_pose)

        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        spawn_model_service(
            model_name=obj_path.split("/")[-1].split(".")[0],  # extract the name of the file
            model_xml=open(obj_path, 'r').read(),
            # robot_namespace='/foo',
            initial_pose=obj_pose,
            reference_frame='world'
        )
    except Exception as ex:
        traceback.print_exc()
        print(ex.args)


async def delete_model(urdf_path):
    try:
        # print(urdf_path)
        # while not urdf_path.empty() and pose.empty():
        obj_path = urdf_path

        # rospy.loginfo("spawning \"" + obj_path + "\" at" )
        print("deleting \"" , obj_path , "\"" )

        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_res = delete_model_service(
            model_name=obj_path.split("/")[-1].split(".")[0],  # extract the name of the file
        )
        print(delete_model_res)
    except Exception as ex:
        traceback.print_exc()
        print(ex.args)



async def main():
    rp = rospkg.RosPack()

    # setting names and the pose of the object
    urdf_path = os.path.join(rp.get_path("ur5_pick_place_gazebo"), "urdf")

    workbench_urdf_path = os.path.join(rp.get_path("ur5_pick_place_gazebo"), "urdf", "workbench.urdf")
    workbench_pose = Pose(position= Point(0.6,     0,      0.0051), orientation=Quaternion(0,0,0,1))

    print(workbench_urdf_path)

    test_objs = [
        os.path.join(urdf_path, "blue_cylinder.urdf"),
        os.path.join(urdf_path, "green_cylinder.urdf"),
        os.path.join(urdf_path, "red_cylinder.urdf"),
        os.path.join(urdf_path, "yellow_cylinder.urdf"),
    ]

    obj_poses = [
        Pose(position= Point(0.45,  0,      0.0251),orientation=Quaternion(0,0,0,1)),
        Pose(position= Point(0.65,  0,      0.0251),orientation=Quaternion(0,0,0,1)),
        Pose(position= Point(0.55,  -0.1,  0.0251),orientation=Quaternion(0,0,0,1)),
        Pose(position= Point(0.55,  0.1,   0.0251),orientation=Quaternion(0,0,0,1)),
    ]

    # setting the work in async queues
    # obj_que = asyncio.Queue()
    # for obj in test_objs:
    #     await obj_que.put(obj)
    

    # poses_que = asyncio.Queue()
    # for pose in obj_poses:
    #     await poses_que.put(pose)


    # calling the work in async queues
    # await asyncio.gather(
    #         asyncio.create_task(spawn_model(obj_que, poses_que)),
    #     )

    # results = await asyncio.gather(map(spawn_model, obj_que, poses_que))

    # delete the model first if they existed in Gazebo
    await delete_model(workbench_urdf_path)

    await asyncio.gather(*map(delete_model, test_objs))



    # now spawn the model in Gazebo
    # await spawn_model(workbench_urdf_path, workbench_pose)

    await asyncio.gather(*map(spawn_model, test_objs, obj_poses))



def spawn_srv_cb(EmptyRequest):
    asyncio.run(main())
    # loop=asyncio.get_event_loop()
    # loop.run_until_complete(main())

    return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("spawn_world_components_node", anonymous=True)

    s = rospy.Service('/spawn_world_components', Empty, spawn_srv_cb)
    print("Spawn Gazebo World Components Service Ready ...")


    rospy.spin()
