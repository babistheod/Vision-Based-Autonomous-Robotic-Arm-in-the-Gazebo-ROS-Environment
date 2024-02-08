import pdb
import traceback

import rospy

from enum import IntEnum, auto
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel

import numpy as np
import math3d

import cv2
import cv2.aruco as aruco


class RobotCommanderCmd(IntEnum):
    ALIGN = auto()
    PICK = auto()
    PLACE = auto()
    HOME = auto()

class RobotCommanderStatus(IntEnum):
    ALIGNED = auto()
    PICKED = auto()
    PLACED = auto()
    AT_HOME = auto()
    FAILED = auto()


def to_m3d_transform(trans_geo_pose):
    if not isinstance(trans_geo_pose, Pose):
        raise Exception("input should be of type {}".format(type(trans_geo_pose)))

    t_m3d = math3d.Vector(
        trans_geo_pose.position.x,
        trans_geo_pose.position.y,
        trans_geo_pose.position.z
    )  
    q_m3d = math3d.UnitQuaternion(
        trans_geo_pose.orientation.w,
        trans_geo_pose.orientation.x,
        trans_geo_pose.orientation.y,
        trans_geo_pose.orientation.z
    )
    return math3d.Transform(q_m3d, t_m3d)


def to_geometry_msgs_pose(trans_m3d):
    if not isinstance(trans_m3d, math3d.Transform):
        raise Exception("input should be of type {}".format(type(trans_m3d)))

    q = trans_m3d.orient.quaternion.array
    t = trans_m3d.pos.array
    
    trans_geo_msgs = Pose()
    trans_geo_msgs.position.x = t[0]
    trans_geo_msgs.position.y = t[1]
    trans_geo_msgs.position.z = t[2]
    trans_geo_msgs.orientation.w = q[0]
    trans_geo_msgs.orientation.x = q[1]
    trans_geo_msgs.orientation.y = q[2]
    trans_geo_msgs.orientation.z = q[3]
    
    return trans_geo_msgs


def detect_aruco(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )
    # detect the sruco markers and display its aruco id.
    output = aruco.drawDetectedMarkers(img, corners, ids)
    return output, corners, ids


def compute_center_from_corners(marker_corners):
    # extract the marker corners (which are always returned in
    # top-left, top-right, bottom-right, and bottom-left order)
    corners = np.array(marker_corners).reshape((4, 2))
    # (topLeft, topRight, bottomRight, bottomLeft) = corners
    # # convert each of the (x, y)-coordinate pairs to integers
    # topRight = (int(topRight[0]), int(topRight[1]))
    # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    # topLeft = (int(topLeft[0]), int(topLeft[1]))

    # # compute and draw the center (x, y)-coordinates of the ArUco
    # # marker
    # cx = int((topLeft[0] + bottomRight[0]) / 2.0)
    # cy = int((topLeft[1] + bottomRight[1]) / 2.0)

    # mean of corners will be the center point of the marker
    # cx , cy = corners.mean(axis=0).astype(int)
    # return [cx, cy]
    return corners.mean(axis=0).astype(int)



def compute_all_centers_from_aruco_corners_list(corners_list):
    centers = []
    # verify *at least* one ArUco marker was detected
    if len(corners_list) > 0:
        # loop over the detected ArUCo corners
        for marker_corners in corners_list:
            cx, cy = compute_center_from_corners(marker_corners)
            centers.append([cx, cy])
        return centers
    else:
        return None



def estimate_pose_single_marker(img, id, cam_coeff, dist_coeff):
    markers_img, corners_list, ids_list = detect_aruco(img)
    print("corners_list:\n", corners_list)
    # if type(corners_list) == tuple:
        # corners_list = corners_list[0]

    print("corners_list:\n", corners_list)
    print("ids_list:\n", ids_list)
    print("id: ", id)
    # ids = np.array(ids_list).squeeze()
    # if ids.size == 1:
    #     ids = np.array([ids])
    ids = ids_list.flatten()
    is_id_in_scene = np.any(ids == id)
    
    success = False
    tvec = np.array([])
    rvec = np.array([])
    center = np.array([])

    K = np.array(cam_coeff, dtype=np.float).reshape((3,3))
    D = np.array(dist_coeff, dtype=np.float).reshape((5,1))

    # pdb.set_trace()

    if is_id_in_scene:
        # idx = np.where(ids == id)[0][0]
        idx = np.array(np.where(ids == id)).flatten().astype('uint8')

        print("id: " , id)
        print("ids: " , ids)
        print("idx: " , idx)
        # if len(idx[0]) > 0:
        #     idx = idx[0][0]
        # corners = np.array(corners_list[idx], dtype=np.float)
        # corners = corners.reshape((*corners.shape, 1))

        # print("corners:\n", corners)
        # print("corners.shape:\n", corners.shape)

        print("K:\n", K)
        print("D:\n", D)

        rvec, tvec, marker_pts = cv2.aruco.estimatePoseSingleMarkers(
            # img,
            corners_list[idx[0]], # corners,#.astype('float32'), 
            np.float(0.0781),
            K,#.astype('float32'),
            D,#.astype('float32'),
        )

        tvec = np.array(tvec).flatten()
        rvec = np.array(rvec).flatten()
        center = compute_center_from_corners(corners_list[idx[0]])
        success = True

        print("tvec:\n", tvec)
        print("rvec:\n", rvec)


    # else:
        # success = False
        # tvec = np.array([])
        # rvec = np.array([])
        # center = np.array([])


    return success, tvec, rvec, center



# async def spawn_model(name, path, pose=Pose()):
def spawn_model(srv_name, name, path, pose=Pose()):
    try:
        # print(urdf_path)

        # rospy.loginfo("spawning \"" + obj_path + "\" at" )
        print("spawning \"" , path , "\" at" )
        print(pose)

        # rospy.wait_for_service(srv_name, timeout=5)
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
        # ex.with_traceback(ex)
        print(ex.args)
        return False


def delete_model(name):
    try:
        # rospy.loginfo("spawning \"" + name + "\" at" )
        print("deleting \"" , name , "\"" )

        # rospy.wait_for_service('/gazebo/delete_model', timeout=5)
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