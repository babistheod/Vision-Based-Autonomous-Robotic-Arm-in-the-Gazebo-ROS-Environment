#!/usr/bin/env python3
import traceback

import rospy

from geometry_msgs.msg import Point, Quaternion

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from pkg_utilts import estimate_pose_single_marker

from ur5_pick_place_gazebo.srv import EstimatePose, EstimatePoseRequest, EstimatePoseResponse 
# aruco_id_range = np.arange(1, 17)   # valid ids
# detected_arucos_ids = np.array([], dtype=np.int)


class ObjectPoseEstimator:
    def __init__(self):
        self.estimate_pose_srv = rospy.Service('/estimate_obj_pose_scene_2', EstimatePose, self.estimate_pose_cb)
        print("Track Object Service is ready ...")


        self.bridge = CvBridge()


    def estimate_pose_cb(self, req):
        try:
            cam_info_msg = rospy.wait_for_message("/ur5/usbcam/camera_info", CameraInfo)
            is_successfull = False
            rate = rospy.Rate(10)
            while not is_successfull:
                # make sure we are accepting the latest frame
                for ii in range(20):
                    img_msg = rospy.wait_for_message("/ur5/usbcam/image_raw", Image)
                    rate.sleep()
    
    
                id = req.id

                try:
                    cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                except CvBridgeError as e:
                    print(traceback.format_exc())
                    print(e)

                is_successfull, tvec, rvec, center = estimate_pose_single_marker(
                    cv_image,
                    id,
                    cam_info_msg.K,
                    cam_info_msg.D
                )

                print("is_successfull: ", is_successfull)
                print("tvec: ", tvec)
                print("rvec: ", rvec)
                print("center: ", center)


            res = EstimatePoseResponse()

            res.success = is_successfull
            res.pose.position = Point(x=tvec[0], y=tvec[1], z=tvec[2])
            res.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)    # not important
            
            rospy.loginfo("sending response: \n{}".format(res))
            
            return res
            
        
        except Exception as ex:
            rospy.logerr("{}".format(ex.args))
            print(traceback.format_exc())

            res = EstimatePoseResponse()

            res.success = False
            res.pose.position = Point()
            res.pose.orientation = Quaternion()    # not important
            # res.stackNum = 0
            return res


def main():
    rospy.init_node('publish_latest_aruco', anonymous=True)

    ope = ObjectPoseEstimator()

    rospy.spin()


if __name__ == '__main__':
    main()

        

