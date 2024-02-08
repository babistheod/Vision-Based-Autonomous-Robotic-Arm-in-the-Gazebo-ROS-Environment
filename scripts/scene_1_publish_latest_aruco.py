#!/usr/bin/env python3
import traceback
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int8, Int8MultiArray

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from pkg_utilts import detect_aruco, compute_center_from_corners

from ur5_pick_place_gazebo.srv import DetectMarkerIDAndStack, DetectMarkerIDAndStackRequest, DetectMarkerIDAndStackResponse

# aruco_id_range = np.arange(1, 17)   # valid ids
# detected_arucos_ids = np.array([], dtype=np.int)


class PublishLatestAruco:
    def __init__(self):
        self.latest_aruco_srv = rospy.Service("/latest_aruco_id", DetectMarkerIDAndStack, self.img_cb)
        
        # self.detected_arucos_pub = rospy.Publisher(
        #     "/detected_aruco_ids", Int8MultiArray)

        self.bridge = CvBridge()

        # self.image_sub = rospy.Subscriber(
        #     "/scene1/camera1/image_raw", Image, self.img_cb)

        # self.detected_arucos_timer = rospy.Timer(rospy.Duration(0.25), self.detected_arucos_cb, oneshot=False)
        # self.detected_arucos_timer.start()
        
        self.aruco_id_range = np.arange(1, 17)   # valid ids

        self.detected_arucos_msg = Int8MultiArray()
        
        self.detected_arucos_arr = np.array([], dtype=np.int)

        self.stackNum = 0


    # def detected_arucos_cb(self, event):
        # print("self.detected_arucos_msg: ", self.detected_arucos_msg)
        # self.detected_arucos_pub.publish(self.detected_arucos_msg)


    def img_cb(self, req):
        res = DetectMarkerIDAndStackResponse()
        res.success = False
        res.stackNum = 0
        res.id = 0

        try:
            self.cam_info = rospy.wait_for_message("/scene1/camera1/camera_info", CameraInfo)

            # make sure we are accepting the latest frame
            rate = rospy.Rate(10)
            for ii in range(20):
                msg = rospy.wait_for_message("/scene1/camera1/image_raw", Image)
                rate.sleep()

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            markers_img, corners_list, ids_list = detect_aruco(cv_image)

            ids = np.array(ids_list).flatten()
            if ids.size == 1:
                ids = np.array([ids])

            # print("detecting markers: ", ids)

            ids_in_range = np.array([
                id >= self.aruco_id_range.min() and id <= self.aruco_id_range.max() 
                for id in ids
            ])

            valid_ids = ids[ids_in_range]

            # print("ids_in_range", ids_in_range)
            # print("valid_ids", valid_ids)


            new_id = 0
            if valid_ids.size > 0:
                ids_already_detected = np.array([
                    np.any(self.detected_arucos_arr == id) for id in valid_ids
                ])
                print("ids_already_detected", ids_already_detected)
                new_id = valid_ids[np.bitwise_not(ids_already_detected)]
                print("new_id", new_id)
                if len(new_id) > 0:
                    new_id = new_id[0]
                    self.detected_arucos_arr = np.append(self.detected_arucos_arr, new_id)
                    self.detected_arucos_msg.data.append(new_id)

                    print("self.detected_arucos_arr: ", self.detected_arucos_arr)
                else:
                    # if now new id found deal latest id as a new id, to further proceed the process
                    new_id = self.detected_arucos_arr[-1]

                idx = np.array(np.where(ids == new_id)).flatten()[0]
                cx, cy = compute_center_from_corners(corners_list[idx])
                threshold = 28
                if cy > int(self.cam_info.height* 0.75) - threshold and cy < int(self.cam_info.height * 0.75) + threshold:
                    self.stackNum = 1
                elif cy > int(self.cam_info.height* 0.5) - threshold and cy < int(self.cam_info.height * 0.5) + threshold:
                    self.stackNum = 2
                elif cy > int(self.cam_info.height* 0.25) - threshold and cy < int(self.cam_info.height * 0.25) + threshold:
                    self.stackNum = 3
                else:
                    self.stackNum = 0
                    rospy.logerr("couldn't find the stack location, setting stack location at {}".format(self.stackNum))

                rospy.loginfo("id is in stack {} and centered at {}".format(self.stackNum, [cx, cy]))

                res.success = True
                res.stackNum = self.stackNum
                res.id = self.detected_arucos_arr[-1]

                # else:
                #     res.success = True
                #     res.stackNum = self.stackNum
                #     res.id = self.detected_arucos_arr[-1]

                # else:
                    # new_id = None
            # else:
                # new_id = valid_ids[0]
                # print("new_id", new_id)
                

            # if new_id:
                # self.detected_arucos_arr = np.append(self.detected_arucos_arr, new_id)
                # self.detected_arucos_msg.data.append(new_id)

                # check the location of stack from image based on y-axis of camera
                # lower in image --> lower in gazebo y-axis
                # lower is first and higher is last
                # cam_info_msg = CameraInfo()
                # new id's idx
                # idx = np.array(np.where(ids == new_id)).flatten()[0]
                # cx, cy = compute_center_from_corners(corners_list[idx])
                # threshold = 10
                # if cy > int(self.cam_info.height* 0.75) - threshold and cy < int(self.cam_info.height * 0.75) + threshold:
                #     self.stackNum = 1
                # elif cy > int(self.cam_info.height* 0.5) - threshold and cy < int(self.cam_info.height * 0.5) + threshold:
                #     self.stackNum = 2
                # elif cy > int(self.cam_info.height* 0.25) - threshold and cy < int(self.cam_info.height * 0.25) + threshold:
                #     self.stackNum = 3


            # res.success = True
            # res.stackNum = self.stackNum
            # res.id = self.detected_arucos_arr[-1]

            return res

            # res = DetectMarkerIDAndStackResponse()
            # res.success = False
            # res.stackNum = 0
            # res.id = 0
            # if len(self.detected_arucos_msg.data) > 0:
            #     res.success = True
            #     res.stackNum = self.stackNum
            #     res.id = new_id

                # publish new id
                # print("Int8(data=self.detected_arucos_msg.data[-1]): ", Int8(data=self.detected_arucos_msg.data[-1]))
                # self.latest_aruco_pub.publish(
                #     DetectMarkerIDAndStack(
                #         id=self.detected_arucos_msg.data[-1],
                #         stackNum=self.stackNum,
                #     )
                # )
            # else:
            #     self.latest_aruco_pub.publish(Int8())






        except Exception as ex:
            res.success = False
            res.stackNum = 0
            res.id = 0

            print(ex)
            print(traceback.format_exc())
            return res





def main():
    rospy.init_node('publish_latest_aruco', anonymous=True)

    pla = PublishLatestAruco()

    rospy.spin()


if __name__ == '__main__':
    main()
