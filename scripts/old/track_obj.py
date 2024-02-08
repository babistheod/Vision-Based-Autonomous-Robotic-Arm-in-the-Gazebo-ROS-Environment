#!/usr/bin/env python3

import os
import traceback
import rospy

import numpy as np

import cv2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo

import threading

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from geometry_msgs.msg import Pose2D

from ur5_pick_place_gazebo.srv import TrackObj, TrackObjRequest, TrackObjResponse



class TrackObject:
    def __init__(self) -> None:


        self.track_obj_srv = rospy.Service('/track_obj', TrackObj, self.track_obj_cb)
        print("Track Object Service is ready ...")

        self.bridge = cv_bridge.CvBridge()
        self.error_px_pub = rospy.Publisher('/error_loc_px', Pose2D, queue_size=1)

        # self.image_sub = rospy.Subscriber('asadasdasdasdasdasd', Image, self.image_cb)

        self.color_book_keys = [
            "red",
            "green",
            "blue",
            "yellow"
        ]

        self.color_book = {
            "red"       : np.array([[ 0,  100, 100] , [10, 255, 255]]),
            "green"     : np.array([[ 60,  100, 100] , [65, 255, 255]]),
            "blue"      : np.array([[ 115,  100, 100] , [120, 255, 255]]),
            "yellow"    : np.array([[ 30,  100, 100] , [35, 255, 255]]),
        }

        self.is_image_sub_active = False

        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_cb)



    def track_obj_cb(self, req):   #TrackObjRequest
        rospy.loginfo("Track Obj Service called for {} color and trigger was {}".format(req.color, req.trigger))
        
        try:
            if req.trigger:
                self.selected_color = req.color
                # Set up the detector with default parameters.
                # detector = cv2.SimpleBlobDetector()
                self.bd_params = cv2.SimpleBlobDetector_Params()
                # self.bd_params.filterByArea = True

                # self.bd_params.filterByCircularity = True
                # self.bd_params.minCircularity = 0.1


                # self.bd_params.filterByInertia = True
                # self.bd_params.minInertiaRatio = 0.25
                # self.bd_params.maxInertiaRatio = 1

                # self.bd_params.filterByColor = True
                self.bd_params.blobColor = 255
                self.bd_params.filterByArea = True
                self.bd_params.minArea = 10
                self.bd_params.maxArea = 30000000
                self.bd_params.filterByCircularity = False
                self.bd_params.filterByConvexity = False
                self.bd_params.filterByColor = False

                ver = (cv2.__version__).split('.')
                if int(ver[0]) < 3 :
                    self.detector = cv2.SimpleBlobDetector(self.bd_params)
                else: 
                    self.detector = cv2.SimpleBlobDetector_create(self.bd_params)

                self.camera_info = rospy.wait_for_message("/ur5/usbcam/camera_info", CameraInfo)#, rospy.Duration(5))
                # self.camera_info = CameraInfo()
                
                print("self.camera_info.K: ", self.camera_info.K)

                self.fx = self.camera_info.K[0]
                self.fy = self.camera_info.K[4]
                
                self.cx = self.camera_info.K[2]
                self.cy = self.camera_info.K[5]

                # print(self.color_book[self.selected_color])

                # cv2.namedWindow("Keypoints", cv2.WINDOW_GUI_NORMAL)
                
                # if self.is_image_sub_active:
                    # self.image_sub.unregister()
                    # cv2.destroyWindow("Keypoints")
                    # rospy.sleep(1)
                    # cv2.destroyWindow("Keypoints")
                
                self.is_image_sub_active = True

                return True
            

            else:
                self.is_image_sub_active = False
                # self.image_sub.unregister()
                # cv2.destroyWindow("Keypoints")
                return True

        except Exception as ex:
            print(ex.args)
            traceback.print_exc()
            return False



    def image_cb(self, msg):
        if self.is_image_sub_active:

            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                img_blur = cv2.GaussianBlur(img, (5,5), 0)

                img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

                # cv2.imshow("img blur", img_blur)
                # cv2.waitKey()

                # print(self.color_book[self.selected_color])

                mask = cv2.inRange(
                    img_hsv, 
                    self.color_book[self.selected_color][0], 
                    self.color_book[self.selected_color][1]
                )


                # Detect blobs.
                keypoints = self.detector.detect(mask)
                # print("len(keypoints): ", len(keypoints))

                # is_empty = False
                for keypoint in keypoints:
                # if len(keypoints) > 0:
                    # x = keypoints[0].pt[0]
                    # y = keypoints[0].pt[1]
                    # r = keypoints[0].size / 2

                    # offset in y should be around 200px
                    x = keypoint.pt[0]
                    y = keypoint.pt[1]
                    r = keypoint.size / 2

                    # print("x: {}, y: {}, self.cy: {}, self.cy: {}".format(x,y,self.cx,self.cy))

                    error_px = Pose2D(
                        x = x - self.cx,
                        y = y - self.cy,
                        theta = 0,
                        )

                    self.error_px_pub.publish(error_px)

                    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
                    mask = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                    break


                # Show keypoints
                cv2.namedWindow("Keypoints", cv2.WINDOW_GUI_NORMAL)
                cv2.imshow("Keypoints", mask)
                cv2.waitKey(1)


            except Exception as ex:
                print(ex.args)
                traceback.print_exc()





if __name__ == "__main__":
    rospy.init_node("track_obj_node", anonymous=True)

    to = TrackObject()


    rospy.spin()

    cv2.destroyAllWindows()
















# img = cv2.imread("test_image.png", cv2.IMREAD_UNCHANGED)

# img_blur = cv2.GaussianBlur(img, (5,5), 0)

# img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

# # BEGIN FILTER
# lower_red = np.array([ 0,  100, 100])
# upper_red = np.array([10, 255, 255])

# # sat and val are same for theese colors
# # 0   - 100   red
# # 120 - 130   green
# # 240 - 250   blue
# # 60  - 70    yellow

# color_book_keys = [
#     "red",
#     "green",
#     "blue",
#     "yellow"
# ]

# color_book = {
#     "red"       : np.array([[ 0,  100, 100] , [10, 255, 255]]),
#     "green"     : np.array([[ 60,  100, 100] , [65, 255, 255]]),
#     "blue"      : np.array([[ 115,  100, 100] , [120, 255, 255]]),
#     "yellow"    : np.array([[ 30,  100, 100] , [35, 255, 255]]),
# }

# tracker_thread = threading.Thread()
# tracker_event = threading.Event()


# #lower_red = np.array([ 36,  0, 0])
# #upper_red = np.array([86, 255, 255])
# # mask = cv2.inRange(img_hsv, color_book["red"][0], color_book["red"][1])


# cv2.imshow("img blur", img_blur)
# cv2.waitKey()



# # Set up the detector with default parameters.
# # detector = cv2.SimpleBlobDetector()
# params = cv2.SimpleBlobDetector_Params()
# # params.filterByArea = True
# # params.filterByCircularity = True

# params.filterByInertia = True
# params.minInertiaRatio = 0.5
# params.maxInertiaRatio = 1

# params.filterByColor = False

# ver = (cv2.__version__).split('.')
# if int(ver[0]) < 3 :
#     detector = cv2.SimpleBlobDetector(params)
# else: 
#     detector = cv2.SimpleBlobDetector_create(params)



# for color in color_book_keys:
#     print(color_book[color])
#     mask = cv2.inRange(img_hsv, color_book[color][0], color_book[color][1])


#     # Detect blobs.
#     keypoints = detector.detect(mask)

#     print(keypoints)

#     print("detector.empty(): ", detector.empty()) # <- now works
#     # keypoints = detector.detect(image) # <- now works
# # rospy.wait_for_message
# # K: [476.7030836014194, 0.0, 400.5, 0.0, 476.7030836014194, 400.5, 0.0, 0.0, 1.0]
#     fx = fy = 476.7030836014194
#     # cx = cy = 400.5
#     # h, w, c = im.shape
#     h, w = mask.shape
#     cx = w/2
#     cy = h/2
#     Rx = Ry = R = 0.05

#     for k in keypoints:
#         print(k.pt[0], k.pt[1], k.size)
#         x = k.pt[0]
#         y = k.pt[1]
#         r = k.size / 2
        
#         Zx = (fx / r) * Rx
#         Zy = (fy / r) * Ry

#         # Z_expre_p = ((2*X*fx*cx + 2*Y*fy*cy) + np.sqrt((2*X*fx*cx + 2*Y*fy*cy)**2 - 4*(X**2*fx**2 + Y**2*fy**2) * (cx**2 + cy**2 - r)) ) / (2*(X**2*fx**2 + Y**2*fy**2))
#         # Z_expre_m = ((2*X*fx*cx + 2*Y*fy*cy) - np.sqrt((2*X*fx*cx + 2*Y*fy*cy)**2 - 4*(X**2*fx**2 + Y**2*fy**2) * (cx**2 + cy**2 - r)) ) / (2*(X**2*fx**2 + Y**2*fy**2))

#         # print("Z_expre_p: ", Z_expre_p)
#         # print("Z_expre_m: ", Z_expre_m)

#         Z = 0.5*(Zx + Zy)
#         X = (x - cx) * (Z / fx)
#         Y = (y - cx) * (Z / fy)

#         # print("Zx: {}, Zy: {}, Z: {}, ".format(Zx, Zy, Z))
#         print(color)
#         print("X: {}, Y: {}, Z: {}, ".format(X, Y, Z))

#     # Draw detected blobs as red circles.
#     # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
#     im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

#     # Show keypoints
#     cv2.imshow("Keypoints", im_with_keypoints)
#     cv2.waitKey()

# # contours, hierarchy  = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# # area = cv2.contourArea(contours[0])

# # print(contours)
# # print(hierarchy)
# # print(area)

# # cv2.imshow("img blur", mask)
# # cv2.waitKey()

# def track_obj_cb(color):
#     bridge = cv_bridge.CvBridge()
#     image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)
#     error_px_pub = rospy.Publisher('/error_loc_px', Pose2D, queue_size=1)





# def track_obj(req): #TrackObjRequest
#     global tracker_thread, tracker_event

#     if req.trigger:
#         tracker_thread = threading.Thread(track_obj, args=(req.color,))

#         tracker_event.set()
#         tracker_thread.start()
        
#         rospy.sleep(1)
#         if tracker_thread.is_alive():
#             return True
#         else:
#             return False
        
#     else:
#         tracker_event.clear()
#         # rospy.sleep(1)

#         try:
#             tracker_thread.join(5)
#             return True
#         except Exception as ex:            
#             ex.with_traceback(ex)
#             print(ex.args)

#             return False





    



# if __name__ == "__main__":
#     rospy.init_node("track_obj_node", anonymous=True)


#     s = rospy.Service('/track_obj', TrackObj, track_obj)
#     print("Spawn Gazebo World Components Service Ready ...")


#     rospy.spin()
