import rospy

for ii in range(0,5):
    rospy.logerr_once("ii: {}".format(ii))
    rospy.logerr("ii: {}".format(ii))