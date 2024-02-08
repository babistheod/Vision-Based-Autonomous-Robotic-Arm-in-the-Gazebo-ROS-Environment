#!/usr/bin/env python3

import rospy

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse



def callback(req):
    print("received: \n", req)

    return TriggerResponse(success=True, message="success")



rospy.init_node("test_trigger_srv_node", anonymous=True)

s = rospy.Service('/test_trigger_srv', Trigger, callback)

rospy.spin()
