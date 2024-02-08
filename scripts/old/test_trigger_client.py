#! /usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Int8

# init a node as usual
rospy.init_node('sos_service_client')

# wait for this sevice to be running
rospy.wait_for_service('/test_trigger_srv')

# Create the connection to the service. Remember it's a Trigger service
sos_service = rospy.ServiceProxy('/test_trigger_srv', Trigger)

# Create an object of the type TriggerRequest. We nned a TriggerRequest for a Trigger service
sos = TriggerRequest(Int8(10))

# Now send the request through the connection
result = sos_service(sos)

# Done
print(result)