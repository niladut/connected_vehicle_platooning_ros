#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyRequest

if __name__ == '__main__':
    rospy.init_node('start_camera_service', anonymous=True)
    rospy.wait_for_service('/morpheus_bot/raspicam_node/start_capture')
    start_cam = rospy.ServiceProxy('/morpheus_bot/raspicam_node/start_capture', Empty)
    request_e = EmptyRequest()
    start_cam(request_e)
    rospy.loginfo("Started Camera Service")