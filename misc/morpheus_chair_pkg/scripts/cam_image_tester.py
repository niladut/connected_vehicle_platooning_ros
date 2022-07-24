#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CamTester(object):
    def __init__(self, camera_topic="/morpheus_bot/raspicam_node/image_raw"):

        # We check which OpenCV version is installed.
        (self.major, minor, _) = cv2.__version__.split(".")
        rospy.logwarn("OpenCV Version Installed==>"+str(self.major))

        # This way we process only half the frames
        self.process_this_frame = True
        self.droped_frames = 0

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)


    def camera_callback(self, data):

        # It seems that making tests, the rapsicam doesnt update the image until 6 frames have passed
        self.process_this_frame = self.droped_frames >= 0

        if self.process_this_frame:
            # We reset the counter
            print("Process Frame, Dropped frame to==" + str(self.droped_frames))
            self.droped_frames = 0
            try:
                # We select bgr8 because its the OpenCV encoding by default
                cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
                cv_image = None

            small_frame = cv2.resize(cv_image, (0, 0), fx=0.2, fy=0.2)

            height, width, channels = small_frame.shape

            rospy.logdebug("height=%s, width=%s" % (str(height), str(width)))
            cv2.imshow("Original", small_frame)
            cv2.waitKey(1)

            #raw_input("Press to process next image")



        else:
            self.droped_frames += 1
            print("Dropped Frames==" + str(self.droped_frames))
            

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cam_image_test_start', anonymous=True)
    cam_tester = CamTester()
    cam_tester.loop()
