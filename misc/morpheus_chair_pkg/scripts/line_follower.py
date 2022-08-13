#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from rgb_hsv import BGR_HSV


class LineFollower(object):
    def __init__(self, rgb_to_track, colour_error = 10.0,colour_cal=False, camera_topic="/morpheus_bot/raspicam_node/image_raw", cmd_vel_topic="/morpheus_bot/cmd_vel"):

        self._colour_cal = colour_cal
        self._colour_error = colour_error
        self.rgb_hsv = BGR_HSV()
        self.hsv, hsv_numpy_percentage = self.rgb_hsv.rgb_hsv(rgb=rgb_to_track)
        # We check which OpenCV version is installed.
        (self.major, minor, _) = cv2.__version__.split(".")
        rospy.logwarn("OpenCV Version Installed==>"+str(self.major))

        # This way we process only half the frames
        self.process_this_frame = True
        self.droped_frames = 0
        # 1 LEFT, -1 Right, 0 NO TURN
        self.last_turn = 0

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, CompressedImage, self.camera_callback)
        #self.image_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

    def camera_callback(self, data):

        # It seems that making tests, the rapsicam doesnt update the image until 6 frames have passed
        self.process_this_frame = self.droped_frames >= 2

        if self.process_this_frame:
            # We reset the counter
            print("Process Frame, Dropped frame to==" + str(self.droped_frames))
            self.droped_frames = 0
            try:
                # We select bgr8 because its the OpenCV encoding by default
                #### direct conversion to CV2 ####
                np_arr = np.fromstring(data.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
                #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
                cv_image = None

            if cv_image is not None:
                small_frame = cv2.resize(cv_image, (0, 0), fx=0.1, fy=0.1)

                height, width, channels = small_frame.shape

                rospy.logdebug("height=%s, width=%s" % (str(height), str(width)))

                #descentre = 160
                #rows_to_watch = 100
                #crop_img = small_frame[(height) / 2 + descentre:(height) / 2 + (descentre + rows_to_watch)][1:width]
                crop_img = small_frame

                # Convert from RGB to HSV
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

                min_hsv = self.hsv * (1.0-(self._colour_error / 100.0))
                max_hsv = self.hsv * (1.0 + (self._colour_error / 100.0))

                min_hsv = [0,0,0]
                #max_hsv = [350,55,100]
                max_hsv = [180,255,70]
                lower_yellow = np.array([min_hsv])
                upper_yellow = np.array(max_hsv)

                print("min_hsv: "+str(min_hsv))
                print("max_hsv: "+str(max_hsv))

                # Threshold the HSV image to get only yellow colors
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                # Bitwise-AND mask and original image
                res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

                if self.major == '3':
                    # If its 3
                    (_, contours, _) = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)

                else:
                    # If its 2 or 4
                    (contours, _) = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
                rospy.logdebug("Number of centroids==>" + str(len(contours)))
                centres = []
                for i in range(len(contours)):
                    if cv2.contourArea(contours[i]) > 100:
                        moments = cv2.moments(contours[i])
                        try:
                            centres.append((int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])))
                            cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
                        except ZeroDivisionError:
                            pass

                rospy.logdebug(str(centres))
                centroids_detected = []

                if len(centres) > 0:
                    try:
                        cx = centres[0][0]
                        cy = centres[0][1]
                        rospy.loginfo("Centroid FOUND ==" + str(cx) + "," + str(cy) + "")
                    except:
                        cy, cx = height / 2, width / 2

                    centroids_detected.append([cx,cy])
                    # Draw the centroid in the result image
                    cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)

                if self._colour_cal:
                    cv2.imshow("Original", small_frame)
                    cv2.waitKey(1)
                else:
                    cv2.imshow("CropImg", crop_img)
                    cv2.waitKey(1)
                    #cv2.imshow("HSV", hsv)
                    #cv2.waitKey(1)
                    cv2.imshow("MASK", mask)
                    cv2.waitKey(1)
                    cv2.imshow("RES", res)
                    cv2.waitKey(1)

                # We send data from the first cetroid we get
                if len(centroids_detected) > 0:
                    cx_final = centroids_detected[0][0]
                    cy_final = centroids_detected[0][1]
                else:
                    cx_final = None
                    cy_final = None

                #self.move_robot(height, width, cx_final, cy_final)
            else:
                pass

        else:
            self.droped_frames += 1
            print("Droped Frames==" + str(self.droped_frames))
            
            
            
    def move_robot(self, image_dim_y, image_dim_x, cx, cy, linear_vel_base = 0.4, lineal_vel_min= 0.23, angular_vel_base = 0.2, movement_time = 0.02):
        """
        It move the Robot based on the Centroid Data
        image_dim_x=96, image_dim_y=128
        cx, cy = [(77, 71)]
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0

        cmd_vel_simple = Twist()
        
        FACTOR_LINEAR = 0.001
        FACTOR_ANGULAR = 0.1

        delta_left_percentage_not_important = 0.1
        min_lin = 0.26
        min_ang = 0.7
        
        if cx is not None and cy is not None:
            origin = [image_dim_x / 2.0, image_dim_y / 2.0]
            centroid = [cx, cy]
            delta_left_right = centroid[0] - origin[0]
            print("delta_left_right===>" + str(delta_left_right))
            if delta_left_right <= image_dim_x * delta_left_percentage_not_important:
                print("delta_left_right TO SMALL <=" + str(image_dim_x* delta_left_percentage_not_important))
                delta_left_right = 0.0
            delta = [delta_left_right, centroid[1]]

            # -1 because when delta is positive we want to turn right, which means sending a negative angular
            cmd_vel.angular.z = angular_vel_base * delta[0] * FACTOR_ANGULAR * -1
            # If its further away it has to go faster, closer then slower
            # We place a minimum based on the real robot. Below this cmd_vel the robot just doesnt move properly
            cmd_vel.linear.x = linear_vel_base - delta[1] * FACTOR_LINEAR

            if cmd_vel.angular.z > 0:
                self.last_turn = 1
            elif cmd_vel.angular.z < 0:
                self.last_turn = -1
            elif cmd_vel.angular.z == 0:
                self.last_turn = 0

            
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

            #if self.last_turn > 0:
            #    cmd_vel.angular.z = -angular_vel_base
            #elif self.last_turn <= 0:
            #    cmd_vel.angular.z = angular_vel_base



            print("NO CENTROID DETECTED...SEARCHING...")


        if cmd_vel.linear.x > 0:
            cmd_vel_simple.linear.x = min_lin
        elif cmd_vel.linear.x < 0:
            cmd_vel_simple.linear.x = -min_lin
        elif cmd_vel.linear.x == 0:
            cmd_vel_simple.linear.x = 0

        if cmd_vel.angular.z > 0:
            cmd_vel_simple.angular.z = min_ang
        elif cmd_vel.angular.z < 0:
            cmd_vel_simple.angular.z = -min_ang
        elif cmd_vel.angular.z == 0:
            cmd_vel_simple.angular.z = 0

        print("SPEED==>["+str(cmd_vel_simple.linear.x)+","+str(cmd_vel_simple.angular.z)+"]")
        self.cmd_vel_pub.publish(cmd_vel_simple)
        # We move for only a fraction of time
        init_time = rospy.get_time()
        finished_movement_time = False
        rate_object = rospy.Rate(10)
        while not finished_movement_time:
            now_time = rospy.get_time()
            delta = now_time - init_time
            finished_movement_time = delta >= movement_time
            rate_object.sleep()

        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        print("Movement Finished...")

    def publish_cmd_vel(self):
        self.update_cmd_vel()
        #cmd_vel_msg = TwistStamped()
        #cmd_vel_msg.header = Header(stamp=self.last_line_position_time, frame_id='line_sensor')
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = self.angular_velocity
        #cmd_vel_msg.twist = twist_msg

        #self._log('publish_cmd_vel cb : ' + str(self.linear_velocity) + ' , ' + str(self.angular_velocity))

        self.cmd_vel_pub.publish(twist_msg)

    def publish_lateral_displacement(self):
        lateral_displacement_msg = Int8()
        lateral_displacement_msg.data = self.line_position

        #self._log('lateral_displacement cb : ' + str(self.line_position))

        self.lateral_displacement_pub.publish(lateral_displacement_msg)

    def update_cmd_vel(self):
        self.pid.update(self.line_position)
        control_value = min(abs(self.pid.output), self.max_angular_velocity)
        self.angular_velocity = 0
        if control_value != 0:
            #self.angular_velocity = self.min_linear_velocity + (
            #        self.max_angular_velocity - self.min_linear_velocity) * control_value
            self.angular_velocity = control_value #* self.max_angular_velocity
            self.angular_velocity = self.angular_velocity * copysign(1,self.linear_velocity) # Invert Direction if velocity is negative
            self.angular_velocity = self.angular_velocity * self.pid.output / abs(self.pid.output)


        #linear_coeff = 0.5  # 0.5 for better forward motion, 1 for better turning (& bumpy forward motion)
        #self.linear_velocity = self.min_linear_velocity + (self.max_linear_velocity - self.min_linear_velocity) * (
        #        1 - linear_coeff * control_value)
        self._log('line position : ' + str(self.line_position) + ' , ang vel ' + str(self.angular_velocity))

        #self.linear_velocity = 0

    def calculate_position(self, image_dim_y, image_dim_x, cx, cy):
        position = 0
        if cx is not None and cy is not None:
            origin = [image_dim_x / 2.0, image_dim_y / 2.0]
            centroid = [cx, cy]
            delta_left_right = centroid[0] - origin[0]
            print("delta_left_right===>" + str(delta_left_right))
            if delta_left_right <= image_dim_x * delta_left_percentage_not_important:
                print("delta_left_right TO SMALL <=" + str(image_dim_x* delta_left_percentage_not_important))
                delta_left_right = 0.0
            delta = [delta_left_right, centroid[1]]
            position = delta_left_right
        else:
            position = 0

        return position

    def run(self):
        r = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_cmd_vel()
            self.publish_lateral_displacement()
            r.sleep()

    def _log(self, log_string):
        log_string = '[' + self._node_name + '] ' + log_string
        rospy.loginfo(log_string)

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('line_follower_start', anonymous=True)

    rospy.loginfo(str(len(sys.argv)))
    rospy.loginfo(str(sys.argv))

    if len(sys.argv) > 5:
        red_value = int(float(sys.argv[1]))
        green_value = int(float(sys.argv[2]))
        blue_value = int(float(sys.argv[3]))
        colour_error_value = float(sys.argv[4])
        mode_value = sys.argv[5]

        is_colour_cal = mode_value == "colour_cal"

        #rgb_to_track = [255,255,255]
        rgb_to_track = [red_value, green_value, blue_value]
        robot_mover = LineFollower(rgb_to_track=rgb_to_track,
                                   colour_error= colour_error_value,
                                   colour_cal=is_colour_cal)
        robot_mover.loop()

        robot_mover.run()
