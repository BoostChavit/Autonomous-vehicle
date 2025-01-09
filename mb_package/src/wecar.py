#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from math import floor
from collections import deque
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import resource

from module import PID
from module import TrafficLightSmoother

class Wecar:
    def __init__(self, pid, center_pos_buffer, wheels_pos_buffer, traffic_light_smoother):
        self.speed = 0
        self.pid = pid
        self.center_pos_buffer = center_pos_buffer
 	self.wheels_pos_buffer = wheels_pos_buffer
        self.traffic_light_smoother = traffic_light_smoother
        self.speed_pub = rospy.Publisher('/vesc/commands/motor/unsmoothed_speed', Float64, queue_size=1)
        self.position_pub = rospy.Publisher('/vesc/commands/servo/unsmoothed_position', Float64, queue_size=1)

    def degTorad(self, deg):
        rad_diff = 0.5304
        rad = deg * (3.14/180)
        return rad + rad_diff


    def detect_light(self, image, gray):
        circles = self.traffic_light_smoother.detect_circles(gray)
        is_traffic_light = self.traffic_light_smoother.check_traffic_light_pattern(circles)
        self.traffic_light_smoother.classify_traffic_light_color_hsvAndLab(image, circles)
	
 	return self.traffic_light_smoother.state

    def control_wheel(self, bird_eye, light_found):
        image_center_x = bird_eye.shape[1] // 2
        low_t = 50
        high_t = 150
        edges = cv2.Canny(bird_eye, low_t, high_t)

        if len(self.wheels_pos_buffer) != 0:
            steering_output = self.wheels_pos_buffer[-1]
        else : 
            steering_output = 0
    
        #find center of lane
        y_lane_pos = 380

        left_lane = 0
        right_lane = edges.shape[1]

        for i in range(0, int(image_center_x/2) + 120):
            if edges[y_lane_pos][i] == 255:
                left_lane = i
                break

        for i in range(edges.shape[1] - 1, image_center_x + int(image_center_x/2) - 160 ,-1):
            if edges[y_lane_pos][i] == 255:
                right_lane = i
                break

        lane_center_x = (right_lane + left_lane) / 2

        cv2.circle(edges, (int(lane_center_x), y_lane_pos), 3, 255, 5)
        cv2.circle(edges, (int(image_center_x), y_lane_pos), 3, 255, 5)
        cv2.circle(edges, (left_lane, y_lane_pos), 3, 255, 5)
        cv2.circle(edges, (right_lane, y_lane_pos), 3, 255, 5)

        # Smoothing the center lane value using buffer
        self.center_pos_buffer.append(lane_center_x)
        lane_center_x = (sum(self.center_pos_buffer) / len(self.center_pos_buffer) * 0.5) + (lane_center_x * 0.5)
        offset = lane_center_x - image_center_x
        global left_offset, right_offset
        print("lane: ",self.traffic_light_smoother.lane)

        if light_found == 1:
            if self.traffic_light_smoother.lane is None:
                print("Normal")
                steering_output = self.pid.update(offset)
            
            elif self.traffic_light_smoother.lane == 0: #left
                steering_output = self.pid.update(offset - left_offset)
                print("Left : ", left_offset)
         	self.wheels_pos_buffer.append(steering_output)
         	self.wheels_pos_buffer.append(steering_output)
                left_offset -= 0.2
                

            elif self.traffic_light_smoother.lane == 1: #right
                steering_output = self.pid.update(offset + right_offset) 
                print("Right : ",right_offset) 
         	self.wheels_pos_buffer.append(steering_output)
         	self.wheels_pos_buffer.append(steering_output)
                right_offset -= 5

            if right_offset <= 0 or left_offset <= 0:
                self.traffic_light_smoother.setLane(None)
                right_offset = 5
                left_offset = 50 
		
            self.wheels_pos_buffer.append(steering_output)
            mean = sum(self.wheels_pos_buffer) / len(self.wheels_pos_buffer)
            print("mean: ", mean)
            steering_output = (mean * 0.5) + (steering_output * 0.5)
	    print("steering_output before degTorad: ", steering_output)
            steering_output = self.degTorad(floor(steering_output))
            
        
        
        return steering_output, edges     
                

    def car_controller(self, image, gray, bird_eye):
        light_found = self.detect_light(image, gray)
        steering_output, edges = self.control_wheel(bird_eye, light_found)

        print("light_found: ", light_found)
        if not pid.isBufferFull() or light_found == 0:
            self.speed_pub.publish(0)         
        
        elif light_found == 1: # not found light or light is green
            #print("speed : ", self.speed)
            #self.speed_pub.publish(self.speed)
            self.position_pub.publish(steering_output)
            
        else:
            print("What Happend!!!")

        return edges


def calculate_speed_based_on_distance(distance):
    SPEED = 2200
    
    if distance < 0.5:
        return 0
    else:
        return SPEED


def bird_eye_view(image):
    height, width = image.shape[:2]
    src_points = np.float32([
        [width * 0.1, height * 0.6],  # top left
        [width * 0.9, height * 0.6],  # top right
        [width * 0.1, height * 0.9],  # bottom left
        [width * 0.9, height * 0.9]   # bottom right
    ])

    dst_points = np.float32([
	[0, 0],	          # top left
	[width, 0],       # top right
	[0, height],      # bottom left
	[width, height]   # bottom right
    ])
	
    homography_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    bird_eye_image = cv2.warpPerspective(image, homography_matrix, (width, height))		

    return bird_eye_image

# Stream Image to Web server
def numpy_to_image(np_array):
    bridge = CvBridge()
    if len(np_array.shape) == 2:
        return bridge.cv2_to_imgmsg(np_array, encoding="mono8") 
    return bridge.cv2_to_imgmsg(np_array, encoding="bgr8") 


def callback_camera(image):
    
    stream_img_pub = rospy.Publisher('/camera/stream_img', Image, queue_size=1)
    stream_canny_pub = rospy.Publisher('/camera/stream_canny', Image, queue_size=1)    

    np_arr = np.frombuffer(image.data, np.uint8) #convert byte data to numpy array

    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) #decode image data to opencv array
    
    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCR_CB)

    Y, Cr, Cb = cv2.split(ycrcb)
    Cr_thresh = cv2.adaptiveThreshold(Cr, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    Cb_thresh = cv2.adaptiveThreshold(Cb, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)   

    yellow_thresh = cv2.bitwise_and(Cr_thresh, Cb_thresh)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    
    _, white_thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    combine = cv2.bitwise_or(yellow_thresh, white_thresh, mask=None)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    morph = cv2.morphologyEx(combine, cv2.MORPH_OPEN, kernel)  
    bird_eye = bird_eye_view(morph)

    edges = wecar.car_controller(image=image, gray=gray, bird_eye=bird_eye)

    # stream img webserver
    stream_img_pub.publish(numpy_to_image(image))
    stream_canny_pub.publish(numpy_to_image(edges))


def callback_lidar(data):
    data = list(data.ranges)[::-1]
    front_distances = data[345:359] + data[0:15]
    min_distance = min(front_distances)
    wecar.speed = calculate_speed_based_on_distance(min_distance)     


def memory_usage():
    mem = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    print("Memory usage: {:.2f} MB".format(mem / 1024))


if __name__ == '__main__':
    try:
        rospy.init_node("find_lane")
	print("not in Autonomous directory")
        pid = PID(0.68, 0.0001, 0.01, 30, -30)
        center_pos_buffer = deque(maxlen=40)
 	wheels_pos_buffer = deque(maxlen=10)
        traffic_light_smoother = TrafficLightSmoother()
        global wecar
        global left_offset, right_offset
        left_offset = 50
        right_offset = 5
        wecar = Wecar(pid, center_pos_buffer, wheels_pos_buffer, traffic_light_smoother)

        lidar_sub = rospy.Subscriber("/scan", LaserScan, callback_lidar)
        camera_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback_camera)
        memory_usage()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
