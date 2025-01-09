#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage	
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Stream Image to Web server
def numpy_to_image(np_array):
    bridge = CvBridge()
    if len(np_array.shape) == 2:
        return bridge.cv2_to_imgmsg(np_array, encoding="mono8") 
    return bridge.cv2_to_imgmsg(np_array, encoding="bgr8") 


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



def callback(image):
    np_arr = np.frombuffer(image.data, np.uint8) #convert byte data to numpy array
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) #decode image data to opencv array
    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCR_CB)

    Y, Cr, Cb = cv2.split(ycrcb)
    Cr_thresh = cv2.adaptiveThreshold(Cr, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    Cb_thresh = cv2.adaptiveThreshold(Cb, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)   

    yellow_thresh = cv2.bitwise_and(Cr_thresh, Cb_thresh)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    
    _, white_thresh = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
    combine = cv2.bitwise_or(yellow_thresh, white_thresh, mask=None)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    morph = cv2.morphologyEx(combine, cv2.MORPH_OPEN, kernel)  
    bird_eye = bird_eye_view(morph)

    y_lane_pos = 380

    image_center_x = bird_eye.shape[1] // 2
    low_t = 50
    high_t = 150
    edges = cv2.Canny(bird_eye, low_t, high_t)
    left_lane = 0
    right_lane = edges.shape[1]

    for i in range(0, int(image_center_x/2) + 140):
        if edges[y_lane_pos][i] == 255:
            left_lane = i
            break

    for i in range(edges.shape[1] - 1, image_center_x + int(image_center_x/2) - 140 ,-1):
        if edges[y_lane_pos][i] == 255:
            right_lane = i
            break

    lane_center_x = (right_lane + left_lane) / 2

    cv2.circle(edges, (int(lane_center_x), y_lane_pos), 3, 255, 5)
    cv2.circle(edges, (int(image_center_x), y_lane_pos), 3, 255, 5)
    cv2.circle(edges, (left_lane, y_lane_pos), 3, 255, 5)
    cv2.circle(edges, (right_lane, y_lane_pos), 3, 255, 5)

    stream_img_pub = rospy.Publisher('/camera/stream_img', Image, queue_size=1)
    stream_canny_pub = rospy.Publisher('/camera/stream_canny', Image, queue_size=1)
    stream_img_pub.publish(numpy_to_image(morph))
    stream_canny_pub.publish(numpy_to_image(edges))


if __name__ == '__main__':
    try:
        rospy.init_node("CameraTest")
        sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback)
        rospy.spin()
    except:
        print("Error occured.")
