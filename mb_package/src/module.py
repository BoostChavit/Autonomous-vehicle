#!/usr/bin/env python
import cv2
import numpy as np
from collections import deque


# PID controller
class PID:
    def __init__(self, Kp, Ki, Kd, max_output, min_output):
        self.Kp = Kp    # steering angle
        self.Ki = Ki    # check whether the car is staying too long on one side
        self.Kd = Kd    # pull back of steering angle
        self.max_output = max_output
        self.min_output = min_output
        self.prev_error = 0
        self.integral = deque(maxlen=100)
    def isBufferFull(self):
        if len(self.integral) == 100:
            return True
        return False

    def update(self, error):
        self.integral.append(error)
        derivative = error - self.prev_error
        output = (self.Kp * error) + (self.Ki * sum(self.integral) / len(self.integral)) + (self.Kd * derivative)
        output = max(min(output, self.max_output), self.min_output)  # Clamp the output
        self.prev_error = error
        return output



# TrafficLightSmoother Class
class TrafficLightSmoother:
    def __init__(self):
        self.state = 1 # 0 Red, 1 Green
        self.lane = None # 0 Left, 1 Right

    def detect_circles(self, gray):
        weight, height = gray.shape[:2]
        half_gray = gray[0:height // 2, :]
        _, thresh = cv2.threshold(half_gray, 120, 255, cv2.THRESH_BINARY)
        blurred = cv2.GaussianBlur(thresh, (9, 9), 2)

    	circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=100,
            param2=60,
            minRadius=15,
            maxRadius=30
        )
    
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

        return circles
   
    def setState(self, state):
        self.state = state

    def setLane(self, lane):
        self.lane = lane

    def check_traffic_light_pattern(self, circles):
        if circles is None or len(circles) <= 2:
            return False

        if len(circles) >= 4:
            self.state = 0
            return False
        
        circles = sorted(circles, key=lambda c: c[1]) 
        distances = [circles[i+1][0] - circles[i][0] for i in range(len(circles) - 1)]
    
        if all(abs(distances[i] - distances[0]) < 10 for i in range(len(distances))):
            return True

        return False


    def classify_traffic_light_color_hsvAndLab(self, image, circles):
        # Convert image to HSV and LAB
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

        # Define color ranges for HSV and LAB
        lower_red_hsv, upper_red_hsv = np.array([0, 50, 50]), np.array([10, 255, 255])
        lower_red_lab, upper_red_lab = np.array([20, 150, 150]), np.array([255, 255, 255])

        lower_green_hsv, upper_green_hsv = np.array([40, 40, 40]), np.array([80, 255, 255])
        lower_green_lab, upper_green_lab = np.array([0, 100, 100]), np.array([255, 150, 150])


        if circles is not None:
            circles = sorted(circles, key=lambda c: c[0])
            x_min_circle = circles[0][0]
            x_max_circle = circles[-1][0]
            for (x, y, r) in circles:
                r = int(r*1)
		roi = image[y-r:y+r, x-r:x+r]

                # Create a mask of the same size as the image
		masked_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		masked_lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

            	# Create masks for red and green in both HSV and LAB and Combine masks
                red_mask = cv2.inRange(masked_hsv, lower_red_hsv, upper_red_hsv) & cv2.inRange(masked_lab, lower_red_lab, upper_red_lab)
		green_mask = cv2.inRange(masked_hsv, lower_green_hsv, upper_green_hsv) & cv2.inRange(masked_lab, lower_green_lab, upper_green_lab)

            	# Calculate color areas within the circle
            	red_area = np.sum(red_mask > 0)
            	green_area = np.sum(green_mask > 0)

            	# Classify color based on the area thresholds	
            	if red_area > green_area and red_area > 50:  # Adjust threshold if needed
                    detected_color = "Red"
                    self.setState(0)
                    border_color = (0, 0, 255)
            	elif green_area > red_area and green_area > 50:
                    detected_color = "Green"
                    self.setState(1)
                    border_color = (0, 255, 0)
                    if len(circles) == 4:
                        if x == x_min_circle:
                            self.setLane(0)
                        elif x == x_max_circle:
                            self.setLane(1)
                       
            	else:
                    detected_color = "Unknown"
                    border_color = (255, 0, 0)

            	# Draw circle with detected color border (optional for visualization)
            	cv2.circle(image, (x, y), int(r), border_color, 2)
            	cv2.putText(image, detected_color, (x - r, y - r), cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)

