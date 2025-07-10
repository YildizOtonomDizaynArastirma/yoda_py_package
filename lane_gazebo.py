#!/usr/bin/env python3
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.executors import ExternalShutdownException


class TemplateNode(Node):
    def __init__(self):
        super().__init__('batu_lane_node')

        self.subscription = self.create_subscription(Image,'/yoda/camera/image_raw',self.image_callback,10)
        self.curve_publisher = self.create_publisher(Float32, '/lane_curve', 10)
        self.offset_publisher = self.create_publisher(Float32, '/lane_offset', 10)
        self.publisher = self.create_publisher(Image,'/lane_detection/image_raw',10)

        self.bridge = CvBridge()
        self.get_logger().info('Lane Detection Başlatıldı...')
        self.last_left_lane_x = None
        self.last_left_lane_y = None
        self.last_right_lane_x = None
        self.last_right_lane_y = None
        
        cv2.namedWindow('Şerit Görüntüsü', cv2.WINDOW_NORMAL)
    ###
    
    
    ###İŞLENMİŞ GÖRÜNTÜYÜ BASTIRMA


    ###

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            warped_image,self.matrix = self.warp_image(cv_image)
            y_position = warped_image.shape[0] - 1
            
            #binary_color_segmentation = self.hsv(warped_image)
            #binary_sobel_thresholding = self.sobel_thresholding(warped_image)
            #binary_combined = np.zeros_like(cv2.cvtColor(warped_image, cv2.COLOR_BGR2GRAY))
            #binary_combined[(binary_sobel_thresholding == 255) | (binary_color_segmentation == 255)] = 255
            #binary_combined = cv2.dilate(binary_combined, np.ones((7, 7), np.uint8), iterations=1)
            #binary_combined = cv2.erode(binary_combined, np.ones((3,3), np.uint8), iterations=1)
            # Optional morphological operations
            #binary_combined = cv2.dilate(binary_combined, np.ones((7, 7), np.uint8), iterations=1)
            #binary_combined = cv2.erode(binary_combined, np.ones((3, 3), np.uint8), iterations=1)
            
            
            binary_combined=self.hsv(warped_image)
            
            
            #cv2.imshow('Binary',cv2.resize(binary_combined, (720,640)))
            
            
            left_lane_x, left_lane_y, right_lane_x, right_lane_y, window_output_image = self.lane_pixel_segmentation(binary_combined)
            scale_factor_y = 30 / 1080  # meters per pixel in y dimension
            scale_factor_x = 3.7 / 1920  # meters per pixel in x dimension
            left_curvature = 0
            right_curvature = 0
            left_lane = 0
            right_lane = 0
            if left_lane_x.size == 0 and self.last_left_lane_x is not None:
                left_lane_x = self.last_left_lane_x
                left_lane_y = self.last_left_lane_y
            else:
                self.last_left_lane_x = left_lane_x
                self.last_left_lane_y = left_lane_y

            if right_lane_x.size == 0 and self.last_right_lane_x is not None:
                right_lane_x = self.last_right_lane_x
                right_lane_y = self.last_right_lane_y
            else:
                self.last_right_lane_x = right_lane_x
                self.last_right_lane_y = right_lane_y
            #cv2.imshow('Window Output', cv2.resize(window_output_image, (720,640)))
            
            
            if left_lane_x.size > 0 or right_lane_x.size > 0 or left_lane_y.size > 0 or right_lane_y.size > 0:
            
                left_line_pts, A_l, B_l, C_l, right_line_pts, A_r, B_r, C_r = self.fit_lane_lines(binary_combined, left_lane_x,
                                                                                                    left_lane_y, right_lane_x, right_lane_y)
                left_curvature = self.calculate_curvature(A_l, B_l, y_position, scale_factor_y, scale_factor_x)
                right_curvature = self.calculate_curvature(A_r, B_r, y_position, scale_factor_y, scale_factor_x)
                lane_center_x = self.calculate_lanecenter(A_l, B_l, C_l, A_r, B_r, C_r, warped_image.shape)
                final_result_image = self.draw_lane_lines(cv_image, warped_image, left_line_pts, right_line_pts,lane_center_x)
                offset=self.calculate_offset(lane_center_x,binary_combined.shape[1],scale_factor_x)
                
                #print(f"Left Curvature:{left_curvature}")
                #print(f"Right Curvature:{right_curvature}")
                #print(f"Offset:{offset}")
                cv2.putText(cv_image,f"Orta Nokta ile Fark={offset:.2f}",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
                self.offset_publisher.publish(Float32(data=offset))
                
                
                if abs(left_curvature)>5000:
                    left_curvature = 0
                    cv2.putText(cv_image,"Sol serit duz.",(50,100),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                    left_lane = 0.0
                elif left_curvature > 0:
                    cv2.putText(cv_image, "Sol serit saga donuyor.", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    left_lane = 1.0
                elif left_curvature < 0:
                    cv2.putText(cv_image, "Sol serit sola donuyor.", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    left_lane = -1.0
                else:
                    pass
    
                if abs(right_curvature)>5000:      
                    right_curvature = 0
                    cv2.putText(cv_image,"Sag serit duz.",(50,150),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                    right_lane = 0.0
                elif right_curvature > 0:
                    cv2.putText(cv_image, "Sag serit saga donuyor.", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    right_lane = 1.0
                elif right_curvature < 0:
                    cv2.putText(cv_image, "Sag serit sola donuyor.", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    right_lane = -1.0
                else:
                    pass
                if left_lane == right_lane:
                    lane_radius=(left_curvature+right_curvature)/2
                    
                    self.curve_publisher.publish(Float32(data=lane_radius))
                
                
                
                
                
                lane_image = final_result_image.copy()
            
            else:
                lane_image = cv_image.copy()
            
            
            lane_image_msg = self.bridge.cv2_to_imgmsg(lane_image, encoding='bgr8')
            self.publisher.publish(lane_image_msg)
            cv2.imshow('Şerit Görüntüsü', cv2.resize(lane_image,(720,640)))
            if cv2.waitKey(1) & 0xFF == 27:
                self.get_logger().info('Düğüm Kapatılıyor...')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {e}')
    
        
    
    
    ###



    ###GÖRÜNTÜ İŞLEME



    ###
    
    
    def warp_image(self,inpImage):
        """
        h , w = inpImage.shape[:2]
    
        src = np.float32([(0,h),(w*0.3,h*0.3),(w*0.7,h*0.3),(w,h)])
        dst = np.float32([(0,h),(0,0),(w,0),(w,h)])

        matrix = cv2.getPerspectiveTransform(src, dst)
        birdseye = cv2.warpPerspective(inpImage, matrix, (w,h))
        cv2.imshow("Birdseye" , cv2.resize(birdseye, (720,640)))
        return birdseye, matrix
        """
        img_size = (inpImage.shape[1], inpImage.shape[0])
        bot_width = .35
        mid_width = .20
        height_pct = .35
        bottom_trim = .65
        
        src = np.float32([[inpImage.shape[1]*(.5-mid_width/2),inpImage.shape[0]*height_pct],
                         [inpImage.shape[1]*(.5+mid_width/2),inpImage.shape[0]*height_pct],
                         [inpImage.shape[1]*(.5+bot_width/2),inpImage.shape[0]*bottom_trim],
                         [inpImage.shape[1]*(.5-bot_width/2),inpImage.shape[0]*bottom_trim]])
        offset = img_size[0]*.25
        dst = np.float32([[offset, 0], 
                        [img_size[0]-offset, 0], 
                        [img_size[0]-offset, img_size[1]],
                        [offset, img_size[1]]])
        
        matrix = cv2.getPerspectiveTransform(src, dst)
        birdseye = cv2.warpPerspective(inpImage, matrix, img_size)
        #cv2.imshow("Birdseye" , cv2.resize(birdseye, (720,640)))
        return birdseye, matrix
        
    
    def hsv(self,image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
        # Define the range for white color in HSV
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 25, 255], dtype=np.uint8)
    
        # Create a binary mask where white colors are in the range
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return mask
    
    def sobel_thresholding(self,image):
        """
        This function uses absolute_sobel_thresholding(), sobel_gradMag_thresholding(),
        sobel_gradDir_thresholding() functions to threshold the input image, combines the results
        obtained from the three functions and returns a binary image.
        """

        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # PARAMETER EXPERIMENTATION NOTE -
        # On OpenCV website it is recommended to use Scharr filter to improve results. To use Scharr filter
        # we pass (ksize = -1) as parameter. Scharr filter is said to give better results than 3x3 Sobel filter
        # Scharr filter(G_x) = [-3,0,3; -10,0,10; -3,0,3];  Sobel filter (G_x) = [-1,0,1; -2,0,2; -1,0,1]
        # A 3x3 Sobel operator may produce inaccuracies since it's an approximation of derivative
        #
        # NOTE: For lane detection a 3x3 Sobel filter produces better results than Scharr filter

        # Computing absolute sobel thresholded image
        sobel_absolute_binary = self.absolute_sobel_thresholding(image, threshold=(30, 100))
        # cv2.namedWindow("Absolute Sobel Result", cv2.WINDOW_NORMAL)
        # cv2.imshow("Absolute Sobel Result", sobel_absolute_binary)

        # Computing sobel gradient magnitude thresholded image
        sobel_gradMag_binary =self.sobel_gradMag_thresholding(image, threshold=(70, 120))
        # cv2.namedWindow("Sobel Gradient Magnitude Result", cv2.WINDOW_NORMAL)
        # cv2.imshow("Sobel Gradient Magnitude Result", sobel_gradMag_binary)

        # Computing sobel gradient direction thresholded image
        sobel_gradDir_binary = self.sobel_gradDir_thresholding(image, threshold=(40*np.pi/180, 70*np.pi/180))
        # cv2.namedWindow("Sobel Gradient Direction Result", cv2.WINDOW_NORMAL)
        # cv2.imshow("Sobel Gradient Direction Result", sobel_gradDir_binary)

        # Combining all the results
        combined_sobel_binary = np.zeros_like(image)
        combined_sobel_binary[(sobel_absolute_binary == 255) |
                          ((sobel_gradMag_binary == 255) & (sobel_gradDir_binary == 255))] = 255

        return combined_sobel_binary


    def absolute_sobel_thresholding(self,image, threshold=(30, 100)):
        """
        This function takes in an image, applies Sobel derivative using x kernel, performs thresholding
        and returns a binary image. Best results have been found with threshold values of 30 and 100
        """
        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        absolute_sobel = np.absolute(cv2.Sobel(image, cv2.CV_64F, 1, 0))
        scaled_absolute_sobel = np.uint8(255 * absolute_sobel / np.max(absolute_sobel))
        binary_result_image = np.zeros_like(scaled_absolute_sobel)
        binary_result_image[(absolute_sobel >= threshold[0]) & (absolute_sobel <= threshold[1])] = 255

        return binary_result_image


    def sobel_gradMag_thresholding(self,image, threshold=(30,100)):
        """
        This function takes in an image, computes sobel derivatives in X and Y direction and uses them
        to compute gradient magnitude, performs thresholding and returns a binary image.
        """
        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)

        gradient_magnitude = np.sqrt(sobelx ** 2 + sobely ** 2)
        scaled_gradient_magnitude = np.uint8(255 * gradient_magnitude / np.max(gradient_magnitude))
        binary_result_image = np.zeros_like(gradient_magnitude)
        binary_result_image[(scaled_gradient_magnitude >= threshold[0]) & (scaled_gradient_magnitude <= threshold[1])] = 255

        return binary_result_image


    def sobel_gradDir_thresholding(self,image, threshold=(30*np.pi/180, 70*np.pi/2)):
        """
        This function takes in an image, computes sobel derivatives in X and Y direction and uses them
        to compute gradient direction, performs thresholding and returns a binary image.
        """
        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)

        # Using absolute since gradient can be both positive and negative and we only care about magnitude
        absolute_gradient_direction = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
        binary_result_image = np.zeros_like(absolute_gradient_direction)
        binary_result_image[
            (absolute_gradient_direction >= threshold[0]) & (absolute_gradient_direction <= threshold[1])] = 255

        return binary_result_image

    
    
    
    
    
    
    ###
    
    
    ###ŞERİTLERİ BULMA
    
    
    ###
   
    
    
    
    def lane_line_polyfit(self,y_coordinates, lane_pixels_y, lane_pixels_x):
        lane_line_parameters = np.polyfit(lane_pixels_y, lane_pixels_x, 2)
        lane_line_x_coordinates = lane_line_parameters[0]*y_coordinates**2 + \
                                lane_line_parameters[1]*y_coordinates + \
                                lane_line_parameters[2]

        pts = np.array(np.transpose(np.vstack([lane_line_x_coordinates, y_coordinates])), np.int32)
        pts = pts.reshape(-1, 1, 2)

        return pts, lane_line_parameters[0], lane_line_parameters[1], lane_line_parameters[2]
    
    def fit_lane_lines(self, image, left_lane_x, left_lane_y, right_lane_x, right_lane_y):
        
        y_coordinates = np.linspace(0, image.shape[0] - 1, image.shape[0])
        if len(left_lane_x) == 0 or len(left_lane_y) == 0:
            pts_left, a_left, b_left, c_left = [], 0, 0, 0
        else:
            pts_left, a_left, b_left, c_left = self.lane_line_polyfit(y_coordinates, left_lane_y, left_lane_x)
        
        if len(right_lane_x) == 0 or len(right_lane_y) == 0:
            pts_right, a_right, b_right, c_right = [], 0, 0, 0
        else:
            pts_right, a_right, b_right, c_right = self.lane_line_polyfit(y_coordinates, right_lane_y, right_lane_x)
        
        return pts_left, a_left, b_left, c_left, pts_right, a_right, b_right, c_right
    
    def draw_lane_lines(self,img, binary_warped_image, left_pts, right_pts,lane_center_x):
       
        lane_lines = np.zeros((binary_warped_image.shape[0], binary_warped_image.shape[1], 3), dtype='uint8')
        # Drawing the lane lines on the colored warped image
        if len(left_pts) == 0 and len(right_pts) == 0:
            pass
        elif len(left_pts) == 0:
            cv2.polylines(lane_lines, [right_pts], isClosed=False, color=(0, 0, 255), thickness=20)
        elif len(right_pts) == 0:
            cv2.polylines(lane_lines, [left_pts], isClosed=False, color=(0, 0, 255), thickness=20)
        else:
            cv2.polylines(lane_lines, [left_pts], isClosed=False, color=(0, 0, 255), thickness=20)
            cv2.polylines(lane_lines, [right_pts], isClosed=False, color=(0, 0, 255), thickness=20)
            lane_lines = cv2.circle(lane_lines, (int(lane_center_x), int(binary_warped_image.shape[0]*0.3)), 20, (0, 255, 0), -1)
            
            

        # Warping warped image back to original image
        warped_back_image = cv2.warpPerspective(lane_lines, np.linalg.inv(self.matrix), (img.shape[1], img.shape[0]))

        # Finding all the lane pixels in the warped back image which are non-zero in the red channel
        lane_pixels = (warped_back_image[:, :, 2]).nonzero()
        lane_center = (warped_back_image[:,:,1]).nonzero()
        
        # Making the identified lane pixels in the original image as red
        
        
        img[np.array(lane_center[0]), np.array(lane_center[1])] = (0, 255, 0)
        img[np.array(lane_pixels[0]), np.array(lane_pixels[1])] = (0, 0, 255)
        
        return img
    
    def calculate_curvature(self,A, B, y_position, scale_factor_y, scale_factor_x):
        
        if A == 0:
            return 0
        y_position_m = y_position * scale_factor_y
        curvature = ((1 + (2 * A * y_position_m + B) ** 2) ** (3 / 2)) / (2 * A)
        return curvature

    def calculate_lanecenter(self,A_l, B_l, C_l, A_r, B_r, C_r, image_shape):
       
        y_eval = image_shape[0]*0.3
        left_lane_bottom = A_l * y_eval**2 + B_l * y_eval + C_l
        right_lane_bottom = A_r * y_eval**2 + B_r * y_eval + C_r
        lane_center_x = (left_lane_bottom + right_lane_bottom) / 2
        return lane_center_x
    
    def calculate_offset(self,lane_center,image_width,scale_factor_x):
        
        img_center = image_width/2
        offset = (lane_center-img_center)*scale_factor_x
        return offset    
    
    def lane_pixel_segmentation(self, image):
       
        # Computing number of white pixels(value = 255) in each column. Basically a histogram
        histogram = np.sum(image, axis=0)
        histogram_midpoint = np.int32(np.shape(histogram)[0]*0.5)
        
        
        
        
        # Sliding window parameters
        number_of_windows = 9
        window_height = np.int32(image.shape[0] / number_of_windows)

        # Identifying x and y position of all non zero pixels
        non_zero_pixels = image.nonzero()
        non_zero_y = np.array(non_zero_pixels[0])
        non_zero_x = np.array(non_zero_pixels[1])

        # Computing X and y coordinates of midpoint of the current window and setting them to the
        # coordinates of midpoint for the bottom most windows on left and right side
        current_window_leftx = np.argmax(histogram[0:histogram_midpoint])
        current_window_rightx = np.argmax(histogram[histogram_midpoint:-1]) + histogram_midpoint

        # Width of the windows. Here width refers to size of window from the midpoint of the window
        window_width = np.int32(image.shape[1] * 0.08)

        # Initializing arrays to store the indices of pixels on left and right side
        left_lane_pixel_indices = []
        right_lane_pixel_indices = []

        # Initializing an image to display output of this function
        window_output_image = np.dstack((image, image, image)).astype('uint8')

        # Stepping through windows one by one. Building them from bottom up
        for window_number in range(number_of_windows):
            # Computing vertices of left and right window
            window_bottom_y = image.shape[0] - window_number * window_height
            window_top_y = image.shape[0] - (window_number + 1) * window_height
            left_window_leftx = current_window_leftx - window_width
            left_window_rightx = current_window_leftx + window_width
            right_window_leftx = current_window_rightx - window_width
            right_window_rightx = current_window_rightx + window_width

            # Drawing windows in the output image
            cv2.rectangle(window_output_image, (left_window_leftx, window_bottom_y),
                        (left_window_rightx, window_top_y), (0, 255, 0), 2)
            cv2.rectangle(window_output_image, (right_window_leftx, window_bottom_y),
                        (right_window_rightx, window_top_y), (0, 255, 0), 2)

            # Identifying non-zero pixels which are inside to left and right window
            left_window_pixels_array_location = ((non_zero_y >= window_top_y) &
                                                (non_zero_y <= window_bottom_y) &
                                                 (non_zero_x >= left_window_leftx) &
                                                (non_zero_x <= left_window_rightx)).nonzero()[0]
            right_window_pixels_array_location = ((non_zero_y >= window_top_y) &
                                                 (non_zero_y <= window_bottom_y) &
                                                  (non_zero_x >= right_window_leftx) &
                                                  (non_zero_x <= right_window_rightx)).nonzero()[0]

            # Appending
            left_lane_pixel_indices.append(left_window_pixels_array_location)
            right_lane_pixel_indices.append(right_window_pixels_array_location)

            # Recentering the next window on the mean position of the pixels of the current window
            # If condition checks for if there are pixels in the window
            top_five_percent_left = ((non_zero_y >= window_top_y) &
                                     (non_zero_y <= window_top_y + 0.2 * window_height) &
                                     (non_zero_x >= left_window_leftx) &
                                     (non_zero_x <= left_window_rightx)).nonzero()[0]
            if top_five_percent_left.size:
                # current_window_leftx = np.int(np.median(non_zero_x[left_window_pixels_array_location]))
                current_window_leftx = np.int32(np.median(non_zero_x[top_five_percent_left]))

            top_five_percent_right = ((non_zero_y >= window_top_y) &
                                      (non_zero_y <= window_top_y + 0.2 * window_height) &
                                      (non_zero_x >= right_window_leftx) &
                                      (non_zero_x <= right_window_rightx)).nonzero()[0]
            if top_five_percent_right.size:
                # current_window_rightx = np.int(np.median(non_zero_x[right_window_pixels_array_location]))
                current_window_rightx = np.int32(np.median(non_zero_x[top_five_percent_right]))

            # Checking if the previous windows were at the correct position
            if window_number > 0:
                previous_window_number = window_number - 1
                previous_window_leftx = current_window_leftx
                previous_window_rightx = current_window_rightx
                while previous_window_number >= 1:
                    previous_window_bottom_y = image.shape[0] - (previous_window_number) * window_height
                    previous_window_top_y = window_bottom_y
                    previous_left_window_leftx = previous_window_leftx - window_width
                    previous_left_window_rightx = previous_window_leftx + window_width
                    previous_right_window_leftx = previous_window_rightx - window_width
                    previous_right_window_rightx = previous_window_rightx + window_width

                    previous_left_window_pixels_array_location = ((non_zero_y >= previous_window_top_y) &
                                                     (non_zero_y <= previous_window_bottom_y) &
                                                     (non_zero_x >= previous_left_window_leftx) &
                                                     (non_zero_x <= previous_left_window_rightx)).nonzero()[0]

                    previous_right_window_pixels_array_location = ((non_zero_y >= previous_window_top_y) &
                                                      (non_zero_y <= previous_window_bottom_y) &
                                                      (non_zero_x >= previous_right_window_leftx) &
                                                      (non_zero_x <= previous_right_window_rightx)).nonzero()[0]

                    change_in_left = (np.shape(previous_left_window_pixels_array_location)[0] - np.shape(left_lane_pixel_indices[previous_window_number])[0])
                    if change_in_left > 20:
                        left_lane_pixel_indices[previous_window_number] = np.append(left_lane_pixel_indices[previous_window_number], previous_left_window_pixels_array_location)
                        previous_window_leftx = np.int32(np.median(non_zero_x[previous_left_window_pixels_array_location]))
                    change_in_right = (np.shape(previous_right_window_pixels_array_location)[0] - np.shape(right_lane_pixel_indices[previous_window_number])[0])
                    if change_in_right > 20:
                        right_lane_pixel_indices[previous_window_number] = np.append(right_lane_pixel_indices[previous_window_number], previous_right_window_pixels_array_location)
                        previous_window_rightx = np.int32(np.median(non_zero_x[previous_right_window_pixels_array_location]))

                    if change_in_left <= 20 and change_in_right <= 20:
                        break
                    previous_window_number = previous_window_number - 1

        # Concatenating arrays because after appending in each loop
        # the left_lane_pixel_indices and right_lane_pixel_indices
        # are in form of nested lists or a 2-d array
        left_lane_pixel_indices = np.concatenate(left_lane_pixel_indices)
        right_lane_pixel_indices = np.concatenate(right_lane_pixel_indices)

        # Extracting the x and y coordinates of pixels for left and right lanes
        left_lane_x = non_zero_x[left_lane_pixel_indices]
        left_lane_y = non_zero_y[left_lane_pixel_indices]
        right_lane_x = non_zero_x[right_lane_pixel_indices]
        right_lane_y = non_zero_y[right_lane_pixel_indices]

        # In output image changing color of pixels in left lane to blue
        # and pixels in right lane to red
        #window_output_image[non_zero_y[left_lane_pixel_indices], non_zero_x[left_lane_pixel_indices]] = [255, 0, 0]
        #window_output_image[non_zero_y[right_lane_pixel_indices], non_zero_x[right_lane_pixel_indices]] = [0, 0, 255]

        # Displaying results of Sliding Window and Segmenting pixels into left and right lane

        #cv2.imshow("Sliding Window" , cv2.resize(window_output_image, (720,640)))
        return left_lane_x, left_lane_y, right_lane_x, right_lane_y, window_output_image
def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # normal shutdown requested
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()