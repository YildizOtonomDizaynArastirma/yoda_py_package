#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('LaneDetection_Node')
        
        # Piksel başına metre cinsinden ölçeklendirme
        self.ym_per_pix = 30 / 1080 
        self.xm_per_pix = 3.7 / 1920 
        # Görüntü Boyutları
        self.image_width = 1920 
        self.image_height = 1080  
        self.bridge = CvBridge()
        self.twist = Twist()
        
        # Publisher ve Subscriber tanımlamaları
        # Publisher eklenmesi
        self.lane_status_publisher = self.create_publisher(Bool, '/yoda/lane_status', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(Image, '/yoda/camera/image_raw', self.serit_callback, 10)
        
        # PID ve LowPass Filter parametreleri
        self.pid_controller = PIDController(0.1, 0.00085, 0.1, 10)
        self.low_pass_filter = LowPassFilter(alpha=0.5)
        
        self.previous_data = None
        self.final_image = None
        self.camera_center_x, self.camera_center_y = self.find_camera_center(self.image_width, self.image_height)

    def find_camera_center(self, image_width, image_height):
        camera_center_x = image_width / 2
        camera_center_y = image_height / 2
        return camera_center_x, camera_center_y

    def processImage(self, inpImage):
        hls = cv2.cvtColor(inpImage, cv2.COLOR_BGR2HLS) 
        lower_white = np.array([0, 200, 0])
        upper_white = np.array([180, 255, 255])
        mask = cv2.inRange(hls, lower_white, upper_white) 
        hls_result = cv2.bitwise_and(inpImage, inpImage, mask = mask) 
        gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY) 
        ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur(thresh,(5, 5), 0)
        return thresh, blur

    def perspectiveWarp(self, inpImage):
        img_size = (inpImage.shape[1], inpImage.shape[0])
        bot_width = .35
        mid_width = .25
        height_pct = .35
        bottom_trim = .50
        
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
        minv = cv2.getPerspectiveTransform(dst, src)
        birdseye = cv2.warpPerspective(inpImage, matrix, img_size)
        cv2.imshow("Birdseye" , cv2.resize(birdseye, (720,640)))
        return birdseye, minv

    def plotHistogram(self, inpImage):
        histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis = 0)
        return histogram

    def slide_window_search(self, binary_warped, histogram):
        # [Önceki slide_window_search fonksiyonunun içeriği aynı kalacak]
        midpoint = histogram.shape[0] // 2
        window_height = binary_warped.shape[0] // 9
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = np.argmax(histogram[:midpoint])
        rightx_current = np.argmax(histogram[midpoint:]) + midpoint
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []
        
        for window in range(9):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                             (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        
        return left_fit, right_fit

    def general_search(self, binary_warped, left_fit, right_fit):
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
        left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
        left_fit[1]*nonzeroy + left_fit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
        right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
        right_fit[1]*nonzeroy + right_fit[2] + margin)))
        
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
        
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        
        ret = {}
        ret['leftx'] = leftx
        ret['rightx'] = rightx
        ret['left_fitx'] = left_fitx
        ret['right_fitx'] = right_fitx
        ret['ploty'] = ploty
        
        return ret

    def draw_lane_lines(self, original_image, warped_image, Minv, draw_info):
        self.final_image = np.copy(original_image)
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']

        self.pid_controller.reset()
        self.low_pass_filter.reset()
        
        warp_zero = np.zeros_like(warped_image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        midpoint = original_image.shape[1] / 2
        lane_width = 400
        left_limit = max(midpoint - lane_width, 0)
        right_limit = min(midpoint + lane_width, original_image.shape[1])

        left_fitx = np.clip(left_fitx, left_limit, right_limit)
        right_fitx = np.clip(right_fitx, left_limit, right_limit)

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        
        mean_x = np.mean((left_fitx, right_fitx), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
        pts_mean_x = pts_mean[0][0][0]
        
        image_center_x = original_image.shape[1] / 2
        max_linear_speed = 0.05
        max_angular_speed = 0.5
        
        deviation = pts_mean_x - image_center_x
        control_signal = self.pid_controller.compute(deviation)
        filtered_control_signal = self.low_pass_filter.apply(control_signal)
        
        speed = Twist()
        if filtered_control_signal < -1:
            direction = "Left"
            amount = abs(filtered_control_signal) / 4
            if 50 > abs(filtered_control_signal) > 6:
                amount *= 2.5
            speed.angular.z = min(amount / 5, max_angular_speed)
            speed.linear.x = max_linear_speed
        elif filtered_control_signal > 1:
            direction = "Right"
            amount = abs(filtered_control_signal) / 20
            if 50 > abs(filtered_control_signal) > 22:
                amount *= 2
            speed.angular.z = -min(amount / 10, max_angular_speed)
            speed.linear.x = max_linear_speed
        else:
            direction = "Straight"
            speed.angular.z = 0
            speed.linear.x = max_linear_speed

        self.get_logger().info(f"Filtered Control Signal: {filtered_control_signal}")
        self.get_logger().info(f"Direction: {direction}")

        #self.publisher.publish(speed)

        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))
        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)
        return result

    def serit_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("orijinal", cv2.resize(frame, (720, 640)))
            birdView, minverse = self.perspectiveWarp(frame)
            thresh, blur = self.processImage(birdView)
            hist = self.plotHistogram(thresh)
            left_fit, right_fit = self.slide_window_search(thresh, hist)
            draw_info = self.general_search(thresh, left_fit, right_fit)
            self.final_image = np.copy(frame)
            result = self.draw_lane_lines(frame, thresh, minverse, draw_info)
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 200, 150)
            height, width = edges.shape
            roi_vertices = [(0, height), (width / 2, height / 2), (width, height)]
            mask = np.zeros_like(edges)
            cv2.fillPoly(mask, np.array([roi_vertices], dtype=np.int32), 255)
            masked_edges = cv2.bitwise_and(edges, mask)
            
            left_lines = cv2.HoughLinesP(masked_edges[:, :width // 2], rho=2, theta=np.pi / 180,
                                       threshold=50, minLineLength=10, maxLineGap=90)
            right_lines = cv2.HoughLinesP(masked_edges[:, width // 2:], rho=2, theta=np.pi / 180,
                                        threshold=50, minLineLength=10, maxLineGap=90)
            
            line_image = np.zeros_like(frame)
            if left_lines is not None:
                for line in left_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 40)
            if right_lines is not None:
                for line in right_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_image, (x1 + width // 2, y1), (x2 + width // 2, y2), (0, 255, 0), 40)

            self.final_image = cv2.addWeighted(result, 0.8, line_image, 1, 0)
            cv2.imshow('Final Image', cv2.resize(self.final_image, (720, 640)))
            cv2.waitKey(1)
            self.previous_data = data
            # Şerit varsa True, yoksa False olarak yayınla
            lane_detected = bool(left_lines or right_lines)  # Şerit algı durumu
            self.lane_status_publisher.publish(Bool(data=lane_detected))
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            if self.previous_data is not None:
                self.serit_callback(self.previous_data)
            else:
                self.get_logger().error('Error occurred but no previous data found')

class PIDController:
    def __init__(self, Kp, Ki, Kd, integral_limit=float('inf')):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = integral_limit

    def reset(self):
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class LowPassFilter:
    def __init__(self, alpha, initial_value=0):
        self.alpha = alpha
        self.prev_value = initial_value

    def reset(self, initial_value=0):
        self.prev_value = initial_value

    def apply(self, new_value):
        filtered_value = self.alpha * new_value + (1 - self.alpha) * self.prev_value
        self.prev_value = filtered_value
        return filtered_value

def main(args=None):
    rclpy.init(args=args)
    LaneDetection_Node = LaneDetectionNode()
    rclpy.spin(LaneDetection_Node)
    LaneDetection_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
