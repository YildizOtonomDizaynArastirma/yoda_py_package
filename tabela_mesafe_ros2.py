import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from vision_msgs.msg import Pose2D, Point2D


class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')
        self.bridge = CvBridge()
        self.camera_info_received=False
        self.model = YOLO('/home/yoda/yoda_ws/src/yoda_py_package/yoda_py_package/mixv4.pt')
        self.model.to('cpu')
        self.model.fuse()

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb_raw/image_raw_color',
            self.image_callback,
            10)

        self.depth_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/depth/depth_registered',
            self.depth_callback,
            10)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb_raw/camera_info',
            self.camera_info_callback,
            10)

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10)

        self.latest_depth_frame = None
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info('✅ YOLOv8 Sign Detection Node Başlatıldı.')

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.get_logger().info("📸 Kamera iç parametreleri alındı.")
            self.camera_info_received = True
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"⚠️ Derinlik görüntüsü dönüşüm hatası: {e}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"⚠️ RGB dönüşüm hatası: {e}")
            return

        results = self.model(frame, verbose=False)[0]
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in results.boxes:
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            label = self.model.names[cls_id]

            center_x = int((x1 + x2) / 2.0)
            center_y = int((y1 + y2) / 2.0)

            Z = None
            X = Y = None

            # Derinlik al
            if self.latest_depth_frame is not None:
                h, w = self.latest_depth_frame.shape[:2]
                if 0 <= center_x < w and 0 <= center_y < h:
                    Z = float(self.latest_depth_frame[center_y, center_x])
                    if np.isfinite(Z) and self.fx is not None:
                        X = (center_x - self.cx) * Z / self.fx
                        Y = (center_y - self.cy) * Z / self.fy
                        self.get_logger().info(f"🧭 {label} | Conf: {conf:.2f} | Pozisyon: ({X:.2f}, {Y:.2f}, {Z:.2f})")
                    else:
                        self.get_logger().warn("⚠️ Geçersiz derinlik veya intrinsics.")
                else:
                    self.get_logger().warn("⚠️ Piksel derinlik görüntüsü dışında.")
            else:
                self.get_logger().warn("⚠️ Henüz derinlik verisi yok.")

            # Mesaj oluştur
            detection = Detection2D()
            center_pose = Pose2D()
            center_pose.position = Point2D()
            center_pose.position.x = (x1 + x2) / 2.0
            center_pose.position.y = (y1 + y2) / 2.0
            center_pose.theta = 0.0

            detection.bbox = BoundingBox2D()
            detection.bbox.center = center_pose
            detection.bbox.size_x = abs(x2 - x1)
            detection.bbox.size_y = abs(y2 - y1)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = label
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

            # Görsel üzerine çiz
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if Z is not None and np.isfinite(Z):
                cv2.putText(frame, f"Z: {Z:.2f}m", (int(x1), int(y2) + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 1)
                if X is not None and Y is not None:
                    cv2.putText(frame, f"3D: [{X:.2f}, {Y:.2f}, {Z:.2f}]m", (int(x1), int(y2) + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 200), 1)

        self.detection_pub.publish(detection_array)
        cv2.imshow("YOLOv8 Tespit ve Derinlik", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
