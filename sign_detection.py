import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import torch

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from vision_msgs.msg import Pose2D, Point2D  # ✅ Doğru tipler

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')
        self.bridge = CvBridge()

        # ✅ YOLOv8 modelini CPU üzerinde yükle
        self.model = YOLO('/home/safa/yoda_ws/src/yoda_py_package/yoda_py_package/best.pt')
        self.model.to('cpu')
        self.model.fuse()

        # ✅ v4l2 kamera görüntü aboneliği
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # ✅ Tespit yayıncısı
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10)

        self.get_logger().info('✅ YOLOv8 Sign Detection Node Başlatıldı.')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge dönüşüm hatası: {e}")
            return

        # ✅ YOLO ile tahmin
        results = self.model(frame, verbose=False)[0]

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in results.boxes:
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            label = self.model.names[cls_id]

            detection = Detection2D()

            # ✅ vision_msgs/Pose2D ve Point2D kullan
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

            # ✅ Görüntü üzerine kutu ve etiket çiz
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 50, 50), 2)

        # ✅ Yayınla
        self.detection_pub.publish(detection_array)

        # ✅ Görsel olarak göster
        cv2.imshow("YOLOv8 Detections", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

