import csv
import os
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class DetectorRecorderNode(Node):
    def __init__(self):
        super().__init__('detector_recorder_node')

        self.declare_parameter('model_path', '/ws/models/yolo11n.pt')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('infer_rate', 3.0)
        self.declare_parameter('target_classes', ['person'])
        self.declare_parameter('cooldown_sec', 30.0)
        self.declare_parameter('record_root', '/ws/records')

        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.image_size = self.get_parameter('image_size').value
        infer_rate = self.get_parameter('infer_rate').value
        self.target_classes = set(self.get_parameter('target_classes').value)
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        record_root = self.get_parameter('record_root').value

        self.get_logger().info(f'YOLO モデルを読み込みます: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.frame = None
        self.waypoint = '-'
        self.robot_xy = (0.0, 0.0)
        self.last_saved = {}     # (地点, クラス) → 最後に記録した時刻
        self.count = 0

        # 記録用フォルダ（起動ごとに新しく作る）
        session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_dir = os.path.join(record_root, session)
        os.makedirs(os.path.join(self.out_dir, 'images'), exist_ok=True)
        self.csv_path = os.path.join(self.out_dir, 'detections.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['sim_time', 'waypoint', 'robot_x', 'robot_y', 'class', 'confidence', 'image'])

        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(String, '/patrol/current_waypoint', self.on_waypoint, 10)
        self.image_pub = self.create_publisher(Image, '/detections/image', 10)
        self.create_timer(1.0 / infer_rate, self.on_timer)

        self.get_logger().info(f'記録先: {self.out_dir}')

    def on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def on_waypoint(self, msg):
        self.waypoint = msg.data

    def on_timer(self):
        if self.frame is None:
            return
        frame, self.frame = self.frame, None   # 同じ画像を二度推論しない

        result = self.model.predict(
            frame, imgsz=self.image_size, conf=self.confidence, device='cpu', verbose=False)[0]
        annotated = result.plot()   # 枠とラベルを描いた画像（BGR）

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(out_msg)

        now = self.get_clock().now().nanoseconds / 1e9
        for box in result.boxes:
            name = result.names[int(box.cls[0])]
            conf = float(box.conf[0])
            if name not in self.target_classes:
                continue
            key = (self.waypoint, name)
            if now - self.last_saved.get(key, -1e9) < self.cooldown_sec:
                continue
            self.last_saved[key] = now
            self.save_record(now, name, conf, annotated)

    def save_record(self, sim_time, name, conf, image):
        self.count += 1
        filename = f'{self.count:04d}_{self.waypoint}_{name}.jpg'
        cv2.imwrite(os.path.join(self.out_dir, 'images', filename), image)
        x, y = self.robot_xy
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(
                [f'{sim_time:.1f}', self.waypoint, f'{x:.2f}', f'{y:.2f}', name, f'{conf:.2f}', filename])
        self.get_logger().info(f'[検出] {self.waypoint}: {name} ({conf:.2f}) を記録しました')


def main(args=None):
    rclpy.init(args=args)
    node = DetectorRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
