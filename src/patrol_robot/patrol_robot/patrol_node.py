import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def yaw_from_quaternion(q):
    """クォータニオンから yaw（z 軸まわりの角度）を求める"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(angle):
    """角度を -π〜π の範囲に収める"""
    return math.atan2(math.sin(angle), math.cos(angle))


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('waypoint_names', ['WP1'])
        self.declare_parameter('waypoint_x', [0.0])
        self.declare_parameter('waypoint_y', [0.0])
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('scan_time', 8.0)
        self.declare_parameter('scan_speed', 0.8)

        names = self.get_parameter('waypoint_names').value
        xs = self.get_parameter('waypoint_x').value
        ys = self.get_parameter('waypoint_y').value
        self.waypoints = list(zip(names, xs, ys))
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.scan_time = self.get_parameter('scan_time').value
        self.scan_speed = self.get_parameter('scan_speed').value

        self.pose = None          # (x, y, yaw)
        self.index = 0            # 今目指している地点の番号
        self.state = 'MOVE'
        self.scan_start = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wp_pub = self.create_publisher(String, '/patrol/current_waypoint', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.on_timer)   # 10Hz で制御

        self.get_logger().info(f'巡回を開始します（{len(self.waypoints)} 地点）')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_timer(self):
        if self.pose is None:
            return   # オドメトリがまだ届いていない

        name, goal_x, goal_y = self.waypoints[self.index]
        self.wp_pub.publish(String(data=name))
        x, y, yaw = self.pose
        cmd = Twist()

        if self.state == 'MOVE':
            dx, dy = goal_x - x, goal_y - y
            distance = math.hypot(dx, dy)
            if distance < self.goal_tolerance:
                self.state = 'SCAN'
                self.scan_start = self.get_clock().now()
                self.get_logger().info(f'{name} に到着しました。周囲を点検します')
            else:
                angle_error = normalize_angle(math.atan2(dy, dx) - yaw)
                # 角度差に比例して旋回（上限あり）
                cmd.angular.z = max(-self.angular_speed,
                                    min(self.angular_speed, 1.5 * angle_error))
                # ほぼ目標の方向を向いているときだけ前進
                if abs(angle_error) < 0.3:
                    cmd.linear.x = min(self.linear_speed, 0.5 * distance + 0.05)

        elif self.state == 'SCAN':
            elapsed = (self.get_clock().now() - self.scan_start).nanoseconds / 1e9
            if elapsed < self.scan_time:
                cmd.angular.z = self.scan_speed
            else:
                self.index = (self.index + 1) % len(self.waypoints)
                self.state = 'MOVE'
                self.get_logger().info(f'次の地点 {self.waypoints[self.index][0]} へ向かいます')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # 停止指令
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
