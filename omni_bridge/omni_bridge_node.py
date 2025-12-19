import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math
import time


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OmniBridgeNode(Node):

    def __init__(self):
        super().__init__('omni_bridge_node')


        self.tf_broadcaster = TransformBroadcaster(self)

        # === Serial ===
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # === Publisher ===
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # === Robot state ===
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = time.time()

        # Timer 20 Hz
        self.create_timer(0.05, self.update)

        self.get_logger().info('Omni Bridge Node Started')

    def update(self):
        line = self.ser.readline().decode(errors='ignore').strip()

        if not line.startswith('<ENC'):
            return

        try:
            data = line.replace('<ENC,', '').replace('>', '').split(',')
            fl, fr, rl, rr = map(int, data)
        except:
            self.get_logger().warn('Serial parse error')
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # === DUMMY velocity (sementara) ===
        vx = 0.2    # m/s
        vy = 0.0
        wz = 0.0

        # === Integrasi posisi ===
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt
        self.yaw += wz * dt

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quaternion(self.yaw)
        
        self.tf_broadcaster.sendTransform(tf)
        # === Publish Odometry ===
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OmniBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
