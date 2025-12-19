import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import serial, math, time


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OmniBridgeNode(Node):
    def __init__(self):
        super().__init__('omni_bridge_node')

        # SERIAL
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        time.sleep(2)

        # ROS
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ROBOT PARAM
        self.R = 0.05
        self.L = 0.20

        self.CPR = [44*45, 44*45, 44*36, 44*36]
        self.GAIN = [1.0, 1.0, 0.8, 0.8]
        self.ENC_DIR = [1, -1, 1, -1]

        # STATE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_enc = [0, 0, 0, 0]
        self.last_time = time.time()

        self.create_timer(0.02, self.loop)
        self.get_logger().info("✅ Omni Teleop READY")

    def cmd_cb(self, msg: Twist):
        cmd = f"CMD,{msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}\n"
        self.ser.write(cmd.encode())

    def loop(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line.startswith("ENC"):
            return

        enc = list(map(int, line.split(',')[1:5]))
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        d = [(enc[i]-self.last_enc[i])*self.ENC_DIR[i] for i in range(4)]
        self.last_enc = enc

        w = [(d[i]/self.CPR[i])*2*math.pi/dt*self.GAIN[i] for i in range(4)]

        vx = ( w[0]+w[1]+w[2]+w[3]) * self.R / 4
        vy = (-w[0]+w[1]+w[2]-w[3]) * self.R / 4
        wz = (-w[0]+w[1]-w[2]+w[3]) * self.R / (4*self.L)

        if abs(vx) > 0.02 or abs(vy) > 0.02:
            wz = 0.0

        self.yaw += wz * dt
        self.x += (vx*math.cos(self.yaw) - vy*math.sin(self.yaw)) * dt
        self.y += (vx*math.sin(self.yaw) + vy*math.cos(self.yaw)) * dt

        # TF
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = yaw_to_quat(self.yaw)
        self.tf_broadcaster.sendTransform(tf)

        # ODOM
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    rclpy.spin(OmniBridgeNode())
    rclpy.shutdown()
