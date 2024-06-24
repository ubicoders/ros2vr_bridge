import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from ubi_vrobots_interface.msg import VRobotStates, VRobotCMD
import time

class VRobotPublisher(Node):

    def __init__(self):
        super().__init__('vrobot_publisher')
        self.publisher_ = self.create_publisher(VRobotStates, 'vrobot_states', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

        self.subscription = self.create_subscription(
            VRobotCMD,
            'vrobot_cmd',
            self.cmd_callback,
            10)
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        msg = VRobotStates()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = 'robot_1'
        msg.sys_id = 1

        msg.lin_acc = Vector3()
        msg.lin_vel = Vector3()
        msg.lin_pos = Vector3()
        msg.altitude = 100.0

        msg.ang_acc = Vector3()
        msg.ang_vel = Vector3()
        msg.euler = Vector3()
        msg.euler_dot = Vector3()
        msg.quaternion = Quaternion()

        msg.pwm = [0]*4
        msg.actuators = [0.0]*4
        msg.force = Vector3()
        msg.torque = Vector3()

        msg.accelerometer = Vector3()
        msg.gyroscope = Vector3()
        msg.magnetometer = Vector3()
        msg.barometer = 1013.25
        msg.temperature = 25.0
        msg.gps_pos = Vector3()
        msg.gps_vel = Vector3()

        msg.mass = 1.0
        msg.cg = Vector3()
        msg.moment_arms = [0.0]*3
        msg.moi_3x1 = Vector3()
        msg.moi_3x3 = [0.0]*9
        msg.extra_props = [0.0]*5

        msg.collisions = []  # Assuming you have a Collision message type defined
        

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing VRobotStates message')


    def cmd_callback(self, msg):
        self.get_logger().info('Received VRobotCMD message')
        # Process the received command message here
        # Example: Print some values
        self.get_logger().info(f"Received cmd_id: {msg.cmd_id}, int_val: {msg.int_val}, float_val: {msg.float_val}")


def main(args=None):
    rclpy.init(args=args)
    vrobot_publisher = VRobotPublisher()
    rclpy.spin(vrobot_publisher)
    vrobot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
