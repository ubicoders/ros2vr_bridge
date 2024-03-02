import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from vrobots_msgs.msg import VRobotActuator, VRobotStates
from ubicoders_vrobots import Multirotor, System
import threading  # Import threading module

class UbicodersMain:
    def __init__(self, mr) -> None:
        self.mr = mr       

    def setup(self):
        # Setup your multirotor here, e.g., self.mr.mass = 2.0
        pass

    def loop(self):
        #print(self.mr.states)
        # # Example of setting PWM values
        #self.mr.set_pwm(m0=1500, m1=1500, m2=1500, m3=1500)
        pass


class VirtualRobotsPubSub(Node):
    def __init__(self):
        super().__init__('hellopy_publisher')
        qos_profile = QoSProfile(depth=10)
        self.vrstates_pub = self.create_publisher(VRobotStates, 'vr_mr_states', qos_profile)
        self.timer = self.create_timer(0.02, self.publish_vr_states)
        
        self.vrpwm_subs = self.create_subscription(VRobotActuator, 'vr_mr_pwm', self.subscribe_vr_pwm, qos_profile)

        # Initialize the Multirotor and UbicodersMain instances
        self.mr = Multirotor()
        self.ubicoders_main = UbicodersMain(self.mr)
        self.system = System(self.mr, self.ubicoders_main)
        self.system_thread = threading.Thread(target=self.system.start)
        self.system_thread.daemon = True  # This ensures the thread will be killed when the main process exits
        self.system_thread.start()

    def publish_vr_states(self):
        acc = self.mr.states.accelerometer
        gyro = self.mr.states.gyroscope
        mag = self.mr.states.magnetometer
        euler = self.mr.states.euler
        angvel = self.mr.states.ang_vel
        pos = self.mr.states.lin_pos
        linvel = self.mr.states.lin_vel
        msg = VRobotStates()
        
        if acc is not None:
            msg.acc.x = float(acc.x)
            msg.acc.y = float(acc.y)
            msg.acc.z = float(acc.z)

        if gyro is not None:
            msg.gyro.x = float(gyro.x)
            msg.gyro.y = float(gyro.y)
            msg.gyro.z = float(gyro.z)

        if mag is not None:
            msg.mag.x = float(mag.x)
            msg.mag.y = float(mag.y)
            msg.mag.z = float(mag.z)

        if euler is not None:
            msg.euler.x = float(euler.x)
            msg.euler.y = float(euler.y)
            msg.euler.z = float(euler.z)

        if angvel is not None:
            msg.angvel.x = float(angvel.x)
            msg.angvel.y = float(angvel.y)
            msg.angvel.z = float(angvel.z)

        if pos is not None:
            msg.pos.x = float(pos.x)
            msg.pos.y = float(pos.y)
            msg.pos.z = float(pos.z)

        if linvel is not None:
            msg.linvel.x = float(linvel.x)
            msg.linvel.y = float(linvel.y)
            msg.linvel.z = float(linvel.z)
        
        self.vrstates_pub.publish(msg)
        # self.get_logger().info("published")
    
    def subscribe_vr_pwm(self, msg):
        pwm = msg.pwm
        # self.get_logger().info(f"pwm: {pwm}")
        self.mr.set_pwm(m0=pwm[0], m1=pwm[1], m2=pwm[2], m3=pwm[3])


def main(args=None):
    rclpy.init(args=args)
    node = VirtualRobotsPubSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()