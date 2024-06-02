import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ubi_vrobots_interface.msg import VRobotActuator, VRobotStates
from ubicoders_vrobots import System, VirtualRobot
from ubicoders_vrobots.vrobots_clients.msg_helper import VRobotState
import threading  # Import threading module
from rclpy.timer import Timer

class UbicodersMain:
    def __init__(self, mr) -> None:
        self.mr = mr       

    def setup(self):
        # Setup your multirotor here, e.g., self.mr.mass = 2.0
        pass

    def loop(self):
        print(self.mr.states)
        # # Example of setting PWM values
        #self.mr.set_pwm(m0=1500, m1=1500, m2=1500, m3=1500)
        #pass

class GeneralRobot(VirtualRobot):
    def __init__(self) -> None:
        super().__init__()
        self.states: VRobotState = VRobotState(None)

    def pack(self) -> [bytes]:
        return []

    def pack_setup(self) -> [bytes]:
        return []

class VirtualRobotsPubSub(Node):
    def __init__(self):
        super().__init__('ubi_vrobots_ros')
        qos_profile = QoSProfile(depth=10)
        self.vrobots_states_pub = self.create_publisher(VRobotStates, 'vrobots_states', qos_profile)
        self.timer = self.create_timer(0.02, self.publish_vr_states)
        
        self.vrobots_actuator_subs = self.create_subscription(VRobotActuator, 'vrobots_actuator', self.subscribe_vr_actuator, qos_profile)
        self.force_reset_timer: Timer = self.create_timer(1.0, self.reset_force)
        
        # Initialize the Multirotor and UbicodersMain instances
        self.vrobot = GeneralRobot()
        self.ubicoders_main = UbicodersMain(self.vrobot)
        self.system = System(self.vrobot, self.ubicoders_main)
        self.system_thread = threading.Thread(target=self.system.start)
        self.system_thread.daemon = True  # This ensures the thread will be killed when the main process exits
        self.system_thread.start()

    def reset_force(self):
        # self.vrobot.set_pwm(m0=900, m1=900, m2=900, m3=900)
        # self.vrobot.set_force(0.0)
        pass

    def publish_vr_states(self):
        acc = self.vrobot.states.accelerometer
        gyro = self.vrobot.states.gyroscope
        mag = self.vrobot.states.magnetometer
        euler = self.vrobot.states.euler
        angvel = self.vrobot.states.ang_vel
        pos = self.vrobot.states.lin_pos
        linvel = self.vrobot.states.lin_vel
        angacc = self.vrobot.states.ang_acc
        euler_dot = self.vrobot.states.euler_dot
        quaternion = self.vrobot.states.quaternion
        force = self.vrobot.states.force
        torque = self.vrobot.states.torque
        pwm = self.vrobot.states.pwm
        actuators = self.vrobot.states.actuators

        msg = VRobotStates()

        # Populate basic fields
        if (self.vrobot.states.timestamp is not None):
            # print(type(self.vrobot.states.timestamp))
            msg.timestamp = float(self.vrobot.states.timestamp)
            msg.name = self.vrobot.states.name
        

        # Populate vector fields
        if acc is not None:
            msg.accelerometer.x = float(acc.x)
            msg.accelerometer.y = float(acc.y)
            msg.accelerometer.z = float(acc.z)

        if gyro is not None:
            msg.gyroscope.x = float(gyro.x)
            msg.gyroscope.y = float(gyro.y)
            msg.gyroscope.z = float(gyro.z)

        if mag is not None:
            msg.magnetometer.x = float(mag.x)
            msg.magnetometer.y = float(mag.y)
            msg.magnetometer.z = float(mag.z)

        if euler is not None:
            msg.euler.x = float(euler.x)
            msg.euler.y = float(euler.y)
            msg.euler.z = float(euler.z)

        if angvel is not None:
            msg.ang_vel.x = float(angvel.x)
            msg.ang_vel.y = float(angvel.y)
            msg.ang_vel.z = float(angvel.z)

        if pos is not None:
            msg.lin_pos.x = float(pos.x)
            msg.lin_pos.y = float(pos.y)
            msg.lin_pos.z = float(pos.z)

        if linvel is not None:
            msg.lin_vel.x = float(linvel.x)
            msg.lin_vel.y = float(linvel.y)
            msg.lin_vel.z = float(linvel.z)

        if angacc is not None:
            msg.ang_acc.x = float(angacc.x)
            msg.ang_acc.y = float(angacc.y)
            msg.ang_acc.z = float(angacc.z)

        if euler_dot is not None:
            msg.euler_dot.x = float(euler_dot.x)
            msg.euler_dot.y = float(euler_dot.y)
            msg.euler_dot.z = float(euler_dot.z)

        if quaternion is not None:
            msg.quaternion.x = float(quaternion.x)
            msg.quaternion.y = float(quaternion.y)
            msg.quaternion.z = float(quaternion.z)
            msg.quaternion.w = float(quaternion.w)

        if force is not None:
            msg.force.x = float(force.x)
            msg.force.y = float(force.y)
            msg.force.z = float(force.z)

        if torque is not None:
            msg.torque.x = float(torque.x)
            msg.torque.y = float(torque.y)
            msg.torque.z = float(torque.z)

        if pwm is not None:
            msg.pwm = pwm  # Assuming pwm is a list of uint32 values

        if actuators is not None:
            msg.actuators = actuators  # Assuming actuators is a list of float values

        # Publish the message
        self.vrobots_states_pub.publish(msg)

    
    def subscribe_vr_actuator(self, msg):
        pwm = msg.pwm
        force = msg.force
        if (pwm is not None and len(pwm) >= 4):
            self.vrobot.set_pwm(m0=pwm[0], m1=pwm[1], m2=pwm[2], m3=pwm[3])
        if (force is not None):
            self.vrobot.set_force(force)
        self.force_reset_timer.reset()

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