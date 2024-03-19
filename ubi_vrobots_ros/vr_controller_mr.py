import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ubi_vrobots_msgs.msg import VRobotActuator, VRobotStates
import numpy as np

class Multirotor:
    def __init__(self):
        pass


    def my_controller(self, eul, angvel, pos, vel):
        print(pos)
        print(vel)
        
        # Implement your controller here


        return 1220, 1220, 1220, 1225
        

class VirtualRobotsPubSub(Node):
    def __init__(self):
        super().__init__('ubi_vr_mr')
        qos_profile = QoSProfile(depth=10)
        self.vr_act_pub = self.create_publisher(VRobotActuator, 'vrobots_actuator', qos_profile)
        self.vr_states_subs = self.create_subscription(VRobotStates, 'vrobots_states', self.subs_vr_state, qos_profile)
        self.mr = Multirotor()
    
    def subs_vr_state(self, msg:VRobotStates):
        eul = msg.euler
        angvel = msg.angvel
        pos = msg.pos
        vel = msg.linvel

        # print(eul)
        # print(angvel)

        m0, m1, m2, m3 = self.mr.my_controller(eul, angvel, pos, vel)
        out_msg = VRobotActuator()
        out_msg.pwm = [m0, m1, m2, m3]
        print(out_msg)
        self.vr_act_pub.publish(out_msg)

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