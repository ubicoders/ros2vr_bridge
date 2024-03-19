import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ubi_vrobots_msgs.msg import VRobotActuator, VRobotStates
import numpy as np

class Multirotor:
    def __init__(self):
        self.h_err_int = 0


    def my_controller(self, eul, angvel, pos, vel):
        print(pos)
        att_set_point = np.array([0, 0, 0])
        
        h_est = -pos.z
        h_dot = -vel.z
        h_sp = 2
        h_kp = 20
        h_kd = 15
        h_ki = 0.1
        h_err = h_sp - h_est
        self.h_err_int += h_err
        if (self.h_err_int > 100):
            self.h_err_int = 100
        
        throttle = 1200 + h_kp * h_err - h_kd * h_dot + h_ki*self.h_err_int

        # attitude controller
        att_kp = np.array([1, 1, 0])
        att_kd = np.array([10, 10, 0])

        eul_est = np.array([eul.x, eul.y, eul.z])
        # print(gt.states)
        angvel_est = np.array([angvel.x, angvel.y, angvel.z]) # rad/s
        att_err = att_set_point - eul_est
        
        output = att_kp * att_err - att_kd * angvel_est

        m0 = throttle + output[0] + output[1]
        m1 = throttle - output[0] + output[1]
        m2 = throttle - output[0] - output[1]
        m3 = throttle + output[0] - output[1]
        return int(m0), int(m1), int(m2), int(m3)
        

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