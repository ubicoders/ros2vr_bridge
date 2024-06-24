import asyncio
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion
from ubi_vrobots_interface.msg import VRobotStates, VRobotCMD
from .vr_msg_py.states_msg_helper import VRobotState
from .vr_ws_srv import run_ws_server
import threading
from .vr_msg_py.R000_states_generated import StatesMsg, StatesMsgT


def start_server(ros_pub_callback):
    asyncio.run(run_ws_server(ros_pub_callback))

class VRobotPublisher(Node):
    def __init__(self):
        super().__init__('vrobot_publisher')
        self.publisher_dict = {}
        self.thread = threading.Thread(target=start_server, args=(self.ws_on_recv_callback,))
        self.thread.start()
        self.timer_dummy = self.create_timer(10, self.dummy_callback)

    def dummy_callback(self):
        self.get_logger().info('Dummy callback')

    def cread_publisher(self, sys_id):
        if sys_id in self.publisher_dict:
            return self.publisher_dict[sys_id]
        else:
            pub = self.create_publisher(VRobotStates, f'vrobot_states_{sys_id}', 10)
            self.publisher_dict[sys_id] = pub
            return pub
        
    def ws_on_recv_callback(self, msg):
        # self._logger.info(f"Received message from websocket: {msg}")
        try:
            if (StatesMsg.StatesMsgBufferHasIdentifier(msg, 0) is True):
                statesMsgT = StatesMsgT.InitFromPackedBuf(msg, 0)
                states = VRobotState(statesMsgT)
                self._logger.info(f"Received message from websocket, sysId: {states.sysId}")
                self._logger.info(f"Received message from websocket: {states.linPos}")
                self.publish_states(states)
        except Exception as e:
            self._logger.error(f"Error while processing message from websocket: {e}")
        finally:
            pass
        
    def publish_states(self, stateMsgT:StatesMsgT):
        seconds = int(stateMsgT.timestamp/1000)
        nanoseconds = int((stateMsgT.timestamp - seconds*1000)*1e6) 
        msg = VRobotStates()
        msg.header = Header()
        msg.header.stamp.sec = seconds
        msg.header.stamp.nanosec = nanoseconds
        msg.header.frame_id = 'default'
        msg.name = stateMsgT.name
        msg.sys_id = stateMsgT.sysId
        msg.lin_acc = Vector3(x=stateMsgT.linAcc.x, y=stateMsgT.linAcc.y, z=stateMsgT.linAcc.z)
        msg.lin_vel = Vector3(x=stateMsgT.linVel.x, y=stateMsgT.linVel.y, z=stateMsgT.linVel.z)
        msg.lin_pos = Vector3(x=stateMsgT.linPos.x, y=stateMsgT.linPos.y, z=stateMsgT.linPos.z)
        msg.altitude = stateMsgT.altitude

        msg.ang_acc = Vector3(x=stateMsgT.angAcc.x, y=stateMsgT.angAcc.y, z=stateMsgT.angAcc.z)
        msg.ang_vel = Vector3(x=stateMsgT.angVel.x, y=stateMsgT.angVel.y, z=stateMsgT.angVel.z)
        msg.euler = Vector3(x=stateMsgT.euler.x, y=stateMsgT.euler.y, z=stateMsgT.euler.z)
        msg.euler_dot = Vector3(x=stateMsgT.eulerDot.x, y=stateMsgT.eulerDot.y, z=stateMsgT.eulerDot.z)
        msg.quaternion = Quaternion(x=stateMsgT.quaternion.x, y=stateMsgT.quaternion.y, z=stateMsgT.quaternion.z, w=stateMsgT.quaternion.w)

        msg.pwm = [ int( pwm) for pwm in stateMsgT.pwm ]
        msg.actuators = [ act for act in stateMsgT.actuators ]
        # msg.force = Vector3()
        # msg.torque = Vector3()

        # msg.accelerometer = Vector3()
        # msg.gyroscope = Vector3()
        # msg.magnetometer = Vector3()
        # msg.barometer = 1013.25
        # msg.temperature = 25.0
        # msg.gps_pos = Vector3()
        # msg.gps_vel = Vector3()

        # msg.mass = 1.0
        # msg.cg = Vector3()
        # msg.moment_arms = [0.0]*3
        # msg.moi_3x1 = Vector3()
        # msg.moi_3x3 = [0.0]*9
        # msg.extra_props = [0.0]*5

        # msg.collisions = []  # Assuming you have a Collision message type defined
        

        publisher = self.cread_publisher(stateMsgT.sysId)
        publisher.publish(msg)
        self.get_logger().info('Publishing VRobotStates message')


class VRobotCMDSubs(Node):
    def __init__(self):
        super().__init__('vrobot_cmd_subs')
        self.subscription = self.create_subscription(
            VRobotCMD,
            'vrobot_cmd',
            self.cmd_callback,
            10)

    def cmd_callback(self, msg):
        self.get_logger().info('Received VRobotCMD message')
        # Process the received command message here
        # Example: Print some values
        self.get_logger().info(f"Received cmd_id: {msg.cmd_id}, int_val: {msg.int_val}, float_val: {msg.float_val}")

def main():
    rclpy.init()

    node1 = VRobotPublisher()
    node2 = VRobotCMDSubs()
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
