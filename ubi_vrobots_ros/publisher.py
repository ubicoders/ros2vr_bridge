import asyncio
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ubi_vrobots_interface.msg import VRobotStates, VRobotCMD
from .ros2ws_utils import CMDS_DICT, pack_rosmsg_to_fb_ba
from .ws2ros_utils import ros2msg_packer
from .vr_msg_py.states_msg_helper import VRobotState
from .vr_ws_srv import run_ws_server
import threading
from .vr_msg_py.R000_states_generated import StatesMsg, StatesMsgT

def start_server(ros_pub_callback):
    asyncio.run(run_ws_server(ros_pub_callback))

class VRBridgePublisher(Node):
    def __init__(self):
        super().__init__('vrobot_publisher')
        self.publisher_dict = {}
        self.thread = threading.Thread(target=start_server, args=(self.ws_on_recv_callback,))
        self.thread.start()
        self.timer_dummy = self.create_timer(10, self.dummy_callback)

    def dummy_callback(self):
        # self.get_logger().info('Dummy callback')
        pass

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
                self.publish_states(states)
        except Exception as e:
            self._logger.error(f"Error while processing message from websocket: {e}")
        finally:
            pass
        
        
    def publish_states(self, _stateMsgT:StatesMsgT):
        publisher = self.cread_publisher(_stateMsgT.sysId)
        msg = ros2msg_packer(_stateMsgT)        
        publisher.publish(msg)
        # self.get_logger().info('Publishing VRobotStates message')


class VRobotCMDSubs(Node):
    def __init__(self):
        super().__init__('vrobot_cmd_subs')
        self.subscriptions_dict = {}  # Renamed to avoid conflict
        self.vrobot_cmd_topics = []
        self.cmd_publisher_prefix = '/vrobot_cmd_pub'
        self.timer = self.create_timer(0.02, self.check_new_cmd_publishers)

    def check_new_cmd_publishers(self):
        topic_names_and_types = self.get_topic_names_and_types()
        new_vrobot_cmd_topics = [name for name, types in topic_names_and_types if 'ubi_vrobots_interface/msg/VRobotCMD' in types]
        new_vrobot_cmd_topics = [name for name in new_vrobot_cmd_topics if name.startswith(self.cmd_publisher_prefix)]

        # Find topics to add and remove
        topics_to_add = set(new_vrobot_cmd_topics) - set(self.vrobot_cmd_topics)
        topics_to_remove = set(self.vrobot_cmd_topics) - set(new_vrobot_cmd_topics)

        # Add new subscribers
        for topic in topics_to_add:
            #self.get_logger().info(f"Subscribing to new topic: {topic}")
            subscription = self.create_subscription(
                VRobotCMD,
                topic,
                self.cmd_callback,
                10
            )
            self.subscriptions_dict[topic] = subscription

        # Remove old subscribers
        for topic in topics_to_remove:
            #self.get_logger().info(f"Unsubscribing from topic: {topic}")
            self.destroy_subscription(self.subscriptions_dict[topic])
            del self.subscriptions_dict[topic]

        # Update the list of current topics
        self.vrobot_cmd_topics = new_vrobot_cmd_topics
        #self.get_logger().info(f"Current VRobotCMD topics: {self.vrobot_cmd_topics}")

    def cmd_callback(self, msg):
        # self.get_logger().info('Received VRobotCMD message')
        #self.get_logger().info(f"Received sys_id: {msg.sys_id} cmd_id: {msg.cmd_id}, int_val: {msg.int_val}, float_val: {msg.float_val}")
        sysId, ba = pack_rosmsg_to_fb_ba(msg)
        CMDS_DICT[sysId] = {"msg": ba, "updated": True}

def main():
    rclpy.init()

    node1 = VRBridgePublisher()
    node2 = VRobotCMDSubs()
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
