import rclpy
from rclpy.node import Node
import socket
import threading
from ubi_vrobots_interface.msg import VRobotActuator, VRobotStates
from ubicoders_vrobots.vrobots_bridge.vr_ws_srv import run_bridge_server
from ubicoders_vrobots.vrobots_bridge.vr_rest_srv import run_server


class ServerNodeROS2(Node):
    def __init__(self):
        super().__init__('server_node')
        self.start_servers()

    def is_port_in_use(self, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            return sock.connect_ex(('localhost', port)) == 0

    def start_servers(self):
        task_pair_list = [
            {"target": run_bridge_server, "args": (12740,)},
            {"target": run_server, "args": (12741,)},
        ]
        self.threads = []
        for task_pair in task_pair_list:
            port = task_pair["args"][0]
            if self.is_port_in_use(port):
                self.get_logger().info(f"Port {port} is already in use. Skipping...")
                continue
            t = threading.Thread(target=task_pair["target"], args=task_pair["args"])
            t.start()
            self.threads.append(t)

        for t in self.threads:
            t.join()


def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNodeROS2()
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
