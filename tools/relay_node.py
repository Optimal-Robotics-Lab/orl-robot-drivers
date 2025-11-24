import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState, LowCmd


class Relay(Node):
    def __init__(self):
        super().__init__('wifi_node_relay')

        # Internal Topic
        self.sub_state = self.create_subscription(
            LowState, 
            '/lowstate', 
            self.state_callback, 
            10,
        )
        
        # External Topic
        self.pub_state = self.create_publisher(
            LowState, 
            '/relay/lowstate', 
            10,
        )

        # Internal Topic
        self.sub_cmd = self.create_subscription(
            LowCmd, 
            '/lowcmd', 
            self.cmd_callback, 
            10,
        )

        # External Topic
        self.pub_cmd = self.create_publisher(
            LowCmd, 
            '/relay/lowcmd', 
            10,
        )

    def state_callback(self, msg):
        self.get_logger().info(f'Received LowState! Relaying...')
        self.pub_state.publish(msg)

    def cmd_callback(self, msg):
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    relay = Relay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
