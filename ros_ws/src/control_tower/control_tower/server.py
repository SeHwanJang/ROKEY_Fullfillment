# publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Int32Publisher(Node):
    def __init__(self):
        super().__init__('int32_publisher')
        self.publisher_ = self.create_publisher(Int32, 'stop', 10)
        msg = Int32()
        msg.data = 1
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Int32Publisher()
    rclpy.spin(node)

    # 종료 시 클린업
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
