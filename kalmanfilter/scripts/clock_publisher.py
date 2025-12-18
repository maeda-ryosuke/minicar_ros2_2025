import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time

def main(args=None):
    rclpy.init(args=args)
    node = Node('simple_clock_publisher')
    pub = node.create_publisher(Clock, '/clock', 10)
    
    def timer_callback():
        msg = Clock()
        msg.clock = node.get_clock().now().to_msg()
        pub.publish(msg)

    timer = node.create_timer(0.01, timer_callback) # 100Hzで配信
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()