import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('square_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 1
        self.counter = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        if self.counter%2 == 1 :
            msg.linear.x = 1.0
        
        elif self.counter%2 == 0 :
            msg.angular.z = 1.57
        
        self.publisher_.publish(msg)
        self.counter += 1
        # self.get_logger().info(f'{self.counter}')


def main(args=None):
    rclpy.init(args=args)
    pub = CirclePublisher()
    rclpy.spin(pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()