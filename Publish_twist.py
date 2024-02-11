import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32MultiArray,Int32

class MyPublish(Node):
    def __init__(self):
        super().__init__('twist_publish')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.publish_message)
      
    def publish_message(self):
        message = Twist()
        message.linear.x = float(sys.argv[1])
        message.linear.y = float(sys.argv[2])
        message.angular.z = float(sys.argv[3]) 
        self.get_logger().info('Sending - Linear Velocity X : %f, Linear Velocity Y : %f, Angular Velocity : %f' 
                               % (message.linear.x, message.linear.y, message.angular.z))
        self.publisher_.publish(message)
   
def main(args=None):
    rclpy.init(args=args)
    publisher = MyPublish()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
