import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MobilityPublisher(Node):
    def __init__(self):
        super().__init__('mobility_publisher')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_vel_pub = self.create_publisher(Float64MultiArray, '/cmd_joint_vel', 10)
        
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        twist_msg = Twist()
        
        twist_msg.linear.x = -0.05
        twist_msg.angular.z = 0.01745
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f'Publishing cmd_vel: linear_x={twist_msg.linear.x}, heading={twist_msg.angular.z}')
        
        joint_msg = Float64MultiArray()
        joint_msg.data = [3.14, 3.14]
        self.joint_vel_pub.publish(joint_msg)
        self.get_logger().info(f'Publishing joint_vel: right_wheel={joint_msg.data[0]}, left_wheel={joint_msg.data[1]}')
        
        
def main(args=None):
    rclpy.init(args=args)
    
    mobility_publisher = MobilityPublisher()
    
    rclpy.spin(mobility_publisher)
    
    mobility_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
