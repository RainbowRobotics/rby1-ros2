import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback,
            10)
        self.get_logger().info('Trajectory subscriber node started!')

    def listener_callback(self, msg):
        self.get_logger().info('Received trajectory points:')
        for point in msg.points:
            self.get_logger().info(f'{point}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
