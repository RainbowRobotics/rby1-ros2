# The joint state of each part of the robot, for monitoring and logging: the driver's
# joint_states topic. The standard-typed joint_overall_states topic (sensor_msgs/JointState,
# every joint including the wheels) is what TF, MoveIt and RViz read; the base's motion is on odom.
std_msgs/Header header
sensor_msgs/JointState torso
sensor_msgs/JointState right_arm
sensor_msgs/JointState left_arm
sensor_msgs/JointState head
