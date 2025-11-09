import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DifferentialDriveCommander(Node):
    """
    A ROS 2 Node that publishes Twist messages to control a differential drive robot.
    This assumes the robot subscribes to the 'cmd_vel' topic for linear and angular velocity.
    """
    def __init__(self):
        super().__init__('differential_drive_commander')
        # The topic name must match the one the gz_ros_diff_drive plugin is subscribed to.
        # We used 'cmd_vel' in the plugin setup.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Differential Drive Commander Node Initialized.")

    def drive_robot(self, linear_x, angular_z, duration=1.0, rate=50):
        """
        Publishes a Twist message for a specified duration at a given rate.

        Args:
            linear_x (float): Linear velocity in the X direction (m/s).
            angular_z (float): Angular velocity around the Z axis (rad/s).
            duration (float): How long to send this command (seconds).
            rate (int): Publishing rate in Hz.
        """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z

        period = 3.0 / rate
        num_messages = int(duration * rate)
        
        self.get_logger().info(f"Sending commands: Linear={linear_x:.2f} m/s, Angular={angular_z:.2f} rad/s for {duration} seconds.")

        for i in range(num_messages):
            self.publisher_.publish(twist_msg)
            # Use a non-ROS sleep to maintain rate in a standalone script
            time.sleep(period) 
            
        self.get_logger().info("Finished sending commands.")

    def stop_robot(self):
        """
        Sends a zero velocity command to stop the robot.
        """
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info("STOP command sent.")

    def run_command_sequence(self):
        """
        Defines a sequence of movements for demonstration.
        For RL, this entire function would be replaced by your training loop logic.
        """
        # 1. Drive forward and slightly right (longer duration)
        self.drive_robot(linear_x=1.0, angular_z=-1.0, duration=3.0)
        self.stop_robot()
        
        # Pause
        time.sleep(1.0)
        
        # 2. Turn in place (spin left)
        self.drive_robot(linear_x=0.0, angular_z=1.0, duration=2.5)
        self.stop_robot()

        # Pause
        time.sleep(1.0)
        
        # 3. Drive backward slowly
        self.drive_robot(linear_x=-0.2, angular_z=0.0, duration=4.0)
        self.stop_robot()

        self.get_logger().info("Full movement sequence finished.")

        
def main(args=None):
    rclpy.init(args=args)

    commander = DifferentialDriveCommander()
    
    # Run the sequence
    commander.run_command_sequence()

    # Clean up
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()