""" This script implements an emergency stop with bump """
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

class EmergencyStopNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No inputs."""
        super().__init__('emergency_stop_node')
        # Create a timer that fires ten times per second
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.twist)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.read_scan, 10)
        self.forward_distance = 0

    def twist(self):
        print(self.forward_distance)
        if self.forward_distance > 0.35:    
            my_twist = Twist(linear = Vector3(x=min(self.forward_distance/2-0.12,1.0),y=0.0,z=0.0), angular = Vector3(x=0.0,y=0.0,z=0.0))
            self.publisher.publish(my_twist)
        else:
            my_twist = Twist(linear = Vector3(x=0.0,y=0.0,z=0.0), angular = Vector3(x=0.0,y=0.0,z=0.0))
            self.publisher.publish(my_twist)
    
    def read_scan(self, msg):
        self.forward_distance = msg.ranges[0]
        if math.isinf(self.forward_distance):
            self.forward_distance = 5

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = EmergencyStopNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
