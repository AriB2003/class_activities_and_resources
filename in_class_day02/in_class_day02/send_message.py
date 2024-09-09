""" This script explores publishing ROS messages in ROS using Python """
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped

class SendMessageNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No inputs."""
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.publisher = self.create_publisher(PointStamped, 'my_point', 10)
        self.accel = [0,0,0]
        self.vel = [0,0,0]
        self.pos = [0,0,0]

    def run_loop(self):
        """Prints a message to the terminal."""
        self.accel = [3*(random.random()-0.5),3*(random.random()-0.5),3*(random.random()-0.5)]
        self.vel = [v+a*self.timer_period for a,v in zip(self.accel, self.vel)]
        self.vel = [max(min(v,1),-1) for v in self.vel]
        self.pos = [p+v*self.timer_period for v,p in zip(self.vel, self.pos)]
        self.pos = [p if -5<p<5 else -1*p for p in self.pos]
        my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        my_point = Point(x=self.pos[0], y=self.pos[1], z=self.pos[2])
        
        my_point_stamped = PointStamped(header=my_header, point=my_point)
        print(my_point_stamped)
        self.publisher.publish(my_point_stamped)

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendMessageNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
