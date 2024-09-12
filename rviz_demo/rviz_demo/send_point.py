""" This script explores publishing ROS messages in ROS using Python """
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3


class SendPointNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendPointNode. No inputs."""
        super().__init__('send_point')
        # Create a timer that fires ten times per second
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.publisher = self.create_publisher(Marker, 'my_marker', 10)

    def run_loop(self):
        """Creates a marker."""
        my_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        my_pose = Pose(position=Point(x=1.0,y=2.0,z=0.0),orientation=Quaternion())
        my_scale = Vector3(x=0.2,y=0.4,z=1.0)
        my_color = ColorRGBA(r=0.5,g=0.9,b=0.2,a=0.7)
        my_marker = Marker(header=my_header,ns="",id=1,type=Marker.SPHERE,action=Marker.ADD,pose=my_pose,scale=my_scale,color=my_color)
        
        print(my_marker)
        self.publisher.publish(my_marker)

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = SendPointNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
