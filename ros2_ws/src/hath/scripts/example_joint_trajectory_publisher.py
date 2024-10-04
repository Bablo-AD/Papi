
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header  # Import the Header message
 
# Define constants
arm_joints = [ 'x', 
               'y_axis', 
               'z']

class ExampleJointTrajectoryPublisherPy(Node):
    """This class executes a sample trajectory for a robotic arm
     
    """     
    def __init__(self):
        """ Constructor.
       
        """
        # Initialize the class using the constructor
        super().__init__('example_joint_trajectory_publisher_py')    
  
        # Create the publisher of the desired arm and gripper goal poses
        self.arm_pose_publisher = self.create_publisher(JointTrajectory, '/kai_controller/joint_trajectory', 1)
        
        self.timer_period = 3.0  # seconds 5.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
 
        self.frame_id = "world"
        
        # Desired time from the trajectory start to arrive at the trajectory point.
        # Needs to be less than or equal to the self.timer_period above to allow
        # the robotic arm to smoothly transition between points.
        self.duration_sec = 1
        self.duration_nanosec = 0.5 * 1e9 # (seconds * 1e9)
 
        # Set the desired goal poses for the robotic arm.
        self.arm_positions = []
        self.arm_positions.append([0.0, 0.0, 0.0]) # Home location
        self.arm_positions.append([2.0, 1.0, 2.0]) 

        self.arm_positions.append([0.0, -2.0, -1.0]) # Home location
 
       
        # Keep track of the current trajectory we are executing
        self.index = 0
 
    def timer_callback(self):
        """Set the goal pose for the robotic arm.
     
        """
        # Create new JointTrajectory messages
        msg_arm = JointTrajectory()
        msg_arm.header = Header()  
        msg_arm.header.frame_id = self.frame_id  
        msg_arm.joint_names = arm_joints
 
    
        # Create JointTrajectoryPoints
        point_arm = JointTrajectoryPoint()
        point_arm.positions = self.arm_positions[self.index]
        point_arm.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  # Time to next position
        msg_arm.points.append(point_arm)
        self.arm_pose_publisher.publish(msg_arm)
 
 
        # Reset the index
        if self.index == len(self.arm_positions) - 1:
            self.index = 0
        else:
            self.index = self.index + 1
     
def main(args=None):
   
    # Initialize the rclpy library
    rclpy.init(args=args)
   
    # Create the node
    example_joint_trajectory_publisher_py = ExampleJointTrajectoryPublisherPy()
   
    # Spin the node so the callback function is called.
    rclpy.spin(example_joint_trajectory_publisher_py)
     
    # Destroy the node
    example_joint_trajectory_publisher_py.destroy_node()
   
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
   
if __name__ == '__main__':
  main()