import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

# Import the RealSense or Vicon nodes
from .realsense_sys_node import RealSense
from .vicon_sys_node import Vicon

from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
import sys

class DroneGUI(QWidget):
    def __init__(self):
        super().__init__('rob498_drone_1')
        print('node init')

        # realsense

        # Initialize storage variables
        self.position = None
        self.orientation = None
        self.timestamp = None
        self.frame_id = None

        # Initialize SENDING storage variables with proper types
        self.set_position = Point()
        self.set_orientation = Quaternion()
        self.set_orientation.w = -1.0

        # Subscriber to RealSense pose data
        self.realsense_subscriber = self.create_subscription(Odometry, '/camera/pose/sample', self.realsense_callback, qos_profile)
        self.get_logger().info('Subscribing to RealSense!')

        # Vicon
        self.vicon_subscriber = self.create_subscription(PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.vicon_callback, 1)
        self.get_logger().info('Subscribing to Vicon!')

        # setpoint
        self.vicon_subscriber = self.create_subscription(PoseStamped, '/mavros/setpoint_position/local', self.setpoint_callback, 1)
        self.get_logger().info('Subscribing to setpoint!')

        # drone pose reading
        self.vicon_subscriber = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 1)
        self.get_logger().info('Subscribing to local pose!')
        
        # Statement to end the inits
        self.get_logger().info('Nodes All Setup and Started!')

        self.frame_id = "map"

        self.initUI()

    def initUI(self):
        self.layout = QVBoxLayout()

        self.realsense_label = QLabel("RealSense Position: x=0, y=0, z=0")
        self.vicon_label = QLabel("Vicon Position: x=0, y=0, z=0")
        self.setpoint_label = QLabel("Setpoint Position: x=0, y=0, z=0")
        self.pose_label = QLabel("Drone Pose Position: x=0, y=0, z=0")

        self.layout.addWidget(self.realsense_label)
        self.layout.addWidget(self.vicon_label)
        self.layout.addWidget(self.setpoint_label)
        self.layout.addWidget(self.pose_label)

        self.setLayout(self.layout)
        self.setWindowTitle("Drone Data GUI")
        self.show()

    def update_realsense(self, position):
        self.realsense_label.setText(f"RealSense Position: x={position.x}, y={position.y}, z={position.z}")

    def update_vicon(self, position):
        self.vicon_label.setText(f"Vicon Position: x={position.x}, y={position.y}, z={position.z}")

    def update_setpoint(self, position):
        self.setpoint_label.setText(f"Setpoint Position: x={position.x}, y={position.y}, z={position.z}")

    def update_pose(self, position):
        self.pose_label.setText(f"Drone Pose Position: x={position.x}, y={position.y}, z={position.z}")
    
    def realsense_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.timestamp = self.get_clock().now().to_msg()
        # self.frame_id = msg.header.frame_id

        self.orientation.x *= -1
        self.orientation.y *= -1
        self.orientation.z *= -1
        self.orientation.w *= -1

        # Print values normally
        print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
        print(f"Frame ID: {self.frame_id}")
        
        # Everytime we get stuff, write both immediately
        self.send_vision_pose()
        self.send_setpoint()

    
    def vicon_callback(self, msg):
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.timestamp = self.get_clock().now().to_msg()
        # self.frame_id = msg.header.frame_id

        self.orientation.x *= -1
        self.orientation.y *= -1
        self.orientation.z *= -1
        self.orientation.w *= -1

        # Print values normally
        print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
        print(f"Frame ID: {self.frame_id}")
    
        # Everytime we get stuff, write both immediately
        self.send_vision_pose()
        self.send_setpoint()


    def setpoint_callback(self, msg):
    # Store the setpoint position and orientation from the incoming message
        self.set_position = msg.pose.position
        self.set_orientation = msg.pose.orientation
        self.timestamp = self.get_clock().now().to_msg()
        
        # Print out the setpoint values
        print(f"Setpoint Position: x={self.set_position.x}, y={self.set_position.y}, z={self.set_position.z}")
        print(f"Setpoint Orientation: x={self.set_orientation.x}, y={self.set_orientation.y}, z={self.set_orientation.z}, w={self.set_orientation.w}")
        print(f"Setpoint Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")

        # Send the setpoint whenever it is received
        self.send_setpoint()


    def pose_callback(self, msg):
        # Store the drone's pose (position and orientation) from the incoming message
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.timestamp = self.get_clock().now().to_msg()

        # Print out the pose values
        print(f"Drone Pose Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        print(f"Drone Pose Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        print(f"Pose Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")

        # Send the drone pose whenever it is received
        self.send_vision_pose()


def main(args=None):
    rclpy.init(args=args) 
    print('starting node')
    app = QApplication(sys.argv)
    
    # running_node = RealSense()
    # running_node = Vicon()
    # rclpy.spin(running_node)  # Start RealSense node
    comm_node = DroneGUI()  # Pass RealSense instance
    # rclpy.spin(running_node)  # Start RealSense node
    rclpy.spin(comm_node)  # Start CommNode

    # running_node.destroy_node()
    comm_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
