#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dx_interfaces.srv import SendString
from enum import Enum, auto


#simple dynamic ENUM list using auto()
#typically auto will just list 1-7, but it's arbitrary since we assigned names to values
class extract_state(Enum):
    IDLE = auto()
    APPROACH = auto()
    PLUNGE = auto()
    MICRO_PULL = auto()
    JERK = auto()
    DISPOSE = auto()
    ERROR = auto()
    RESET = auto() #G28

class RobotPoll(Node):
    def __init__(self):
        super().__init__('robot_poll')

        # Subscribe to the G-code topic published by yolo_translator
        # (Make sure this topic name matches what you set in yolo_translator!)
        self.subscription = self.create_subscription(
            String,
            '/yolo/gcode_commands', 
            self.gcode_callback,
            10
        )

        self.client = self.create_client(SendString, 'raw_gcode') #service setup for send_ts

        ### STATE MACHINE ###
        self.state = extract_state.IDLE

        ### PERFORMANCE VARIABLES ###
        self.target_x = 0
        self.target_y = 0
        self.target_z = -750 #default Z (consider making this higher to remove camera interference)
        self.target_f = 100 #default speed 
        self.pull_count = 0 
        self.pull_max = 3

        self.get_logger().info('robot_poll has started. Ensure yolo_translator is publishing.')

    def gcode_callback(self, msg):
        # passover if robot is busy
        if self.state != extract_state.IDLE:
            return

        raw_gcode = msg.data #preps gcode to send to send_ts
        
        try:
            # Parse the string to apply the Y constraint 
            # Expected format: "G1 X<val> Y<val> Z<val>"
            parts = raw_gcode.split()

            for part in parts:
                if part.startswith('X'):
                    self.target_x = int(part[1:])
                elif part.startswith('Y'):
                    self.target_y = int(part[1:]) # Extract the number after 'Y'
                    if self.target_y < -200:
                        self.target_y = -200
                        self.get_logger().warn("Weed is beyond project bounding box. Capping Y to -200") #consider calling end state here
                elif part.startswith('Z'):
                    self.target_z = int(part[1:])
            self.get_logger().info(f'Target locked at X{self.target_x} Y{self.target_y} Y{self.target_z}. Initiating sequence.')

            ### ENGAGE EXTRACT SM ###
            self.state = extract_state.APPROACH
            self.pull_count = 0
            self.step_machine()

        except ValueError:
            self.get_logger().error(f'Failed to parse coordinates from: {raw_gcode}')
            self.state = extract_state.IDLE

    def call_hardware_service(self,gcode):
            if self.client.wait_for_service(timeout_sec=1.0):
                request = SendString.Request()
                request.data = gcode
                future = self.client.call_async(request)
                future.add_done_callback(self.hardware_response_callback))
            else:
                self.get_logger().warn('Service raw_gcode not available! Aborting...')
                self.state = extract_state.ERROR

    
    ###FINISH UP CODE HERE
    ### ADD STEP_MACHINE FUNCTION
    def hardware_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Move complete')
            else:
                self.get_logger().warn('Move failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            # Free up the node to accept the next incoming detection
            self.is_moving = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




