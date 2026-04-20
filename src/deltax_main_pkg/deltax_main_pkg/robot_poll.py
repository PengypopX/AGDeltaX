#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dx_interfaces.srv import SendString
from enum import Enum, auto


# simple dynamic ENUM list using auto()
class extract_state(Enum):
    IDLE = auto()
    APPROACH = auto()
    DESCEND = auto()
    TWIST_AND_PULL = auto()
    YANK = auto()
    DISPOSE = auto()
    ERROR = auto()
    RESET = auto() #G28


class RobotPoll(Node):
    def __init__(self):
        super().__init__('robot_poll')


        self.subscription = self.create_subscription(
            String,
            '/yolo/gcode_commands',
            self.gcode_callback,
            10
        )


        self.client = self.create_client(SendString, 'raw_gcode') # service setup for send_ts


        ### STATE MACHINE ###
        self.state = extract_state.IDLE


        ### PERFORMANCE VARIABLES ###
        self.target_x = 0
        self.target_y = 0
        self.target_z = -750 
        self.target_f = 100 
        self.pull_count = 0
        self.pull_max = 3


        self.get_logger().info('robot_poll has started. Ensure yolo_translator is publishing.')


    def gcode_callback(self, msg):
        # passover if robot is busy
        if self.state != extract_state.IDLE:
            return


        raw_gcode = msg.data 
        
        try:
            # Parse the string to apply the Y constraint
            # Expected format: "G1 X<val> Y<val> Z<val>"
            parts = raw_gcode.split()


            for part in parts:
                if part.startswith('X'):
                    self.target_x = int(part[1:])
                elif part.startswith('Y'):
                    self.target_y = int(part[1:]) 
                    if self.target_y < -200:
                        self.target_y = -200
                        self.get_logger().warn("Weed is beyond project bounding box. Capping Y to -200")
                elif part.startswith('Z'):
                    self.target_z = int(part[1:])
            
            # Fixed typo in logger (Y{self.target_z} -> Z{self.target_z})
            self.get_logger().info(f'Target locked at X{self.target_x} Y{self.target_y} Z{self.target_z}. Initiating sequence.')


            ### ENGAGE EXTRACT SM ###
            self.state = extract_state.APPROACH
            self.pull_count = 0
            self.step_machine()


        except ValueError:
            self.get_logger().error(f'Failed to parse coordinates from: {raw_gcode}')
            self.state = extract_state.IDLE


    def step_machine(self):
        gcode = ""
        
        match self.state:
            case extract_state.APPROACH:
                gcode = f'G1 X{self.target_x} Y{self.target_y} Z{self.target_z}'


            case extract_state.DESCEND:
                # Hardsetting floor. Note: Ensure this doesn't crash your physical Z-axis!
                floor_z = -950 
                gcode = f'G1 Z{floor_z}' 
                # point to CLAW_TOGGLE


            case extract_state.TWIST_AND_PULL:
                self.pull_count += 1
                # Incrementally raise Z from the -950 floor
                current_z = -950 + (self.pull_count * 10)
                
                # Alternate twist angle: e.g., 45 degrees, then -45 degrees
                #angle = 45 if (self.pull_count % 2 == 0) else -45
                gcode = f'G1 Z{current_z}'
                #claw logic here
                
                #gcode = f'G1 Z{current_z} W{angle}'


            case extract_state.YANK:
                gcode = f'G1 F2000 Z-550'


            case extract_state.DISPOSE:
                gcode = f'G1 X200 Y200 Z-550'
                # point to CLAW_TOGGLE


            case extract_state.ERROR:
                self.get_logger().error('Extraction sequence aborted, ERROR')
                self.state = extract_state.IDLE
                return


        if gcode:
            self.call_hardware_service(gcode)
           
    def call_hardware_service(self, gcode):
        if self.client.wait_for_service(timeout_sec=1.0):
            request = SendString.Request()
            request.data = gcode
            future = self.client.call_async(request)
            # Fix: Passed the function reference, didn't execute it
            future.add_done_callback(self.hardware_response_callback)
        else:
            self.get_logger().warn('Service raw_gcode not available! Aborting...')
            self.state = extract_state.ERROR
            self.step_machine()


    def hardware_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.state = extract_state.ERROR
                self.step_machine()
                return


            match self.state:
                case extract_state.APPROACH:
                    self.state = extract_state.DESCEND
                case extract_state.DESCEND:
                    self.state = extract_state.TWIST_AND_PULL
                case extract_state.TWIST_AND_PULL:
                    if self.pull_count >= self.pull_max:
                        self.state = extract_state.YANK
                case extract_state.YANK:
                    self.state = extract_state.DISPOSE
                case extract_state.DISPOSE:
                    self.get_logger().info('Extraction complete. Setting to IDLE')
                    self.state = extract_state.IDLE
                    return
            
            # Execute the next state
            self.step_machine()
               
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.state = extract_state.ERROR
            self.step_machine()


def main(args=None):
    rclpy.init(args=args)
    node = RobotPoll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




