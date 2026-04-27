#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dx_interfaces.srv import SendString
import serial
import time

PORT = "/dev/ttyUSB1" #May need to change
BAUD = 115200 #Monitor baud rate if errors occur

class ClawNode(Node):
    def __init__(self):
        super().__init__("claw")
        self.get_logger().info("Claw node initializing...")


        # Initialize the DeltaX hardware connection
        try:
            self.get_logger().info(f"Connecting to Arduino: {PORT}")
            self.arduino = (serial.Serial(PORT,BAUD,timeout=5)) #connect to arduino device
            time.sleep(2) #wait for connection on instantiation
            startup = self.arduino.readline().decode("utf-8").strip() #Purely for logging
            self.get_logger().info(f"{startup}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None


        # Create the Service Server
        self.claw_service = self.create_service(
            SendString,
            "CLAW_INPUT",
            self.claw_callback
        )


        self.get_logger().info("claw_command service server is ready and listening.")


    def claw_callback(self, request, response):
        cmd = request.data.strip().upper() #parses data to uppercase

        if cmd not in ("OPEN", "CLOSE"):
            self.get_logger().warn(f"Expecting OPEN or CLOSE, check input string")
            response.success = False
            return response
        if self.arduino is None:
            self.get_logger().error("Arduino is not connected")
            response.success = False
            return response
        
        self.get_logger().info(f"Sending command: {cmd}")

        try:
            self.arduino.write(f"{cmd}\n".encode("utf-8"))
            arduino_response = self.arduino.readline().decode("utf-8").strip() #Logging
            self.get_logger().info(f"Arduino response: {arduino_response}")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Arduino communication failure: {e}")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ClawNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly disconnect the serial port when the node is killed
        if hasattr(node, 'arduino') and node.arduino is not None:
            node.arduino.close()
            node.get_logger().info("Arduino disconnected.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()




