#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
import deltax

class dxonNode(Node):
   def __init__(self):
      super().__init__("dxon")
      self.srv = self.create_service(Trigger, 'connect_dxon', self.connect_dxon_callback)
      self.get_logger().info("dxon node has been started.")
      # Creating deltax node here
      self.dx = deltax.DeltaX(port="/dev/ttyUSB0", model=deltax.DeltaX.DeltaX_S)
      #Are we connected on bootup?
      
   def connect_dxon_callback(self, request, response):
      self.dx.connect() 
      if self.dx.is_connected():
         self.get_logger().info("Connection success [ROS2]")
         response.success = True
         response.message = "DeltaX connected."
      else:
         self.get_logger().error("Failed to connect [ROS2]")
         response.success = False
         response.message = "DeltaX is not connected. Ensure all connections are secure."

      return response

def main(args=None):
   rclpy.init(args=args)
   node = dxonNode()
   rclpy.spin(node)
   rclpy.shutdown()


if __name__ == "__main__":
   main()