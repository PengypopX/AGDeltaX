#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import deltax

class drive_motor(Node):
   def __init__(self):
      #initialization
      super().__init__("drive_motor")
      #end initialization
      #variable section
      self.x = 0
      self.y = 0
      self.z = 0
      self.last_x = 1.5
      self.last_y = .53
      self.last_z = -740
      self.F = 100
      # Prior homing values X = 1.5 Y = .53 Z = -740

      #end variable section
      #subscription logic
      self.get_logger().info("dxon node has been started.")
      #end subscription logic
      # Creating deltax node here
      self.dx = deltax.DeltaX(port="/dev/ttyUSB0", model=deltax.DeltaX.DeltaX_S)
      #Are we connected on bootup?

      #test
      self.dx.connect()
      self.setSpeed(self.F) #set speed to F value
      self.drive_thru(100,-100,-800)

      self.get_logger().info("The node has moved into position")

      self.reset_pos()

   
   #IDEAL Vector: X100 Y100 Z100 F100
   def setSpeed(self, pass_f):
      self.dx.send(f"G1 F{pass_f}")
   def drive_x(self, pass_x):
      self.dx.send(f"G1 X{pass_x}")
      self.last_x = pass_x
   def drive_y(self, pass_y):
      self.dx.send(f"G1 Y{pass_y}")
      self.last_y = pass_y
   def drive_z(self, pass_z):
      self.dx.send(f"G1 Z{pass_z}")
      self.last_z = pass_z
   def drive_x_and_y(self,pass_x,pass_y):
      self.dx.send(f"G1 X{pass_x} Y{pass_y}")

   def drive_thru(self, pass_x, pass_y, pass_z):
      self.drive_x_and_y(pass_x, pass_y)
      self.drive_z(pass_z)
      self.dx.send("G28") #home
      self.get_logger().info("drive_thru_executed")
   
   def reset_pos(self):
      self.dx.homing()

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
   node = drive_motor()
   rclpy.spin(node)
   rclpy.shutdown()


if __name__ == "__main__":
   main()