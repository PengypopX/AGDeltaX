#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import deltax

class RobotSensorNode(Node):
   def __init__(self):
      super().__init__("robot_sensor")
      self.publisher_ = self.create_publisher(String, "robot_sensor", 10)
      self.timer = self.create_timer(1.0, self.publish_status)
      self.get_logger().info("robot_sensor node has been started.")

      # Creating deltax node here
      self.dx = deltax.DeltaX(port="/dev/ttyUSB0", model=deltax.DeltaX.DeltaX_S)
      #Are we connected on bootup?
      self.dx.connect()
      if self.dx.is_connected():
         self.get_logger().info("DeltaX Sensor connected successfully.")
      else:
         self.get_logger().error("Failed to connect to DeltaX Sensor.")

   def publish_status(self):
      msg = String()
      msg.data = "connected"
      self.publisher_.publish(msg)
      self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
   rclpy.init(args=args)
   node = RobotSensorNode()
   rclpy.spin(node)
   rclpy.shutdown()


if __name__ == "__main__":
   main()