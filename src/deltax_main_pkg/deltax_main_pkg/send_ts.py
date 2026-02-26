#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import deltax
from dx_interfaces.srv import SendString
class SendTS(Node):
    def __init__(self):
        super().__init__("send_ts")
        self.get_logger().info("DeltaX can send it!")

        # Creating deltax node here
        self.dx = deltax.DeltaX(port="/dev/ttyUSB0", model=deltax.DeltaX.DeltaX_S)
        self.dx.connect()
        self.dx.send("G1 F100")

        # Service server
        self.send_ts_service = self.create_service(
            SendString,
            'raw_gcode',
            self.send_ts_callback
        )

        self.get_logger().info("send_ts node started")

    def send_ts_callback(self, request, response):
        gcode = request.data
        if gcode:
            self.dx.send(gcode)
            self.get_logger().info(f'Executing: {gcode}')
            response.success = True
        else:
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SendTS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()