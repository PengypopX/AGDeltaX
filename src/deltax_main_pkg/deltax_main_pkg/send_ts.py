#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import deltax
from dx_interfaces.srv import SendString


class SendTS(Node):
    def __init__(self):
        super().__init__("send_ts")
        self.get_logger().info("DeltaX node initializing...")


        # Initialize the DeltaX hardware connection
        try:
            self.dx = deltax.DeltaX(port="/dev/ttyUSB0", model=deltax.DeltaX.DeltaX_S)
            self.dx.connect()
            self.dx.send("G1 F100", timeout=10.0) # Set initial feedrate
            self.get_logger().info("Hardware connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to DeltaX: {e}")


        # Create the Service Server
        self.send_ts_service = self.create_service(
            SendString,
            'raw_gcode',
            self.send_ts_callback
        )


        self.get_logger().info("send_ts service server is ready and listening.")


    def send_ts_callback(self, request, response):
        gcode = request.data
        if gcode:
            self.get_logger().info(f'Executing: {gcode}')
            try:
                # 1. Send command and override the 5-second library trap
                # 30 seconds should be plenty for a workspace move, but increase it if needed.
                self.dx.send(gcode, timeout=30.0)
                
                # 2. Check if the move succeeded cleanly
                if self.dx.lastGcodeState() == self.dx.ERROR:
                    self.get_logger().warn(f'Move failed or timed out in hardware.')
                    response.success = False
                else:
                    self.get_logger().info(f'Move confirmed complete by hardware.')
                    response.success = True
                    
            except Exception as e:
                self.get_logger().error(f"Hardware communication failure: {e}")
                response.success = False
        else:
            self.get_logger().warn("Received empty G-code string.")
            response.success = False
            
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SendTS()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly disconnect the serial port when the node is killed
        if hasattr(node, 'dx'):
            node.dx.disconnect()
            node.get_logger().info("Hardware disconnected.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()




