import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/tracking',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            x = detection.bbox.center.position.x
            y = detection.bbox.center.position.y
            self.get_logger().info(
                f'ID: {detection.id} | Class: {detection.class_name} | '
                f'Score: {detection.score:.2f} | Position: ({x:.1f}, {y:.1f})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()