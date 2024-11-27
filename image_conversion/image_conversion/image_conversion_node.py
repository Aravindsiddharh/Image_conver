import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import SetBool


class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')
        self.declare_parameter('input_image_topic', '/image_raw')
        self.declare_parameter('output_image_topic', '/converted_image')

        self.input_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.mode = 1  # Default mode: Greyscale

        self.image_sub = self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.mode_service = self.create_service(SetBool, 'set_mode', self.set_mode_callback)

        self.get_logger().info(f"Subscribed to: {self.input_topic}, Publishing to: {self.output_topic}")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply conversion based on mode
            if self.mode == 1:
                converted_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                ros_image = self.bridge.cv2_to_imgmsg(converted_image, encoding='mono8')
            else:
                ros_image = msg  # Publish unmodified image in Mode 2

            # Publish the processed image
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def set_mode_callback(self, request, response):
        if request.data:  # True -> Mode 1: Greyscale
            self.mode = 1
            response.message = "Mode changed to Greyscale"
        else:  # False -> Mode 2: Color
            self.mode = 2
            response.message = "Mode changed to Color"
        response.success = True
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

