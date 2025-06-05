import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import torch
import torchvision.transforms as transforms
import cv2
from torchvision.models import resnet18
import torch.nn as nn
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
PUB_TOPIC_NAME = "topic_control_signal"

class ResNet18RegressionNode(Node):
    def __init__(self):
        super().__init__('resnet18_regression_node')

        # Subscribe to image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )



        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # Publishers for left and right regression outputs
        self.publisher_steering = self.create_publisher(Float32, '/regression/steering_output', 10)
        self.publisher_speed = self.create_publisher(Float32, '/regression/speed_output', 10)
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        self.bridge = CvBridge()

        # Load ResNet18 model
        self.model = resnet18(weights='IMAGENET1K_V1')

        # Modify the last layer to output two values for regression (instead of one)
        self.model.fc = nn.Linear(self.model.fc.in_features, 2)  # Change the output layer to 2

        # Load your trained weights (model for regression)
        self.model.load_state_dict(torch.load('model_250405.pt'))
        self.model.eval()

        # Image preprocessing
        self.preprocess = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225])
        ])

        self.get_logger().info("ResNet18 regression node initialized.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image
            input_tensor = self.preprocess(cv_image)
            input_batch = input_tensor.unsqueeze(0)  # Add batch dimension

            # Perform inference
            with torch.no_grad():
                output = self.model(input_batch)

            # Debugging: Print the shape of the output tensor
            self.get_logger().info(f"Output tensor shape: {output.shape}")

            # Ensure output is a 1D tensor with 2 elements
            if output.ndimension() > 1:
                output = output.squeeze()  # Remove extra dimensions
                self.get_logger().info(f"Output tensor after squeeze: {output.shape}")

            # Check if we have two elements in the output
            if output.ndimension() == 1 and output.shape[0] == 2:
                steering_output = output[0].item()  # First element (left)
                speed_output = output[1].item()  # Second element (right)

                # # Publish the left and right regression results
                # steering_msg = Float32()
                # steering_msg.data = steering_output
                # self.publisher_steering.publish(steering_msg)

                # speed_msg = Float32()
                # speed_msg.data = speed_output
                # self.publisher_speed.publish(speed_msg)

                # 모션 명령 메시지 생성 및 퍼블리시
                motion_command_msg = MotionCommand()
                motion_command_msg.steering = round(steering_output)
                motion_command_msg.left_speed = round(speed_output)
                motion_command_msg.right_speed = round(speed_output)
                self.publisher.publish(motion_command_msg)



                self.get_logger().info(f"Published steering output: {steering_output:.4f}")
                self.get_logger().info(f"Published speed output: {speed_output:.4f}")

            else:
                raise ValueError(f"Unexpected output shape: {output.shape}")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ResNet18RegressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
