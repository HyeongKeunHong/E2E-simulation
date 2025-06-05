import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Float32
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand

#---------------Variable Setting---------------
PUB_TOPIC_NAME = "topic_control_signal"

SUB_STEERING_TOPIC_NAME = '/regression/steering_output'
SUB_SPEED_TOPIC_NAME = '/regression/speed_output'


#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('e2e_motion_node')

        # 토픽 이름 설정
        self.sub_steering_topic = self.declare_parameter('sub_steering_topic', SUB_STEERING_TOPIC_NAME).value
        self.sub_speed_topic = self.declare_parameter('sub_lane_topic', SUB_SPEED_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        
        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(Float32, self.sub_steering_topic, self.steering_callback, self.qos_profile)
        self.path_sub = self.create_subscription(Float32, self.sub_speed_topic, self.speed_callback, self.qos_profile)
        
        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def steering_callback(self, msg: Float32):
        print(msg.data)
        self.steering_command = msg.data

    def speed_callback(self, msg: Float32):
        print(msg.data)
        self.left_speed_command = msg.data
        
    def timer_callback(self):

        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = round(self.steering_command)
        motion_command_msg.left_speed = round(self.left_speed_command)
        motion_command_msg.right_speed = round(self.right_speed_command)
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

