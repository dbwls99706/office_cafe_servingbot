import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
import math

class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor')
        
        self.current_position = None
        self.goal_position = None
        self.tolerance = 0.2  # 목표 위치 근처로 간주할 허용 오차
        self.status = "대기중"
        self.previous_status = None

        # /plan 토픽 구독 (목표 위치)
        self.plan_subscription = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10)

        # /odom 토픽 구독 (현재 위치)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # 상태 업데이트 타이머
        self.timer = self.create_timer(1.0, self.update_status)

        # status 퍼블리셔 생성
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)

    def plan_callback(self, msg):
        # 경로의 마지막 지점을 목표 위치로 설정
        if msg.poses:
            self.goal_position = msg.poses[-1].pose.position

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def update_status(self):
        # 초기 대기 위치 확인 (예: x=-2, y=-0.5 주변에 있을 경우 "대기중"으로 설정)
        if self.current_position and \
           (self.current_position.x <= -1.9 and self.current_position.x >= -2.1) and \
           (self.current_position.y <= -0.4 and self.current_position.y >= -0.6):
            self.status = "대기중"

        # 목표 위치와 현재 위치 간의 거리 계산  
        elif self.current_position and self.goal_position:
            distance_to_goal = math.sqrt(
                (self.current_position.x + 2.0 - self.goal_position.x) ** 2 +
                (self.current_position.y + 0.5 - self.goal_position.y) ** 2
            )

            # 목표 위치 근처에 있을 경우 "이동 완료"
            if distance_to_goal < self.tolerance:
                self.status = "이동 완료"
            else:
                self.status = "이동중"

        # 상태 출력 및 퍼블리싱
        if self.status != self.previous_status:
            self.get_logger().info(f"현재 상태: {self.status}")
            status_msg = String()
            status_msg.data = self.status
            self.status_publisher.publish(status_msg)
            self.previous_status = self.status  # 상태 업데이트
        
        

def main(args=None):
    rclpy.init(args=args)
    robot_status_monitor = RobotStatusMonitor()
    rclpy.spin(robot_status_monitor)
    robot_status_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()