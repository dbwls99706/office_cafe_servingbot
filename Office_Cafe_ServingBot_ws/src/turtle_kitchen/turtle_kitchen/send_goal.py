import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class TableNumberSubscriber(Node):

    def __init__(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        super().__init__('table_number_subscriber')
        self.initial_pose_set = False  # 초기 위치 설정 여부를 추적

        self.subscription = self.create_subscription(
            Int32,
            'table_num',
            self.listener_callback,
            10
        )

        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        # 노드가 시작될 때 initial pose 설정 
        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')


        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):

            self.get_logger().info('Service /set_initial_pose not available, waiting again...')

        self.set_initial_pose(0.0,0.0,0.0,1.0)

    def set_initial_pose(self, x,y,z,w):

        req = SetInitialPose.Request()

        req.pose.header.frame_id = 'map'

        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)

        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)

        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1,

                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

                              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,

                              0.0, 0.0, 0.0, 0.01, 0.0, 0.0,

                              0.0, 0.0, 0.0, 0.0, 0.01, 0.0,

                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]



        future = self.set_initial_pose_service_client.call_async(req)
        return future.result()


    def listener_callback(self, msg):
        self.get_logger().info('Received table number: %d' % msg.data)
        
        # 목표 위치 설정 
        goal_pose = self.create_goal_pose(msg.data)
        
        # NAV2에 목표 위치 전송
        self.send_goal(goal_pose)


    def create_goal_pose(self, table_num):
        # 테이블 번호에 따라 목표 위치 설정
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        
        if table_num == 0: #초기 위치
            goal_pose.pose.position.x = 0.0
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.orientation.w = 1.0  
            goal_pose.pose.orientation.z = 0.0  
        # 필요에 따라 다른 테이블 번호의 목표 위치 추가

        elif table_num == 1:
            goal_pose.pose.position.x = 0.55
            goal_pose.pose.position.y = 1.20
            goal_pose.pose.orientation.w = 0.9239  # 방향 설정 (왼쪽 위 45도 각도)
            goal_pose.pose.orientation.z = 0.3827


        elif table_num == 2:
            goal_pose.pose.position.x = 1.63
            goal_pose.pose.position.y = 1.20
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        elif table_num == 3:
            goal_pose.pose.position.x = 2.74
            goal_pose.pose.position.y = 1.20
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        elif table_num == 4:
            goal_pose.pose.position.x = 0.55
            goal_pose.pose.position.y = 0.10
            goal_pose.pose.orientation.w = 0.9239 
            goal_pose.pose.orientation.z = 0.3827
        
        elif table_num == 5:
            goal_pose.pose.position.x = 1.63
            goal_pose.pose.position.y = 0.10
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        elif table_num == 6:
            goal_pose.pose.position.x = 2.74
            goal_pose.pose.position.y = 0.10
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        elif table_num == 7:
            goal_pose.pose.position.x = 0.55
            goal_pose.pose.position.y = -0.99
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        elif table_num == 8:
            goal_pose.pose.position.x = 1.63
            goal_pose.pose.position.y = -0.99
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        elif table_num == 9:
            goal_pose.pose.position.x = 2.74    
            goal_pose.pose.position.y = -0.99
            goal_pose.pose.orientation.w = 0.9239  
            goal_pose.pose.orientation.z = 0.3827

        return goal_pose

    def send_goal(self, pose):
        
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NAV2 액션 서버를 찾을 수 없습니다.")
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal pose...')
        self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Received feedback: {feedback.feedback}')

def main(args=None):
    rclpy.init(args=args)
    table_number_subscriber = TableNumberSubscriber()
    
    try:
        rclpy.spin(table_number_subscriber)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
    finally:
        table_number_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
