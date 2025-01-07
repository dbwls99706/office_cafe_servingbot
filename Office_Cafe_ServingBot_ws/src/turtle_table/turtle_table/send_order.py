#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_interfaces.srv import MySrv  # 정의한 MySrv 서비스 사용

class SendOrderServer(Node):
    def __init__(self):
        super().__init__('service_server')
        
       
        self.srv = self.create_service(MySrv, 'check_order', self.handle_check_order)

        # 주문 데이터를 수신
        self.subscription = self.create_subscription(
            String,
            'table_order',
            self.listener_callback,
            10
        )
        self.order_data = None  # 수신한 주문 데이터 저장
    
    def listener_callback(self, msg):
        # table_order 데이터를 수신했을 때 실행되는 콜백 함수
        self.order_data = msg.data
        self.get_logger().info(f'Received order data: {self.order_data}')
    
    def handle_check_order(self, request, response):
        # 클라이언트의 요청 처리
        if self.order_data:
            response.success = True  # 주문 데이터가 수신 되었을 때 
            response.message = self.order_data  # 현재 저장된 주문 데이터 반환
        else:
            response.success = False  # 주문 데이터가 수신되지 않았을 때 
            response.message = "아직 주문 데이터가 수신되지 않았습니다."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SendOrderServer()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass 
    finally:
        node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
