#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtle_interfaces.srv import MySrv  # 정의한 MySrv 서비스 임포트

class CheckOrderClient(Node):
    def __init__(self):
        super().__init__('service_client')
        
        # 서비스 클라이언트 
        self.client = self.create_client(MySrv, 'check_order')
        
        # table_order  수신
        self.subscription = self.create_subscription(
            String,
            'table_order',
            self.listener_callback,
            10
        )
        self.order_data = None  

        
        self.response_publisher = self.create_publisher(String, 'order_start', 10)

    def listener_callback(self, msg):
        
        self.order_data = msg.data  
        self.get_logger().info(f'Received order data for confirmation: {self.order_data}')
        self.check_order_match()  # 수신된 데이터와 일치하는지 확인

    def check_order_match(self):
        # 서비스 요청 
        if self.client.wait_for_service(timeout_sec=1.0):
            request = MySrv.Request() 
            request.table_number = int(self.extract_table_number(self.order_data))  # 테이블 번호 추출

            future = self.client.call_async(request)  
            future.add_done_callback(self.handle_response)  
        else:
            self.get_logger().error('Service not available')  # 서비스 사용 불가 시 에러 로그 

    def handle_response(self, future):
        # 서비스 응답
        response_msg = String()
        try:
            response = future.result()
            if response.success and response.message == self.order_data:
                confirmation_message = '테이블과 주문이 일치합니다. 조리를 시작하세요!!!'
                self.get_logger().info(confirmation_message)
                response_msg.data = confirmation_message
            else:
                warning_message = '테이블과 주문이 일치하지 않습니다. 주문을 취소해주세요!!!'
                self.get_logger().warning(warning_message)
                response_msg.data = warning_message

            self.response_publisher.publish(response_msg)

        except Exception as e: # 주문 수신에 실패 했을 때 
            error_message = f'Service call failed: {str(e)}'
            self.get_logger().error(error_message)
            response_msg.data = error_message
            self.response_publisher.publish(response_msg)

    def extract_table_number(self, data):
        # 테이블 번호를 추출
        import re
        match = re.search(r'테이블 번호:\s*(\d+)', data)
        return match.group(1) if match else '0'

def main(args=None):
    rclpy.init(args=args)
    node = CheckOrderClient()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
