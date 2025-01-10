#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import sqlite3
import re
import os
from collections import defaultdict

class TableOrderSubscriber(Node):

    def __init__(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        super().__init__('tableorder_subscriber')

        
        self.subscription = self.create_subscription(
            String,
            'table_order',  
            self.listener_callback,  
            qos_profile
        )
        self.subscription 

        # 테이블 번호 발행
        self.table_num_publisher = self.create_publisher(Int32, 'table_number', 10)
        self.top_3_publisher = self.create_publisher(String, 'top_3_drinks', 10)
        self.total_revenue_publisher = self.create_publisher(Int32, 'total_revenue', 10)

        # 데이터베이스 파일 경로 설정
        db_path = 'orders.db'

        
        if os.path.exists(db_path):
            os.remove(db_path)
            self.get_logger().info('Deleted existing database file.')

       
        self.conn = sqlite3.connect(db_path)
        self.cursor = self.conn.cursor()
        
        # 테이블 생성 
        self.cursor.execute('''CREATE TABLE orders
                               (id INTEGER PRIMARY KEY AUTOINCREMENT,
                                menu_name TEXT,
                                price INTEGER,
                                options TEXT,  -- 옵션을 저장할 컬럼 추가
                                table_num INTEGER)''')
        self.conn.commit()
        self.get_logger().info('Created new orders table.')

        
        self.menu_sales = defaultdict(int)  # 메뉴별 판매 수
        self.menu_revenue = defaultdict(int)  # 메뉴별 판매 금액

    def listener_callback(self, msg):
        # 수신한 주문 데이터 출력 
        self.get_logger().info(f"Received order data:\n{msg.data}")
        
        
        lines = msg.data.split('\n')
        menu_items = []
        total_amount = None
        table_num = None

        for line in lines:
            if '주문확인 - 합계:' in line:
                total_amount = int(re.sub(r'[^\d]', '', line.split(':')[1]))
            elif '테이블 번호:' in line:
                table_num = int(re.search(r'\d+', line).group())
            elif '- ' in line:  # 메뉴와 가격 구분 
                match = re.search(r'(.+?) - ([\d,]+)원', line)
                if match:
                    # 메뉴 이름에서 괄호와 괄호 속 내용 제거
                    menu_name = re.sub(r'\(.*\)', '', match.group(1)).strip()
                    price = int(re.sub(r'[^\d]', '', match.group(2)))
                    
                    # 괄호 안의 내용 추출
                    options = re.search(r'\((.*?)\)', match.group(1))
                    options = options.group(1) if options else None
                    
                    menu_items.append((menu_name, price, options))

        if menu_items and table_num:
            try:
                
                for menu_name, price, options in menu_items:
                    # 데이터베이스에 저장 (options도 함께)
                    self.cursor.execute("INSERT INTO orders (menu_name, price, options, table_num) VALUES (?, ?, ?, ?)",
                                        (menu_name, price, options, table_num))
                    
                    # 메뉴별 판매 수 및 금액 추적
                    self.menu_sales[menu_name] += 1
                    self.menu_revenue[menu_name] += price
                
                self.conn.commit()
                self.get_logger().info(f"Saved to database: {len(menu_items)} items, total {total_amount}원, 테이블 {table_num}")
            except sqlite3.Error as e:
                self.get_logger().error(f"Database error: {e}")

            # 테이블 번호 발행
            table_num_msg = Int32()
            table_num_msg.data = table_num
            self.table_num_publisher.publish(table_num_msg)
            self.get_logger().info(f"Published table number: {table_num}")

            # 가장 많이 팔린 음료 Top 3
            top_3 = sorted(self.menu_sales.items(), key=lambda x: x[1], reverse=True)[:3]
            self.get_logger().info(f"Top 3 best-selling drinks: {top_3}")

            # Top 3 정보를 publish
            top_3_msg = String()
            top_3_msg.data = ', '.join([f"{item[0]} ({item[1]})" for item in top_3])
            self.top_3_publisher.publish(top_3_msg)

            # 전체 판매 금액
            total_revenue = sum(self.menu_revenue.values())
            self.get_logger().info(f"Total revenue: {total_revenue}원")

            # 전체 판매 금액을 publish
            total_revenue_msg = Int32()
            total_revenue_msg.data = total_revenue
            self.total_revenue_publisher.publish(total_revenue_msg)

    def __del__(self):
        # 노드 종료 시 데이터베이스 연결 닫기
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)
    tableorder_subscriber = TableOrderSubscriber()
    
    try:
        rclpy.spin(tableorder_subscriber)
    except KeyboardInterrupt:
        print('Stopped by keyboard interrupt')
    finally:
        
        tableorder_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
