import sys
import re
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QMessageBox
from PyQt5.QtGui import QPixmap, QPainter, QPen, QTransform, QFont
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32

class ROS2Thread(QThread):
    position_updated = pyqtSignal(object)
    top_3_updated = pyqtSignal(str)
    total_revenue_updated = pyqtSignal(int)
    order_data_updated = pyqtSignal(str)
    robot_status_updated = pyqtSignal(str)
    order_start_updated = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.table_num_publisher = None
        self.status_publisher = None

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('gui_subscriber')
        self.table_num_publisher = self.node.create_publisher(Int32, 'table_num', 10)
        self.status_publisher = self.node.create_publisher(String, 'order_status', 10)

        # 로봇 정보를 받기 위한 수신 데이터 종류 
        self.node.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.node.create_subscription(String, 'top_3_drinks', self.top_3_callback, 10)
        self.node.create_subscription(Int32, 'total_revenue', self.total_revenue_callback, 10)
        self.node.create_subscription(String, 'table_order', self.order_data_callback, 10)
        self.node.create_subscription(String, 'robot_status', self.robot_status_callback, 10)
        self.node.create_subscription(String, 'order_start', self.order_start_callback, 10)
        
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def robot_status_callback(self, msg):
        self.robot_status_updated.emit(msg.data)

    def order_start_callback(self, msg):
        self.order_start_updated.emit(msg.data)

    def publish_table_num(self, table_num):
        msg = Int32()
        msg.data = table_num
        self.table_num_publisher.publish(msg)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def odom_callback(self, msg):
        self.position_updated.emit(msg.pose.pose.position)

    def top_3_callback(self, msg):
        self.top_3_updated.emit(msg.data)

    def total_revenue_callback(self, msg):
        self.total_revenue_updated.emit(msg.data)

    def order_data_callback(self, msg):
        self.order_data_updated.emit(msg.data)

class RobotPositionMap(QLabel):
    def __init__(self):
        super().__init__()
        self.scale_factor = 4.0
        original_pixmap = QPixmap("/home/hyuna/map/map.pgm")
        transform = QTransform().rotate(0)
        rotated_pixmap = original_pixmap.transformed(transform)
        self.map_pixmap = rotated_pixmap.scaled(
            int(rotated_pixmap.width() * self.scale_factor),
            int(rotated_pixmap.height() * self.scale_factor),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )

        self.setPixmap(self.map_pixmap)
        self.gazebo_x_min, self.gazebo_x_max = -3.0, 3.0
        self.gazebo_y_min, self.gazebo_y_max = -3.0, 3.0
        self.offset_x, self.offset_y = 20, 5
        self.map_width = self.map_pixmap.width()
        self.map_height = self.map_pixmap.height()

    def update_position(self, position):
        self.draw_robot_position(position.x, position.y)

    def draw_robot_position(self, x, y):
        pixmap = self.map_pixmap.copy()
        painter = QPainter(pixmap)
        pen = QPen(Qt.red, 5)
        painter.setPen(pen)
        map_x = int((x - self.gazebo_x_min) / (self.gazebo_x_max - self.gazebo_x_min) * self.map_width) + self.offset_x
        map_y = int((self.gazebo_y_max - y) / (self.gazebo_y_max - self.gazebo_y_min) * self.map_height) + self.offset_y
        painter.drawPoint(map_x, map_y)
        painter.end()
        self.setPixmap(pixmap)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("주방 디스플레이")
        self.setGeometry(100, 100, 1300, 600)
        self.setFixedSize(1300, 600)

        self.ros_thread = ROS2Thread()
        main_widget = QWidget()
        main_layout = QHBoxLayout(main_widget)

        left_widget = QWidget()
        left_layout = QGridLayout()
        left_widget.setLayout(left_layout)

        self.order_widgets = {}
        for i in range(9):
            order_widget = OrderWidget(i + 1, self.ros_thread)
            left_layout.addWidget(order_widget, i // 3, i % 3)
            self.order_widgets[i + 1] = order_widget

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        self.large_area = RobotPositionMap()
        self.large_area.setStyleSheet("background-color: lightgray;")
        self.large_area.setAlignment(Qt.AlignCenter)
        self.large_area.setMinimumHeight(400)

        bottom_widget = QWidget()
        bottom_layout = QHBoxLayout(bottom_widget)

        self.small_area_1 = QLabel("현재 로봇 상태")
        self.small_area_1.setStyleSheet("background-color: lightblue;")
        self.small_area_1.setAlignment(Qt.AlignCenter)

        self.small_area_2 = QWidget()
        self.small_area_2_layout = QVBoxLayout(self.small_area_2)
        self.top_3_label = QLabel("Top 3 메뉴: -")
        self.top_3_label.setFont(QFont("Arial", 8))
        self.top_3_label.setAlignment(Qt.AlignCenter)
        self.revenue_label = QLabel("총 판매액: - 원")
        self.revenue_label.setFont(QFont("Arial", 8))
        self.revenue_label.setAlignment(Qt.AlignCenter)
        self.small_area_2_layout.addWidget(self.top_3_label)
        self.small_area_2_layout.addWidget(self.revenue_label)
        self.small_area_2.setStyleSheet("background-color: lightgreen;")

        bottom_layout.addWidget(self.small_area_1)
        bottom_layout.addWidget(self.small_area_2)

        right_layout.addWidget(self.large_area)
        right_layout.addWidget(bottom_widget)

        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 2)

        self.setCentralWidget(main_widget)

        self.ros_thread.position_updated.connect(self.large_area.update_position)
        self.ros_thread.top_3_updated.connect(self.update_top_3)
        self.ros_thread.total_revenue_updated.connect(self.update_total_revenue)
        self.ros_thread.order_data_updated.connect(self.update_order_data)  # 주문 데이터 연결
        self.ros_thread.robot_status_updated.connect(self.update_robot_status)  # Connect to robot status update
        self.ros_thread.order_start_updated.connect(self.show_order_start_popup)
        self.ros_thread.start()

    def update_top_3(self, top_3_data):
        items = re.sub(r"\s*\([^)]*\)", "", top_3_data).split(",")
        items = [item.strip() for item in items if item.strip()]
        formatted_menu = "\n".join(f"{rank+1}위: {item}" for rank, item in enumerate(items[:3]))
        self.top_3_label.setText(f"Top 3 메뉴:\n{formatted_menu}")

    def update_total_revenue(self, total_revenue):
        self.revenue_label.setText(f"총 판매액: {total_revenue}원")

    def update_order_data(self, data):
        orders = self.parse_order_data(data)
        for table_num, items in orders.items():
            if table_num in self.order_widgets:
                self.order_widgets[table_num].update_order_button_text(items)

    def show_order_start_popup(self, order_text):
        QMessageBox.information(self, "Order Start", f"{order_text}")

    def parse_order_data(self, data):
        parsed_orders = {}
        lines = data.split("\n")
        current_table = None
        
        for line in lines:
            line = line.strip()
            if line.startswith("테이블 번호:"):
                try:
                    # 테이블 번호 추출 후 current_table 갱신
                    current_table = int(line.split(":")[1].strip())
                    if current_table not in parsed_orders:
                        parsed_orders[current_table] = []
                except ValueError:
                    # 테이블 번호를 변환할 수 없는 경우, 로그를 찍고 계속 진행
                    self.get_logger().error(f"잘못된 테이블 번호 형식: {line}")
                    continue
            elif "- " in line:
                if current_table is None:
                    print(f"테이블 번호가 지정되지 않았습니다: {line}")
                    continue 
                parsed_orders[current_table].append(line)
            else:
                print(f"건너뛰는 라인: {line} (유효한 테이블 번호나 항목 없음)")
        
        
        print(f"파싱된 주문 데이터: {parsed_orders}") # 디버깅 
        return parsed_orders

    def update_robot_status(self, status):
        self.small_area_1.setText(f"현재 로봇 상태: {status}")

class OrderWidget(QWidget):
    def __init__(self, order_id, ros_thread, parent=None):
        super().__init__(parent)
        self.order_id = order_id
        self.ros_thread = ros_thread
        self.robot_status = "대기중" 

        self.main_layout = QVBoxLayout()

        self.default_text = f"Table {self.order_id}\n({self.order_id - 6}층 픽업)" if self.order_id in [7, 8, 9] else f"Table {self.order_id}"
        self.order_button = QPushButton(self.default_text)
        self.order_button.setFixedSize(170, 110)
        self.order_button.setStyleSheet("font-size: 12px;")
        self.order_button.clicked.connect(self.show_order_options)

        self.main_layout.addWidget(self.order_button)

        self.extra_buttons_layout = QHBoxLayout()
        self.confirm_button = QPushButton("주문확인")
        self.cancel_button = QPushButton("주문취소")
        self.confirm_button.setStyleSheet("font-size: 11px;")
        self.cancel_button.setStyleSheet("font-size: 11px;")
        self.confirm_button.clicked.connect(self.confirm_order)
        self.cancel_button.clicked.connect(self.show_cancel_reasons)
        self.extra_buttons_layout.addWidget(self.confirm_button)
        self.extra_buttons_layout.addWidget(self.cancel_button)

        self.cancel_reasons_layout = QVBoxLayout()
        for reason in ["품절", "시스템점검", "가게사정"]:
            button = QPushButton(reason)
            button.setStyleSheet("font-size: 10px;")  
            button.setFixedSize(170, 15)  
            button.clicked.connect(lambda _, r=reason: self.cancel_order(r))
            self.cancel_reasons_layout.addWidget(button)

        self.main_layout.addLayout(self.extra_buttons_layout)
        self.main_layout.addLayout(self.cancel_reasons_layout)

        self.setLayout(self.main_layout)
        self.toggle_extra_buttons(False)
        self.toggle_cancel_reasons(False)

        
        self.ros_thread.robot_status_updated.connect(self.update_robot_status)

    def update_robot_status(self, status):
        self.robot_status = status  # 현재 로봇 상태 업데이트

    def show_order_options(self):
        is_visible = not self.confirm_button.isVisible()
        self.toggle_extra_buttons(is_visible)
        self.toggle_cancel_reasons(False)

    def toggle_extra_buttons(self, show):
        self.confirm_button.setVisible(show)
        self.cancel_button.setVisible(show)

    def toggle_cancel_reasons(self, show):
        for i in range(self.cancel_reasons_layout.count()):
            self.cancel_reasons_layout.itemAt(i).widget().setVisible(show)

    def confirm_order(self):
        print(f"Table {self.order_id} 주문접수")
        self.ros_thread.publish_status("조리 중")  
        self.confirm_button.setText("조리완료")
        self.confirm_button.clicked.disconnect()
        self.confirm_button.clicked.connect(self.complete_cooking)

    def complete_cooking(self):
        if self.robot_status != "대기중":
            QMessageBox.warning(self, "로봇 상태", "서빙로봇이 사용 중입니다.")
            self.get_logger().warning(f'서빙 로봇이 사용 중입니다.')
            return

        print(f"Table {self.order_id} 조리완료")
        self.ros_thread.publish_status("조리완료")

        self.ros_thread.publish_table_num(self.order_id)
        QMessageBox.information(self, "조리 완료", f"서빙로봇이 출발합니다: Table {self.order_id}")

        self.order_button.setText(self.default_text)
        self.toggle_extra_buttons(False)
        self.toggle_cancel_reasons(False)

        self.confirm_button.setText("주문확인")
        self.confirm_button.clicked.disconnect()
        self.confirm_button.clicked.connect(self.confirm_order)

    def show_cancel_reasons(self):
        self.toggle_cancel_reasons(not self.cancel_reasons_layout.itemAt(0).widget().isVisible())

    def cancel_order(self, reason):
        print(f"Table {self.order_id} 취소사유: {reason}")
        cancel_message = f"{self.order_id}번 테이블 주문취소, 사유: {reason}"
        self.ros_thread.publish_status(cancel_message) 

        self.order_button.setText(self.default_text)
        self.toggle_extra_buttons(False)
        self.toggle_cancel_reasons(False)

    def update_order_button_text(self, items):
        order_text = f"Table {self.order_id}\n" + "\n".join(items)
        self.order_button.setText(order_text)

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
