from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QMessageBox,
    QInputDialog,
)
from PyQt5.QtCore import Qt, QTimer, QObject, pyqtSignal
from PyQt5.QtGui import QPainter, QBrush, QPen, QColor
import sys
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32


class RobotStatusSubscriber(Node):
    

    def __init__(self):
        super().__init__('robot_status_subscriber')
        self.subscription = self.create_subscription(
            String, '/robot_status', self.status_callback, 10
        )
        self.current_status = None

        # table_num 퍼블리셔 생성
        self.table_num_publisher = self.create_publisher(Int32, 'table_num', 10)

    def publish_table_num(self, value):
        """table_num 값을 퍼블리시"""
        msg = Int32()
        msg.data = value
        self.table_num_publisher.publish(msg)
        self.get_logger().info(f"Published table_num: {value}")

    def status_callback(self, msg):
        """/robot_status 메시지 콜백"""
        self.get_logger().info(f"Received message: {msg.data}")
        self.current_status = msg.data


class RobotStatusHandler(QObject):
    

    status_updated = pyqtSignal(str)

    def __init__(self, ros2_node):
        super().__init__()
        self.ros2_node = ros2_node

    def check_status(self):
        
        rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
        if self.ros2_node.current_status:
            self.status_updated.emit(self.ros2_node.current_status)

    def publish_table_num(self, value):
        
        self.ros2_node.publish_table_num(value)


class EyeWidget(QWidget):
    """눈 깜박이는 애니메이션 구현"""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 100)
        self.blink_state = False
        self.is_center_mode = False  # 눈동자가 가운데에 고정되는 상태
        self.eye_position = 0  # 눈동자의 좌우 이동 (-1: 왼쪽, 1: 오른쪽)

        # 모드 변경 타이머 (4초마다 상태 전환)
        self.mode_timer = QTimer(self)
        self.mode_timer.timeout.connect(self.toggle_mode)
        self.mode_timer.start(4000)  # 4초마다 전환

        # 눈 깜박임 타이머
        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_blink)
        self.blink_timer.start(2000)  # 2초마다 깜박임

        # 눈동자 이동 타이머
        self.move_timer = QTimer(self)
        self.move_timer.timeout.connect(self.move_eye)
        self.move_timer.start(200)  # 0.2초마다 눈동자 이동

    def toggle_mode(self):
        """눈동자 모드 전환 (좌우 이동 ↔ 중앙 고정)"""
        self.is_center_mode = not self.is_center_mode
        if self.is_center_mode:
            self.eye_position = 0  # 중앙 고정
        self.update()

    def toggle_blink(self):
        """눈 깜박임 상태 전환"""
        self.blink_state = not self.blink_state
        self.update()  # 다시 그리기

    def move_eye(self):
        """눈동자 좌우 이동"""
        if not self.is_center_mode:
            self.eye_position += 0.1
            if self.eye_position > 1:  # 오른쪽 끝에서 방향 전환
                self.eye_position = -1
        self.update()  

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 눈 배경 
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.setPen(QPen(Qt.black, 2))
        width, height = self.width(), self.height()

        eye_width, eye_height = 70, 70  # 눈 크기 (동그랗게)
        eye_left_x = width // 2 - 90
        eye_right_x = width // 2 + 20
        eye_y = height // 2 - 35

        # 눈 그리기
        painter.drawEllipse(eye_left_x, eye_y, eye_width, eye_height)
        painter.drawEllipse(eye_right_x, eye_y, eye_width, eye_height)

        
        if not self.blink_state:
            # 눈동자
            pupil_width, pupil_height = 30, 30
            pupil_offset = int(self.eye_position * 20) if not self.is_center_mode else 0  # 이동량 계산
            pupil_y = eye_y + 20
            painter.setBrush(QBrush(QColor(0, 0, 255)))
            painter.drawEllipse(eye_left_x + 20 + pupil_offset, pupil_y, pupil_width, pupil_height)
            painter.drawEllipse(eye_right_x + 20 + pupil_offset, pupil_y, pupil_width, pupil_height)
        else:
            # 깜박임 상태
            painter.drawLine(eye_left_x, eye_y + eye_height // 2, eye_left_x + eye_width, eye_y + eye_height // 2)
            painter.drawLine(eye_right_x, eye_y + eye_height // 2, eye_right_x + eye_width, eye_y + eye_height // 2)


class ServingRobotGUI(QWidget):
    """PyQt5 GUI: 배달 상태와 버튼 인터페이스 제공"""

    def __init__(self, ros_handler):
        super().__init__()
        self.ros_handler = ros_handler
        self.ros_handler.status_updated.connect(self.update_status)

        self.setWindowTitle("Serving Robot GUI")
        self.setGeometry(100, 100, 400, 300)

        
        self.main_layout = QVBoxLayout()

       
        self.eye_widget = EyeWidget()
        self.main_layout.addWidget(self.eye_widget)

        self.status_label = QLabel("현재 상태: 대기중")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.status_label)

        # 배달 메시지 표시 라벨 (숨김 상태)
        self.delivery_message_label = QLabel()
        self.delivery_message_label.setAlignment(Qt.AlignCenter)
        self.delivery_message_label.setVisible(False)
        self.main_layout.addWidget(self.delivery_message_label)

        
        self.button_layout = QHBoxLayout()
        self.confirm_button = QPushButton("배달 확인")
        self.reassign_button = QPushButton("다른 테이블로 배달 보내기")

        
        self.confirm_button.clicked.connect(self.confirm_delivery)
        self.reassign_button.clicked.connect(self.reassign_delivery)

        # 메시지 또는 퀴즈가 한 번만 표시
        self.message_displayed = False

        self.setLayout(self.main_layout)
        self.show()

    def update_status(self, status):
        
        self.status_label.setText(f"현재 상태: {status}")

        if status == "대기중":
            # 초기화
            self.delivery_message_label.setVisible(False)
            self.message_displayed = False
            self.hide_buttons()  
            return

        if status == "이동 완료":
            
            if not self.message_displayed:
                self.delivery_message_label.setText("배달이 도착했습니다!")
                self.delivery_message_label.setVisible(True)
                QTimer.singleShot(3000, self.delivery_message_label.hide)
                QTimer.singleShot(3000, self.display_random_message)
                self.message_displayed = True  # 메시지 표시 상태로 변경
                
    def hide_buttons(self):
        
        self.button_layout.removeWidget(self.confirm_button)
        self.button_layout.removeWidget(self.reassign_button)
        self.confirm_button.setParent(None)
        self.reassign_button.setParent(None)


    def display_random_message(self):
        """랜덤으로 일반 메시지 또는 퀴즈 메시지 표시"""
        if random.random() < 0.5:
            self.show_quiz_message()
        else:
            self.show_general_message()

        self.show_buttons()

    def show_general_message(self):
        """일반 메시지 표시"""
        general_messages = [
            "감사합니다 다음에도 주문해주세요^_^!",
            "조금만 더 힘내요! 제가 늘 곁에 있어요!",
            "날씨가 쌀쌀하네요, 감기 조심하세요!",
            "힘든 하루였나요? 저의 배달로 위로가 되었으면 좋겠어요!",
            "천천히 여유 있게 드세요, 그게 제일 맛있답니다!",
            "하루가 조금 바빴다면 잠깐 쉬어가도 좋아요!",
            "오늘 하루도 대단했어요! 좀 더 힘내봐요!",
            "제가 그리웠나요? 맛있게 즐기고 에너지 충전하세요!",
            "주문 감사해요, 또 불러주시면 언제든 달려올게요!",
            "귀한 분께 배달 드릴 수 있어 영광입니다 ^_^!"
        ]
        message = random.choice(general_messages)
        QMessageBox.information(self, "배달 완료", message)

    def show_quiz_message(self):
        """퀴즈 메시지 표시"""
        quizzes = [
            {"question": "퀴즈: 로봇의 눈 색깔은(oo색)?", "answer": "파란색"},
            {"question": "퀴즈: 몽골의 수도는?", "answer": "울란바토르"},
            {"question": "8월 15일은 무슨 기념일일까요?", "answer": "광복절"},
        ]
        quiz = random.choice(quizzes)
        self.current_quiz_answer = quiz["answer"]
        QMessageBox.information(self, "퀴즈 도전!", quiz["question"])

        answer, ok = QInputDialog.getText(self, "퀴즈 답변", "정답을 입력하세요:")
        if ok:
            if answer == self.current_quiz_answer:
                QMessageBox.information(self, "정답!", "축하합니다 정답입니다! 무료 쿠폰 1개를 드려요")
            else:
                QMessageBox.warning(self, "오답", "다음 기회에 도전하세요!")

    def show_buttons(self):
        
        self.button_layout.addWidget(self.confirm_button)
        self.button_layout.addWidget(self.reassign_button)
        self.main_layout.addLayout(self.button_layout)

    def confirm_delivery(self):
        """배달 확인 처리"""
        QMessageBox.information(self, "확인", "배달 확인이 되었습니다 맛있게 드세요")
        self.ros_handler.publish_table_num(0)  # table_num 토픽에 0 퍼블리시 - 원래 위치로 복귀
        QTimer.singleShot(3000)
        self.message_displayed = False  

    def reassign_delivery(self):
        """다른 테이블로 배달"""
        items = ["1", "2", "3", "4", "5", "6", "7", "8", "9"]
        table_num, ok = QInputDialog.getItem(self, "테이블 선택", "배달할 테이블을 선택하세요:", items, 0, False)
        if ok:
            selected_num = int(table_num)
            QMessageBox.information(self, "다른 테이블로", f"{selected_num}번 테이블로 배달을 보냅니다.")
            self.ros_handler.publish_table_num(selected_num)  # 선택한 테이블 번호를 퍼블리시
            QTimer.singleShot(3000, lambda: setattr(self, "message_displayed", False))
            
def main():
    rclpy.init()

 
    ros2_node = RobotStatusSubscriber()
    ros_handler = RobotStatusHandler(ros2_node)

   
    app = QApplication(sys.argv)
    gui = ServingRobotGUI(ros_handler)

   
    timer = QTimer()
    timer.timeout.connect(ros_handler.check_status)
    timer.start(100)

    try:
        sys.exit(app.exec_())
    finally:
        ros2_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()