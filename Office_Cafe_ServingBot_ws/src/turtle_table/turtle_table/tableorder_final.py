#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                            QLabel, QRadioButton, QGridLayout, QDialog, QListWidget, QButtonGroup,
                            QMessageBox)
from PyQt5.QtGui import QFont, QPalette, QColor
from PyQt5.QtCore import Qt

class TableSelectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('주문 방식 선택')
        self.selected_table = None
        self.order_type = None
        
        # Center the dialog relative to parent
        if parent:
            geometry = self.frameGeometry()
            center = parent.frameGeometry().center()
            geometry.moveCenter(center)
            self.move(geometry.topLeft())
            
        self.setStyleSheet("""
            QDialog {
                background-color: #ffffff;
            }
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: #00704A;
                padding: 10px 0;
            }
            QPushButton {
                font-size: 20px;
                padding: 20px;
                border: 2px solid #00704A;
                border-radius: 10px;
                min-width: 150px;
                min-height: 80px;
            }
            QPushButton:hover {
                background-color: #00704A;
                color: white;
            }
        """)
        
        layout = QVBoxLayout()
        
        # Add instruction label
        instruction_label = QLabel('주문 방식을 선택해주세요')
        instruction_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(instruction_label)
        
        # Create buttons for delivery and pickup
        btn_layout = QHBoxLayout()
        delivery_btn = QPushButton('배달')
        pickup_btn = QPushButton('픽업')
        
        delivery_btn.clicked.connect(lambda: self.show_table_selection('배달'))
        pickup_btn.clicked.connect(lambda: self.show_table_selection('픽업'))
        
        btn_layout.addWidget(delivery_btn)
        btn_layout.addWidget(pickup_btn)
        layout.addLayout(btn_layout)
        
        self.setLayout(layout)


    def show_table_selection(self, order_type):
        table_dialog = TableNumberDialog(order_type, self)
        if table_dialog.exec_():
            self.selected_table = table_dialog.selected_table
            self.order_type = order_type
            self.accept()

class TableNumberDialog(QDialog):
    def __init__(self, order_type, parent=None):
        super().__init__(parent)
        self.setWindowTitle('테이블 선택')
        self.selected_table = None
        
        # Center the dialog
        if parent:
            geometry = self.frameGeometry()
            center = parent.frameGeometry().center()
            geometry.moveCenter(center)
            self.move(geometry.topLeft())
            
        self.setStyleSheet("""
            QDialog {
                background-color: #ffffff;
            }
            QLabel {
                font-size: 16px;
                font-weight: bold;
                color: #00704A;
                padding: 10px 0;
            }
            QPushButton {
                font-size: 20px;
                padding: 20px;
                border: 2px solid #00704A;
                border-radius: 10px;
                min-width: 80px;
                min-height: 80px;
            }
            QPushButton:hover {
                background-color: #00704A;
                color: white;
            }
        """)
        
        layout = QVBoxLayout()
        
        instruction_label = QLabel('테이블 번호를 선택해주세요')
        instruction_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(instruction_label)
        
        grid = QGridLayout()
        grid.setSpacing(10)
        
        if order_type == '배달':
            # Show tables 1-6 for delivery
            for i in range(6):
                row = i // 3
                col = i % 3
                table_btn = QPushButton(f"{i + 1}")
                table_btn.clicked.connect(lambda checked, num=i+1: self.select_table(num))
                grid.addWidget(table_btn, row, col)
        else:
            # Show tables 7-9 for pickup
            for i in range(3):
                floor = i + 1
                table_btn = QPushButton(f"{i + 7}\n({floor}층 픽업대)")
                table_btn.clicked.connect(lambda checked, num=i+7: self.select_table(num))
                grid.addWidget(table_btn, 0, i)
                
        layout.addLayout(grid)
        self.setLayout(layout)
        
    def select_table(self, table_num):
        self.selected_table = table_num
        self.accept()

class OptionsDialog(QDialog):
    def __init__(self, item_name, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f'Customize "{item_name}"')
        # Center the dialog relative to parent
        if parent:
            geometry = self.frameGeometry()
            center = parent.frameGeometry().center()
            geometry.moveCenter(center)
            self.move(geometry.topLeft())
            
        self.setStyleSheet("""
            QDialog {
                background-color: #ffffff;
            }
            QLabel {
                font-size: 14px;
                font-weight: bold;
                color: #00704A;
                padding: 10px 0;
            }
            QRadioButton {
                font-size: 13px;
                padding: 5px;
                border: 2px solid #00704A;
                border-radius: 5px;
                margin: 5px;
                min-width: 150px;
                min-height: 30px;
            }
            QRadioButton:checked {
                background-color: #00704A;
                color: white;
            }
            QRadioButton::indicator {
                width: 0px;
                height: 0px;
            }
            QPushButton {
                background-color: #00704A;
                color: white;
                border: none;
                padding: 10px;
                font-size: 14px;
                border-radius: 20px;
                min-width: 200px;
            }
            QPushButton:hover {
                background-color: #004F35;
            }
        """)
        layout = QVBoxLayout()
        layout.setSpacing(15)

        # Common options for drinks
        if '라떼' in item_name or '아메리카노' in item_name or '카라멜 마끼아또' in item_name or '티' in item_name:
            temp_label = QLabel('핫 or 아이스 중 선택')
            temp_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(temp_label)
            self.temp_group = QButtonGroup()
            temp_options = ['핫', '아이스']
            temp_layout = QHBoxLayout()
            temp_layout.setAlignment(Qt.AlignCenter)
            for i, option in enumerate(temp_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.temp_group.addButton(radio)
                temp_layout.addWidget(radio)
            layout.addLayout(temp_layout)

            size_label = QLabel('사이즈 선택')
            size_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(size_label)
            self.size_group = QButtonGroup()
            size_options = ['Tall (추가 금액 없음)', 'Grande (+500원)', 'Venti (+1000원)']
            size_layout = QHBoxLayout()
            size_layout.setAlignment(Qt.AlignCenter)
            for i, option in enumerate(size_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.size_group.addButton(radio)
                size_layout.addWidget(radio)
            layout.addLayout(size_layout)
        if '프라푸치노' in item_name or '쉐이크' in item_name or '스무디' in item_name:
            temp_label = QLabel('핫 or 아이스 중 선택')
            temp_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(temp_label)
            self.temp_group = QButtonGroup()
            temp_options = ['아이스 만 가능']
            temp_layout = QHBoxLayout()
            temp_layout.setAlignment(Qt.AlignCenter)
            for i, option in enumerate(temp_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.temp_group.addButton(radio)
                temp_layout.addWidget(radio)
            layout.addLayout(temp_layout)

            size_label = QLabel('사이즈 선택')
            size_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(size_label)
            self.size_group = QButtonGroup()
            size_options = ['Tall (추가 금액 없음)', 'Grande (+500원)', 'Venti (+1000원)']
            size_layout = QHBoxLayout()
            size_layout.setAlignment(Qt.AlignCenter)
            for i, option in enumerate(size_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.size_group.addButton(radio)
                size_layout.addWidget(radio)
            layout.addLayout(size_layout)
        # Espresso options for coffee drinks
        if '라떼' in item_name or '아메리카노' in item_name or '모카' in item_name or '마끼아또' in item_name:
            shot_label = QLabel('에스프레소 샷 추가')
            shot_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(shot_label)
            self.shot_group = QButtonGroup()
            shot_options = ['샷 추가 없음', '샷 추가 (+500원)']
            shot_layout = QHBoxLayout()
            shot_layout.setAlignment(Qt.AlignCenter)
            for i, option in enumerate(shot_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.shot_group.addButton(radio)
                shot_layout.addWidget(radio)
            layout.addLayout(shot_layout)

        # Milk options for latte drinks
        if '라떼' in item_name or '모카' in item_name or '마끼아또' in item_name:
            milk_label = QLabel('우유 선택')
            milk_label.setAlignment(Qt.AlignCenter)
            layout.addWidget(milk_label)
            self.milk_group = QButtonGroup()
            milk_options = ['일반 우유', '두유 (+500원)', '아몬드 우유 (+500원)']
            milk_layout = QHBoxLayout()
            milk_layout.setAlignment(Qt.AlignCenter)
            for i, option in enumerate(milk_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.milk_group.addButton(radio)
                milk_layout.addWidget(radio)
            layout.addLayout(milk_layout)

        # Options for desserts and snacks
        if '케이크' in item_name or '떡볶이' in item_name or '붕어빵' in item_name or '군고구마' in item_name:
            serving_label = QLabel('포크 지급')
            serving_label.setAlignment(Qt.AlignCenter)  # Center align the text

            layout.addWidget(serving_label)
            self.serving_group = QButtonGroup()
            serving_options = ['포크O', '포크X']
            serving_layout = QHBoxLayout()
            serving_layout.setAlignment(Qt.AlignCenter)  # Center align the options
            for i, option in enumerate(serving_options):
                radio = QRadioButton(option)
                if i == 0:
                    radio.setChecked(True)
                self.serving_group.addButton(radio)
                serving_layout.addWidget(radio)
            layout.addLayout(serving_layout)
        # Show season menu notice first if applicable
        # Show season menu notice at the top if applicable
        # Show season menu notice at the very top if applicable
        if '시즌메뉴' in item_name:
            # Insert season notice at the beginning of the layout
            season_label = QLabel('시즌메뉴 안내')
            season_label.setAlignment(Qt.AlignCenter)  # Center align the text
            season_label.setStyleSheet('QLabel { qproperty-alignment: AlignCenter; }')
            layout.insertWidget(0, season_label)
            
            season_info = QLabel('11월 ~ 2월 한정 판매 상품입니다.\n재고 소진시 주문이 어려울 수 있습니다.')
            season_info.setAlignment(Qt.AlignCenter)  # Center align the text
            season_info.setStyleSheet('QLabel { color: red; qproperty-alignment: AlignCenter; }')
            layout.insertWidget(1, season_info)
            
            # Add spacing after season notice
            layout.insertSpacing(2, 20)
        # Add buttons layout
        buttons_layout = QHBoxLayout()
        
        # Add cancel button
        cancel_btn = QPushButton('취소')
        cancel_btn.clicked.connect(self.reject)
        cancel_btn.setStyleSheet("""
            QPushButton {
                background-color: #DC3545;
            }
            QPushButton:hover {
                background-color: #BB2D3B;
            }
        """)
        buttons_layout.addWidget(cancel_btn)
        
        # Add confirm button
        confirm_btn = QPushButton('완료')
        confirm_btn.clicked.connect(self.accept)
        buttons_layout.addWidget(confirm_btn)
        
        layout.addLayout(buttons_layout)

        self.setLayout(layout)

    def get_selected_option(self, button_group):
        """Get the text of the selected radio button in a button group"""
        selected_button = button_group.checkedButton()
        return selected_button.text() if selected_button else None

class TableOrder(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'table_order')
        QMainWindow.__init__(self)
        
        self.subscription_order_status = self.create_subscription(
            String,  # 메시지 타입
            'order_status',  # 구독할 토픽 이름
            self.status_callback,  # 콜백 함수
            10  # 큐 사이즈
        )
        
        self.subscription_robot_status = self.create_subscription(
            String,  # 메시지 타입
            '/robot_status',  # 구독할 토픽 이름
            self.status_callback,  # 콜백 함수
            10  # 큐 사이즈
        )

        self.subscription_goback_status = self.create_subscription(
            Int32,
            'table_num',
            self.goback_callback,
            10
        )

        self.subscription_robot_status
        self.subscription_order_status
        self.subscription_goback_status       
 
        # Initialize menu categories and subcategories with prices
        self.menu_categories = {
            '커피': [
                ('아메리카노', 1500),
                ('꿀아메리카노', 2000),
                ('헤이즐넛 아메리카노', 2000),
                ('카페라떼', 2000),
                ('바닐라라떼', 2000),
                ('헤이즐넛라떼', 2500),
                ('연유라떼', 2500),
                ('카페모카', 2500),
                ('카라멜 마끼아또', 3000),
                ('아이스크림라떼', 3500),
                ('티라미수라떼', 4000)
            ],
            '논커피': [
                ('군밤고구마라떼(시즌메뉴)', 3500),
                ('초코라떼', 3500),
                ('딸기라떼', 3500),
                ('그린티라떼', 3500),
                ('밀크티', 4000),
                ('민트초코라떼', 4000)
            ],
            '프라페': [
                ('초코 프라푸치노', 4000),
                ('그린티 프라푸치노', 4000),
                ('딸기 프라푸치노', 4000),
                ('민트초코 프라푸치노', 4000),
                ('자바칩 프라푸치노', 4000)
            ],
            '티': [
                ('페퍼민트티', 3000),
                ('카모마일티', 3000),
                ('히비스커스티', 3000),
                ('유자티', 3000),
                ('레몬티', 3000),
                ('진저레몬티', 3500)
            ],
            '쉐이크/스무디': [
                ('밀크 쉐이크', 3000),
                ('초코 쉐이크', 3500),
                ('바나나 쉐이크', 3500),
                ('딸기 쉐이크', 3500),
                ('딸기 요거트 스무디', 4000),
                ('멜론 요거트 스무디', 4000),
                ('망고 요거트 스무디', 4000)
            ],
            '디저트': [
                ('치즈케이크', 6000),
                ('초코케이크', 6000),
                ('우유케이크', 6000),
                ('블루베리케이크', 6000)
            ],
            '스낵': [
                ('떡볶이', 4000),
                ('군고구마(시즌메뉴)', 3500),
                ('붕어빵(2개)(시즌메뉴)', 2000)
            ]
        }

        # Initialize total price and order list
        self.total_price = 0
        self.orders = []
        self.order_confirmed = False
        self.selected_table = None

        # Set up publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.order_publisher = self.create_publisher(String, 'table_order', qos_profile)

        # Set up UI styling
        self.setStyleSheet("""
            QMainWindow {
                background-color: #ffffff;
            }
            QPushButton {
                border: 2px solid #00704A;
                border-radius: 20px;
                padding: 10px;
                background-color: white;
                color: #00704A;
            }
            QPushButton:hover {
                background-color: #00704A;
                color: white;
            }
            QListWidget {
                border: 2px solid #E5E5E5;
                border-radius: 10px;
                padding: 10px;
            }
            QLabel {
                color: #00704A;
                font-weight: bold;
            }
        """)

        # Set up window properties
        self.setWindowTitle('ROKEY F4 Cafe')
        self.setGeometry(100, 100, 1400, 900)

        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # Create category buttons layout
        category_layout = QHBoxLayout()
        category_layout.setSpacing(15)
        for category in self.menu_categories.keys():
            category_btn = QPushButton(category)
            category_btn.setMinimumHeight(60)
            category_btn.setFont(QFont('Arial', 12, QFont.Bold))
            category_btn.clicked.connect(lambda checked, cat=category: self.show_subcategories(cat))
            category_layout.addWidget(category_btn)
        main_layout.addLayout(category_layout)

        # Create grid layout for menu items
        self.subcategory_grid = QGridLayout()
        self.subcategory_grid.setSpacing(15)
        main_layout.addLayout(self.subcategory_grid)

        # Create order section
        order_section = QVBoxLayout()
        order_section.setSpacing(10)

        order_header = QLabel('Your Order')
        order_header.setFont(QFont('Arial', 16, QFont.Bold))
        order_section.addWidget(order_header)

        self.order_list = QListWidget()
        self.order_list.setFont(QFont('Arial', 14))
        self.order_list.setMinimumHeight(150)
        order_section.addWidget(self.order_list)

        # Add remove item button
        remove_item_btn = QPushButton('선택한 메뉴 삭제')
        remove_item_btn.clicked.connect(self.remove_selected_item)
        remove_item_btn.setFont(QFont('Arial', 14, QFont.Bold))  # Changed font size to match order status label
        remove_item_btn.setStyleSheet("""
            QPushButton {
                background-color: #00704A;
                color: white;
                font-weight: bold;
                min-width: 200px;
            }
            QPushButton:hover {
                background-color: #004F35;
            }
        """)
        remove_item_btn.setMinimumHeight(50)
        order_section.addWidget(remove_item_btn)

        # Add order status label
        self.order_status_label = QLabel('주문 상태: 주문 대기중')
        self.order_status_label.setFont(QFont('Arial', 14, QFont.Bold))
        self.order_status_label.setAlignment(Qt.AlignCenter)
        self.order_status_label.setStyleSheet("""
            QLabel {
                background-color: #E5E5E5;
                padding: 10px;
                border-radius: 10px;
                margin: 10px 0;
            }
        """)
        order_section.addWidget(self.order_status_label)

        self.total_price_label = QLabel('합계: 0원')
        self.total_price_label.setFont(QFont('Arial', 14, QFont.Bold))
        self.total_price_label.setAlignment(Qt.AlignRight)
        order_section.addWidget(self.total_price_label)

        # Add order control buttons
        order_buttons_layout = QHBoxLayout()
        order_buttons_layout.setSpacing(15)
        
        finish_order_btn = QPushButton('주문하기')
        finish_order_btn.clicked.connect(self.finish_order)
        finish_order_btn.setStyleSheet("""
            QPushButton {
                background-color: #00704A;
                color: white;
                font-weight: bold;
                min-width: 200px;
            }
            QPushButton:hover {
                background-color: #004F35;
            }
        """)
        finish_order_btn.setMinimumHeight(50)
        
        cancel_order_btn = QPushButton('주문 초기화')
        cancel_order_btn.clicked.connect(self.cancel_order)
        cancel_order_btn.setStyleSheet("""
            QPushButton {
                background-color: #DC3545;
                color: white;
                font-weight: bold;
                min-width: 200px;
            }
            QPushButton:hover {
                background-color: #BB2D3B;
            }
        """)
        cancel_order_btn.setMinimumHeight(50)
        
        order_buttons_layout.addStretch()
        order_buttons_layout.addWidget(cancel_order_btn)
        order_buttons_layout.addWidget(finish_order_btn)
        
        order_section.addLayout(order_buttons_layout)
        main_layout.addLayout(order_section)
    
    def status_callback(self, msg):
        # 수신한 메시지 출력
        if(msg.data == '조리 중'):
            self.order_status_label.setText('주문 상태: 조리 중')
        elif('주문취소' in msg.data):
            self.order_status_label.setText('주문 상태 : 주문 대기중')

            # 주문 취소 팝업창 표시
            QMessageBox.warning(self, "주문 취소", msg.data)

        elif(msg.data == '조리완료'):
            self.order_status_label.setText('주문 상태: 이동 중')
        elif(msg.data == '이동 완료'):
            self.order_status_label.setText('로봇이 음식을 가져왔어요! 배달 확인 버튼을 눌러주세요!')




    def goback_callback(self, msg):
        if (msg.data == 0) and (self.order_status_label.text() == '로봇이 음식을 가져왔어요! 배달 확인 버튼을 눌러주세요!'):
            self.order_status_label.setText('주문 상태 : 주문 대기중')

    def show_subcategories(self, category):
        """Show subcategories in grid when category button is clicked"""
        if self.order_confirmed:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle('경고')
            msg.setText('주문이 이미 확정되었습니다. 새로운 주문을 하시려면 주문 초기화를 해주세요.')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()
            return
            
        # Clear previous subcategories
        for i in reversed(range(self.subcategory_grid.count())): 
            self.subcategory_grid.itemAt(i).widget().setParent(None)
            
        # Add new subcategories in grid layout
        subcategories = self.menu_categories[category]
        rows = (len(subcategories) + 2) // 3
        for i, (item_name, price) in enumerate(subcategories):
            row = i // 3
            col = i % 3
            btn = QPushButton(f"{item_name}\n{price:,}원")
            btn.setMinimumSize(250, 100)
            btn.setFont(QFont('Arial', 12))
            btn.clicked.connect(lambda checked, name=item_name, p=price: self.show_options(name, p))
            self.subcategory_grid.addWidget(btn, row, col)

    def show_options(self, item_name, base_price):
        """Show options dialog when menu item is clicked"""
        if self.order_confirmed:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle('경고')
            msg.setText('주문이 이미 확정되었습니다. 새로운 주문을 하시려면 주문 초기화를 해주세요.')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()
            return
            
        dialog = OptionsDialog(item_name, self)
        if dialog.exec_():
            # Calculate additional costs from options
            additional_cost = 0
            options_text = []

            if hasattr(dialog, 'temp_group'):
                temp = dialog.get_selected_option(dialog.temp_group)
                if temp == '핫':  # Only add temperature if it's Hot
                    options_text.append(temp)
                else:
                    options_text.append('아이스')

            if hasattr(dialog, 'size_group'):
                size = dialog.get_selected_option(dialog.size_group)
                size_type = size.split(' ')[0]
                if size_type != 'Tall':  # Only add size if not Tall
                    options_text.append(size_type)
                if 'Grande' in size:
                    additional_cost += 500
                elif 'Venti' in size:
                    additional_cost += 1000

            if hasattr(dialog, 'shot_group'):
                shot = dialog.get_selected_option(dialog.shot_group)
                if '샷 추가' in shot:
                    additional_cost += 500
                    options_text.append('샷 추가')

            if hasattr(dialog, 'milk_group'):
                milk = dialog.get_selected_option(dialog.milk_group)
                milk_type = milk.split(' ')[0]
                if milk_type != '일반 우유':  # Only add milk type if not Whole
                    options_text.append(milk_type)
                if '두유' in milk or '아몬드 우유' in milk:
                    additional_cost += 500

            if hasattr(dialog, 'serving_group'):
                serving = dialog.get_selected_option(dialog.serving_group)
                if serving != 'To Go':  # Only add serving option if not To Go
                    options_text.append(serving)

            # Update total price
            self.total_price += base_price + additional_cost
            self.total_price_label.setText(f'합계: {self.total_price:,}원')

            # Create order message and add to list
            options_str = ' / '.join(options_text) if options_text else ''
            order_text = f"{item_name}"
            if options_str:
                order_text += f" ({options_str})"
            order_text += f" - {base_price + additional_cost:,}원"
            
            self.order_list.addItem(order_text)
            self.orders.append(order_text)

            # Update order status
            self.order_status_label.setText('주문 상태: 메뉴 선택중')
            self.order_status_label.setStyleSheet("""
                QLabel {
                    background-color: #FFF3CD;
                    color: #856404;
                    padding: 10px;
                    border-radius: 10px;
                    margin: 10px 0;
                }
            """)

            # Publish order
            msg = String()
            msg.data = f"Order: {order_text}"
            self.get_logger().info(f'Order placed: {msg.data}')

    def remove_selected_item(self):
        """Remove selected item from order list"""
        if self.order_confirmed:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle('경고')
            msg.setText('주문이 이미 확정되었습니다.')
            msg.setInformativeText('새로운 주문을 추가해주세요.')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()
            return
            
        current_item = self.order_list.currentItem()
        if current_item:
            # Extract price from the item text and subtract from total
            price_str = current_item.text().split(' - ')[1].replace('원', '').replace(',', '')
            self.total_price -= int(price_str)
            self.total_price_label.setText(f'합계: {self.total_price:,}원')
            
            # Remove item from orders list and list widget
            row = self.order_list.row(current_item)
            self.orders.pop(row)
            self.order_list.takeItem(row)
            
            # Update order status if no items remain
            if not self.orders:
                self.order_status_label.setText('주문 상태: 주문 대기중')
                self.order_status_label.setStyleSheet("""
                    QLabel {
                        background-color: #E5E5E5;
                        padding: 10px;
                        border-radius: 10px;
                        margin: 10px 0;
                    }
                """)
            
            # Publish update
            msg = String()
            msg.data = f"Removed item: {current_item.text()}"
            self.order_publisher.publish(msg)
            self.get_logger().info(msg.data)
        else:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle('경고')
            msg.setText('삭제할 메뉴를 선택해주세요.')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()

    def finish_order(self):
        """Finish and confirm the current order"""
        if not self.orders:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle('경고')
            msg.setText('주문을 추가해주세요!')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()
            return
            
        if not self.order_confirmed:
            # Show table selection dialog first
            table_dialog = TableSelectionDialog(self)
            if table_dialog.exec_():
                self.selected_table = table_dialog.selected_table
                
                msg = QMessageBox(self)
                msg.setIcon(QMessageBox.Question)
                msg.setWindowTitle('주문확인')
                msg.setText(f'테이블 번호: {self.selected_table}\n합계: {self.total_price:,}원\n\n주문을 확정하시겠습니까?\n주문 확정 후에는 수정이 불가능합니다.\n취소는 매장으로 문의해주세요.')
                msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
                msg.setDefaultButton(QMessageBox.No)
                msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
                reply = msg.exec_()
                
                if reply == QMessageBox.Yes:
                    self.order_confirmed = True
                    # Update order status
                    self.order_status_label.setText('주문 상태: 주문 완료')
                    self.order_status_label.setStyleSheet("""
                        QLabel {
                            background-color: #D4EDDA;
                            color: #155724;
                            padding: 10px;
                            border-radius: 10px;
                            margin: 10px 0;
                        }
                    """)
                    
                    # Publish final order with table number
                    msg = String()
                    msg.data = f"테이블 번호: {self.selected_table}\n주문확인 - 합계: {self.total_price:,}원\n" + "\n".join(self.orders)
                    self.order_publisher.publish(msg)
                    self.get_logger().info('주문완료')
                    
                    success_msg = QMessageBox(self)
                    success_msg.setIcon(QMessageBox.Information)
                    success_msg.setWindowTitle('성공')
                    success_msg.setText('주문이 확정되었습니다!')
                    success_msg.move(self.frameGeometry().center() - success_msg.frameGeometry().center())
                    success_msg.exec_()
                    
                    # Clear all orders after confirmation
                    self.orders.clear()
                    self.order_list.clear()
                    self.total_price = 0
                    self.total_price_label.setText('합계: 0원')
                    self.order_confirmed = False
                    self.selected_table = None
        else:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Warning)
            msg.setWindowTitle('경고')
            msg.setText('주문이 이미 확정되었습니다. 새로운 주문을 하시려면 주문 초기화를 해주세요.')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()

    def cancel_order(self):
        """Cancel the current order"""
        if self.orders:  # Only show popup if there are orders to cancel
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Question)
            msg.setWindowTitle('주문취소')
            msg.setText('정말로 주문을 취소하시겠습니까?')
            msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
            msg.setDefaultButton(QMessageBox.No)
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            reply = msg.exec_()
            
            if reply == QMessageBox.Yes:
                self.orders.clear()
                self.order_list.clear()
                self.total_price = 0
                self.total_price_label.setText('합계: 0원')
                self.order_confirmed = False
                self.selected_table = None
                
                success_msg = QMessageBox(self)
                success_msg.setIcon(QMessageBox.Information)
                success_msg.setWindowTitle('취소완료')
                success_msg.setText('주문이 취소되었습니다.')
                success_msg.move(self.frameGeometry().center() - success_msg.frameGeometry().center())
                success_msg.exec_()
                
                self.get_logger().info('주문이 취소되었습니다.')
        else:
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Information)
            msg.setWindowTitle('알림')
            msg.setText('취소할 주문이 없습니다.')
            msg.move(self.frameGeometry().center() - msg.frameGeometry().center())
            msg.exec_()

def main(args=None):
    rclpy.init(args=args)
    from PyQt5.QtWidgets import QApplication
    import sys
    
    app = QApplication(sys.argv)
    table_order = TableOrder()
    table_order.show()
    
    while rclpy.ok():
        rclpy.spin_once(table_order, timeout_sec=0.1)
        app.processEvents()
    
    table_order.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
