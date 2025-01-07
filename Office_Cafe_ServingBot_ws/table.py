import sqlite3
from prettytable import PrettyTable, ALL

# 데이터베이스 
conn = sqlite3.connect('orders.db')
cursor = conn.cursor()


cursor.execute("SELECT * FROM orders")
rows = cursor.fetchall()

# 컬럼 이름 가져오기
column_names = [description[0] for description in cursor.description]


table = PrettyTable()
table.field_names = column_names


for row in rows:
    table.add_row(row)

# 테이블 스타일 설정
table.align = 'l'  # 왼쪽 정렬
table.max_width = 30  # 각 열의 최대 너비
table.border = True
table.header = True
table.hrules = ALL
table.vrules = ALL


print(table)

conn.close()
