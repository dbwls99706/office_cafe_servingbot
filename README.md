 🤖 사내 키페 서빙 로봇 시스템 시뮬레이션
=============
Gazebo와 Rviz2를 활용하여 사내 카페와 서빙 로봇을 결합한 커피 배달 서비스 구축 시뮬레이션


[프로젝트 기록](https://velog.io/@cherry0319/%EC%82%AC%EB%82%B4-%EC%B9%B4%ED%8E%98-%EC%84%9C%EB%B9%99-%EB%A1%9C%EB%B4%87-%EC%8B%9C%EB%AE%AC%EB%A0%88%EC%9D%B4%EC%85%98) 

<br>


인원 및 기간
-------------
* 4명 : 기태훈, [김현아](https://github.com/Hyuna-319), [장석환](https://github.com/JSH0101), [홍유진](https://github.com/dbwls99706)
* 2024.11.12 ~ 2024.11.18 (7일)
  
<br> 

사용 기술
-------------
* Language : Python3
* OS : Linux Ubuntu 22.04 jammy
* Skills : ROS2 Humble, Gazebo11, Rviz2, Nav2, Turtlebo3 Packages, PyQT5, SQLite3

  
<br>

특징
-------------
* 테이블 오더
  - 실제 카페와 유사한 다양한 메뉴와 커스텀 옵션 제공
  - 사무실 내 배달과 사무실 외 픽업 공간을 분리 운영

 
<br>

* 주방 디스플레이
  - 테이블 오더와 주방 디스플레이 주문 정보를 비교하여 오조리 방지
  - 주문 취소 사유를 제공하여 고객 만족도 향상
  - 조리 완료 후 서빙 로봇에게 정확한 테이블 번호로 배달 명령

    
<br>

* 서빙 로봇
  - 사용자 친화적인 로봇 이미지 디자인 
  - 친근한 메시지와 무료 쿠폰 제공 퀴즈로 특별한 고객 경험 제공
  - 팀 단위 주문 시 다른 테이블로의 배달 기능 추가로 유용성 향상

<br>

* 기타 시스템
  - 모든 주문 내역을 데이터베이스에 저장하고, 총 판매액 및 TOP3 메뉴 통계로 운영 효율성 제고
  - 시스템 상태(Info, Warning, Error)에 대한 로그를 기록하여 문제 해결 기여  



<br>

보완 사항
-------------
* 로봇 음성 서비스 추가로 더욱 사용자 친화적인 인터페이스 제공
* 데이터베이스 세분화를 통해 상세 매출 분석 및 고객 선호도 파악 가능
* 더치페이 기능 추가로 다른 테이블로 배달과 결제의 시너지 효과 창출


<br>
<br>


프로젝트 결과
-------------



명령어
-------------

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

ros2 launch turtle_table turtle_table_launch.py
ros2 launch turtle_kithcen turtle_kitchen_launch.py
ros2 launch turtle_serving turtle_serving_launch.py

```

<br>

결과 이미지
-------------

![썸네일](https://github.com/user-attachments/assets/a9a08523-2e82-4bdd-a49b-35def1179d53)

<br>

결과 동영상
-------------
[![사내 카페 서빙 로봇 ](https://img.youtube.com/vi/FOsplIEOXMg/hqdefault.jpg)](https://www.youtube.com/watch?v=FOsplIEOXMg)





