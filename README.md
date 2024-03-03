*라이더를 통해 후미 로봇이 선두 로봇을 추적하도록 하는 시뮬레이션. 터틀봇 두 대 사용.



로보티즈 홈페이지 : https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview

1.로보티즈 홈페이지에서 quick start guide, simulation에서 gazebo simulation, SLAM simulation, navigation simulation에서 시키는거 다 따라할 것.(humble)

2.위 파일들 zip으로 다운받아서 설명란에 적혀있는대로 파일 덮어쓰기할 것.

3.  $ sudo apt-get install python3-pandas

4.  $ pip3 install -U scikit-learn

실행법

1.  $ ros2 launch turtlebot3_gazebo multi_robot.launch.py (gazebo 실행)

2.  $ ros2 launch turtlebot3_navigation2 nav2_custom.launch.py (nav2 실행)

3.  $ ros2 launch lidar_cluster lidar_cluster.launch.py (선두 터틀봇을 감지하고, nav2에 goalpoint로 업데이트시켜주는 노드)

ldiar_cluster : 후미 로봇의 라이다 데이터를 DBSCAN으로 분류 후 가장 작은 클러스터(그룹)을 선두 로봇으로 인식하여 pointcloud를 publish하도록 함.

goal_update : lidar_cluster에서 얻은 pointcloud 데이터 중 가장 가까운 점을 tf2_ros를 이용하여 map 기준의 절대좌표로 변환 후 0.5초마다 nav2의 goal로 업데이트. (/goal_update)
