로보티즈 홈페이지 : https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview

1.로보티즈 홈페이지에서 quick start guide, simulation에서 gazebo simulation, SLAM simulation, navigation simulation에서 시키는거 다 따라할 것.(humble)

2.위 파일들 zip으로 다운받아서 설명란에 적혀있는대로 파일 덮어쓰기할 것.

3.  $ sudo apt-get install python3-pandas

4.  $ pip3 install -U scikit-learn

실행법

1.  $ ros2 launch turtlebot3_gazebo multi_robot.launch.py (gazebo 실행)

2.  $ ros2 launch turtlebot3_navigation2 nav2_custom.launch.py (nav2 실행)

3.  $ ros2 launch lidar_cluster lidar_cluster.launch.py (선두 터틀봇을 감지하고, nav2에 goalpoint로 업데이트시켜주는 노드)
