KẾT NỐI CẢ LIDAR VÀ ARDUINO
1.	Chạy slam
-roscore
-roslaunch my_robot slam.launch
mở thêm 1 teminal chạy
-roslaunch rplidar_ros rplidar.launch 
( cài git clone của rplidar_ros cài về là được git clone https://github.com/Slamtec/rplidar_ros.git )
2.	Chạy move_base
-roscore
-roslaunch my_robot move_base.launch
mở thêm 1 teminal chạy
-roslaunch rplidar_ros rplidar.launch 
( cài git clone của rplidar_ros cài về là được git clone https://github.com/Slamtec/rplidar_ros.git )
3.	Chạy PID(riêng pid kết nối arduino là được)
-roscore
-roslaunch my_robot display.launch 
Hiện rvix thêm tf và robot model
Lệnh di chuyển mở terminal 
rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1}, angular: {z: 0.0}}'



