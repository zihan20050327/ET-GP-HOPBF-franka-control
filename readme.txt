To run ET-GP-P-HOCBF, follow the 5 steps:

1. open a terminal and run command: roscore

2. run path visualization node

(1) open a terminal and go to path: cd /home/yiming/catkin_ws/src/DEMO03_WS
(2) source devel/setup.bash
(3) rosrun plumbing_pub_sub tf_path_publisher (Ignore the warning)

3. run obstacle visualization node

(1) open or split terminal and go to path: cd /home/yiming/catkin_ws/src/DEMO03_WS
(2) source devel/setup.bash
(3) rosrun plumbing_pub_sub sphere_marker

4. run GP node

(1) open or split terminal and go to path: cd /home/yiming/catkin_ws/src/DEMO03_WS
(2) source devel/setup.bash
(3) rosrun plumbing_pub_sub gp_node

5. run franka control node

(1) open a terminal and go to path: cd /home/yiming/catkin_ws
(2) source devel/setup.bash
(3) roslaunch franka_example_controllers force_example_controller.launch robot_ip:=192.168.1.1 load_gripper:=true

6. RViz window (optional)
Add(bottom left) - By topic - /end_effector_path - path 
Add(bottom left) - By topic - /sphere_maker - Maker





To modify and compile code:

1. modify franka control node

(1) modify the code (/home/yiming/catkin_ws/src/franka_ros/franka_example_controllers)
(2) open a terminal and go to path: cd /home/yiming/catkin_ws
(3) source devel/setup.bash
(4) compile: catkin_make

2. modify GP or visualization code

(1) open a terminal and go to path: cd /home/yiming/catkin_ws/src/DEMO03_WS
(2) open VScode under this path: code .
(3) modify code
(4) compile: CTRL+SHIFT+B - build  

