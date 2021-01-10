feedback_raw.cc : ROS中发布位置的节点

在ros中使用，发布/fobot_raw_joint_position_publisher和/fobot_joint_position_publisher两个节点。
放到fobot_control/src中，修改CMakeLists.txt添加可执行文件。

======

SCServo_Linux : 位置调试，回到2048位置，测试写位置

进入SCServo_Linux，
mkdir build && cd build && cmake .. && make
有 Super_FeedBack，Super_GoHome 和 Super_WritePos 三个程序