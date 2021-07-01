*************

hedge_rcv_bin.cpp

File that allows to publish Marvelmind's position into /mavros/vision_pose/pose

*************
offb_node.cpp

File that allows to control PX4 by attitude by publishing into /mavros/setpoint_attitude/attitude and mavros/setpoint_attitude/thrust.
It subscribes to /sim_attitude and /sim_thrust. These topics should be published from an external controller, such as the one in this repo done in Simulink

*************
offb_node_pos.cpp 

File that allows you to control PX4 by position. It will subscribe to the position in /sim_position and publish in mavros/setpoint_position/local