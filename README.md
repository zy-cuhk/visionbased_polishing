the tutorial for using this package is shown as follows:
1. roslaunch visionbased_polishing.launch:
launch the camera driver, the force sensor driver, the ur driver 

2. rosrun cross_line_detecting_four_point_v1.py:
run cross line detecting program

3. rosrun structure_light_point_xdydzd_pub_v1.py

4. rosrun structure_light_point_control_structure_four_point_v3.py
run vision-based controller

the modifications are shown as follows:
1. visionbased_polishingcontroller_v1.py is the original version
2. visionbased_polishingcontroller_v2.py modifies almost all functions except the function of get_joint_speed