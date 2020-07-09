the tutorial for using this package is shown as follows:
1. roslaunch visionbased_polishing.launch:
launch the camera driver, the force sensor driver, the ur driver 

2. rosrun cross_line_detecting_four_point_v1.py:
run cross line detecting program

3. rosrun structure_light_point_xdydzd_pub_v1.py

4. rosrun structure_light_point_control_structure_four_point_v3.py
run vision-based controller

the modifications on 20200628 are shown as follows:
1. visionbased_polishingcontroller_v1.py is the original version
2. visionbased_polishingcontroller_v2.py modifies almost all functions except the function of get_joint_speed
3. visionbased_polishingcontroller_v3.py is the final version on 19:40 pm 20200628

the modifications on 20200629 are shown as follows:
1. desiredpath_visualization_v1.py is the original version 
2. desiredpath_visualization_v2.py is modified 
3. desiredpath_visualization_v3.py is similar to v2.py, which deletes some notes and add some functions
4. polygon_detection_v3.py is the newest version for computing and visualizing polygon 
5. image_features_generation.py generates three image features: xnynan
6. image_features_visualization.py visualizes the polygons

the final version for these py documents are:
1. py 1: desired path publish
2. py 2: image feature publish
3. py 3: desired path and image features visualization
4. py 4: vision based polishing controller


the debugging log of 20200707 is shown as follows:
roslaunch visionbased_polishing visionbased_polishing_configuration.launch 
rosrun robotiq_ft_sensor rq_test_sensor

1. rosrun visionbased_polishing fixedpoint_motion.py
and then ctrl+c
2. rosrun visionbased_polishing generation_desiredpath_feature.py
which aims to generate desired paths 
3. rosrun visionbased_polishing generation_structuredlight_feature_v2.py
which aims to generate structured light features
4. rosrun visionbased_polishing visualization_all_features_v2.py
which aims to visualize all image features
5. rosrun visionbased_polishing visionbased_polishingcontroller_v6.py


The debug process doesn't work so well


fd=[A,B,C], B and C is not the right direction for polishing operation 













