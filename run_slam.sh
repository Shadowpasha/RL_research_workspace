#!/bin/bash

#gnome-terminal --title "YOLO-ORBSLAM3" --tab --working-directory /home/mtaufiq23/ros2_yolov5_orb_slamv3_ws -- ./ros2slam.sh 

#gnome-terminal --title "Octomap-Rviz2" --tab --working-directory /home/mtaufiq23/ros2_yolov5_orb_slamv3_ws -- ./ros2octomap.sh 
gnome-terminal --title "RS-D435i-Node" --tab --working-directory /root/ros2_ws -- ./ros2cam.sh 
./ros2slam.sh



#gnome-terminal --tab -- bash -c 'echo "Terminal 1"; exec bash'
#gnome-terminal --tab -- bash -c 'echo "Terminal 2"; exec bash'
#gnome-terminal --tab -- bash -c 'echo "Terminal 3"; exec bash'
