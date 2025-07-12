

#Run Ros2 usbcam node with mono yolov5_orbslam3 using realsense_D435i.yaml or Logitech-C920.yaml as example:
#ros2 run rqt_image_view rqt_image_view
#ros2 launch image_publisher_mono.launch.py
#ros2 run usb_cam usb_cam_node_exe
#ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/mtaufiq23/.ros/camera_info/test_camera.yaml
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:='640x480x15' depth_module.profile:='640x480x15'
#ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true rgb_camera.profile:='640x480x15' depth_module.profile:='640x480x15' 
#ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true rgb_camera.profile:='640x480x15' depth_module.profile:='640x480x15' gyro_fps:=200 accel_fps:=200 enable-gyro:=true enable_accel:=true

#ros2 launch realsense2_camera rs_launch.py config_file:="'$COLCON_PREFIX_PATH/realsense2_camera/share/realsense2_camera/config/d435i.yaml'"
