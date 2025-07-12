
TOTAL_CORES=2
KMP_SETTING="KMP_AFFINITY=granularity=fine,compact,1,0"
KMP_BLOCKTIME=1
PREFIX="numactl --physcpubind=0-$LAST_CORE --membind=0"

export OMP_NUM_THREADS=$TOTAL_CORES
export $KMP_SETTING
export KMP_BLOCKTIME=$KMP_BLOCKTIME
export PREFIX

echo -e "### using OMP_NUM_THREADS=$TOTAL_CORES"
echo -e "### using $KMP_SETTING"
echo -e "### using KMP_BLOCKTIME=$KMP_BLOCKTIME"
echo -e "### using $PREFIX\n"


#Run Ros2 usbcam node with mono YOLO_SLAM_OCTOMAPusing realsense_D435i.yaml or Logitech-C920.yaml as example:

ros2 run yolo_slam_octomap rgbd ~/Desktop/SLAM/YOLO_SLAM_OCTOMAP/Vocabulary/ORBvoc.txt ~/Desktop/SLAM/YOLO_SLAM_OCTOMAP/Examples/RGB-D/RealSense_D435i.yaml


