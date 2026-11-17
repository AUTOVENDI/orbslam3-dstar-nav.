import os
import time

def check_orbslam_status():
    # Check if ORB-SLAM3 is running
    result = os.popen("ros2 node list | grep ORB_SLAM3_ROS2").read().strip()
    return bool(result)  # Returns True if ORB-SLAM3 is running

def restart_orbslam():
    print("Restarting ORB-SLAM3...")
    os.system("ros2 run ros2_orb_slam3 monocular-slam-node \
        /home/autovendi/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin \
        /home/autovendi/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/TUM1.yaml &")

if __name__ == "__main__":
    while True:
        if not check_orbslam_status():
            restart_orbslam()
        time.sleep(5)  # Check every 5 seconds

