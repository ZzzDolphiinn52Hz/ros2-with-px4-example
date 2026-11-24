# Terminal 1
ssh raspi
cd ~/ros2_ws
./uavcup aruco-camera


# Terminal 2
ssh raspi
cd ~/ros2_ws
./uavcup aruco-test


# Terminal 3
ssh raspi
cd ~/ros2_ws

docker compose run --rm ros bash -lc \
'source /opt/ros/humble/setup.bash && \
 source install/setup.bash && \
 ros2 topic echo /uav/aruco/status --once'
 
 TEST="distance_0p30"

docker compose run --rm ros bash -lc \
"source /opt/ros/humble/setup.bash && \
 source install/setup.bash && \
 timeout --signal=INT --kill-after=5s 30s \
 ros2 bag record \
   -o /ros2_ws/artifacts/aruco_static/$SESSION/$TEST \
   /uav/aruco/target_pose \
   /uav/aruco/status \
   /uav/aruco/ids \
   /uav/aruco/rvecs \
   /uav/aruco/tvecs"
