# roslaunch hsr_bringup co605_gripper.launch &
# sleep 5s
# roslaunch gripper_bridge gripper_bridge0.launch &
# sleep 2s
# roslaunch realsense2_camera rs_camera.launch &
# roslaunch hsr_bringup publish_d435i_calibration_single.launch &
# roslaunch hsr_bringup publish_d435i_calibration_point_cloud_pub.launch &
# rosrun dm_bridge dm_bridge &
# rosrun perception_bridge perception_bridge &
# roslaunch vision_bridge vision_bridge_realsense2.launch &

rosservice call /setGripper "gripperName: 'SerialGripper'"
rosservice call /connectGripper "{}"

# rosrun planner_bridge planner &
# roslaunch motion_bridge singel_bring.launch &
# roslaunch pickplace_bridge serial_pickplace_bridge.launch &

# cd ~/clouds/
# rosrun read_pcd read_pcd_node point_cloud_pub_frame cloud10.pcd &


