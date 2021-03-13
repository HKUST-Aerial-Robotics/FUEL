# camera, map, range, path, traj, cmd
# /vins_estimator/camera_pose
rosbag record  /sdf_map/occupancy /sdf_map/occupancy_inflate /planning_vis/frontier /planning_vis/trajectory /planning_vis/yaw /planning/position_cmd_vis /position_cmd /vins_estimator/camera_pose /planning_vis/visib_constraint 