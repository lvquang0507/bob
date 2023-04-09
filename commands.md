## Launch robot with gazebo
`ros2 launch bob rsp_gz.launch.py`
## Launch robot with gazebo with world config
`ros2 launch bob rsp_gz.launch.py world:=path/to/file.world`

**Example**: `ros2 launch bob rsp_gz.launch.py world:=./src/bob/worlds/w2.world`
## Launch rviz2 with camera and lidar
`rviz2 -d ./src/bob/config/lidar_camera_bob.rviz`
## Launch slam_tool box online async :
`ros2 launch slam_toolbox online_async_launch.py params_file:=./src/bob/config/mapper_params_online_async.yaml use_sim_time:=true`