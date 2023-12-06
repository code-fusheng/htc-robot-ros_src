# gps_tools

# pose_recorder
Used to recored positions from imu_gps_localization or ndt_localization, 

## Params
Subscribe:
* `/current_pose` for imu_gps_localization
录制的时候区分gps路径还是ndt路径

Publish:
* `/path/gps_poses`: gps history poses
* `/path/ndt_poses`: ndt history poses

* `filter`: set if filter pose  -- `on` is recommended, to avoid recording two many same poses when car is halted.
* `filter_distance`: min distance before two point, default 0.1m

*By default, data will be saved under `~/Documents/${basedir}/pose_recorder/`, named as gps_poses_{TIME}.csv|ndt_poses_{TIME}.csv*