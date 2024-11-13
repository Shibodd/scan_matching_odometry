Output an odometry based on successive scan matching.
Note that this does not perform any loop closure of sorts - the output is intended to be used differentially.


ros2 run scan_matching_odometry scan_matching_odometry --ros-args -p tf.publish:=true -p twist.timestamp_from_scan:=average
