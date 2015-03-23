# This message defines the measurement of a single camera.

# The unique ID of the camera.
string camera_id

# The list of marker measurements made by the camera.
MarkerMeasurement[] marker_points

# The intrinsic calibration parameters of the camera.
sensor_msgs/CameraInfo camera_info
