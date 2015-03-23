# This message defines the measurement of a single marker.

# The unique ID of the measured marker.
string marker_id

# The unique ID of the camera measuring this marker.
string camera_id

# The position and orientation of the marker in the camera frame.
geometry_msgs/Pose marker_pose

# The image points of the marker in the camera image.
geometry_msgs/Point[] image_points
