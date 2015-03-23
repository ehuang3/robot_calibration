# This message defines the estimated parameters of a single joint.

# The name of the joint the estimated parameters belong to.
string joint_name

# The estimated encoder offset (in radians) to the joint's zero position.
float64 joint_offset

# The estimated position of the joint axis.
geometry_msgs/Point joint_axis
