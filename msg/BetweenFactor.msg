# A loop closure message used for graph slam.
std_msgs/Header header

# The ids of the two timestamps used for loop closure.
int32 from_id
int32 to_id

# The relative pose measured between the two timestamps for loop closure.
geometry_msgs/Pose2D pose

# The covariance matrix as a row first 3x3 matrix
# [xx, xy, xt,
#  yx, yy, yt,
#  tx, ty, tt]
#
float32[9] covariance