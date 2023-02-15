# header specifying the frame of ref. of the position/velocity ref.
std_msgs/Header header

# mode used for control (position, velocity).
# mode decides which mrs_msgs will be used.
string control_mode

# position in meter(m)
geometry_msgs/Point position

# velocity in m/s
geometry_msgs/Vector3 velocity

# Heading is atan2() of XY-world projection of the UAV's body X-axis.
# specified in radians
float64 heading

# true if the UAV wants to search for point of interest by rotating.
# useful when camera FOV is limited.
bool rotate_to_search
