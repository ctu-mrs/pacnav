#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/pacnav2020swarm/sim/\$(arg UAV_NAME)"

# exclude=(
# # exclude every compressedDepth
# '(.*)compressedDepth(.*)'
# '(.*)theora(.*)'
# '(.*)h264(.*)'
# # bluefox_left
# '(.*)image_raw'
# '(.*)image_raw(.*)'
# '(.*)compressed'
# '(.*)compressed(.*)'
# # bluefox_right
# '/$(arg UAV_NAME)/uvdar_bluefox/right/image_raw'
# '/$(arg UAV_NAME)/uvdar_bluefox/right/image_raw/compressed'
# '/$(arg UAV_NAME)/uvdar_bluefox/right/image_raw/compressed/(.*)'
# # bluefox_optic_flow
# '(.*)bluefox_optflow(.*)'
# # Mapping
# '/$(arg UAV_NAME)/hector_mapping/(.*)'
# '/$(arg UAV_NAME)/path_finder/path_map'
# '/$(arg UAV_NAME)/path_finder/distance_map'
# # Realsense t265
# '/$(arg UAV_NAME)/rs_t265/fisheye(.*)'
# # Realsesne d435
# '(.*)rs_d435(.*)depth_to_infra(.*)'
# '(.*)rs_d435(.*)depth_to_color/image_raw'
# '(.*)rs_d435(.*)depth_to_color(.*)compressed'
# '(.*)rs_d435(.*)depth_to_color(.*)compressed/(.*)'
# '(.*)rs_d435(.*)color/image_raw'
# '(.*)rs_d435(.*)/depth/(.*)'
# '(.*)rs_d435(.*)/infra(.*)'
# '(.*)rs_d435(.*)/color/image_rect_color'
# # Every theora message
# '(.*)theora(.*)'
# '(.*)gazebo(.*)'
# )

include=(
  '/tf'
  '/tf_static'
  '/$(arg UAV_NAME)/odometry/odom_main'
  '/$(arg UAV_NAME)/control_manager/speed_tracker/command'
  '/$(arg UAV_NAME)/control_manager/speed_tracker/command'
  '/$(arg UAV_NAME)/mrs_uav_status/display_string'
  )

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

# echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o $path -a" >> "$filename"
echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o $path " >> "$filename"

# if there is anything to exclude
# if [ "${#exclude[*]}" -gt 0 ]; then

#   echo -n " -x " >> "$filename"

#   # list all the string and separate the with |
#   for ((i=0; i < ${#exclude[*]}; i++));
#   do
#     echo -n "${exclude[$i]}" >> "$filename"
#     if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
#       echo -n "|" >> "$filename"
#     fi
#   done

# fi

# if using include
if [ "${#include[*]}" -gt 0 ]; then

  # list all the string and separate them with |
  for ((i=0; i < ${#include[*]}; i++));
  do
    echo -n "${include[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#include[*]} - 1)" ]; then
      echo -n " " >> "$filename"
    fi
  done

fi

echo "\" />" >> "$filename"

# file's footer
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
