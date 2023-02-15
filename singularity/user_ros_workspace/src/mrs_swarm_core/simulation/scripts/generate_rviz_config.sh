#!/usr/bin/env bash

# get the parameters
uav_colors=('220; 0; 0' '85; 170; 0' '85; 170; 255' '255; 85; 255')

frame="/global_origin"
uav_num=1

# read command line options
while getopts hn:f: opts; do
  case ${opts} in
    h) echo 'generate_rviz_config [-n number_of_uavs] [-f frame_name] [-d directory]'; exit 1 ;;
    n) uav_num=${OPTARG} ;;
    f) frame=${OPTARG} ;;
  esac
done

file="$(cd /tmp && pwd)/swarm_rviz_config.yaml"
uav_names=(${all_uav_names[@]:0:${uav_num}})
echo "Generating rviz config file: $file (Number of uavs: $uav_num)"

# #{ generate_config()

generate_config() {
  displays=${GENERAL_DISPLAYS}

  # Add the displays for all the uavs
  for ((i=1; i<=uav_num; i++)); do
    uav_name="uav$i"
    # replace all the $UAV_NAME with real uav_name
    uav_displays=${UAV_DISPLAYS//\$UAV_NAME/$uav_name}
    displays="$displays $uav_displays"
  done

  config=$CONFIG
  config=${config//\$frame/$frame}
  config=${config//\$DISPLAYS/$displays}

  echo $config > $file 
}

# #}

# #{ GENERAL_DISPLAYS

GENERAL_DISPLAYS='
{Cell Size: 1, Class: rviz/Grid, Color: 140; 140; 140, Enabled: true, Line Style: {Line Width: 0.03, Value: Lines}, Name: Grid, Plane Cell Count: 10, Reference Frame: <Fixed Frame>},
{Cell Size: 10, Class: rviz/Grid, Color: 160; 160; 160, Enabled: true, Line Style: {Line Width: 0.03, Value: Lines}, Name: Grid, Plane Cell Count: 10, Reference Frame: <Fixed Frame>},
{Name: Transforms, Class: rviz/TF, Enabled: false, Marker Scale: 1, Show Arrows: true, Show Axes: true, Show Names: true, Frame Timeout: 15},
'

# #}

# #{ UAV_DISPLAYS

# params: $UAV_NAME
UAV_DISPLAYS='
{
  Name: $UAV_NAME,
  Class: rviz/Group,
  Enabled: true,
  Displays: [
    {Name: Pose, Class: rviz/Odometry, Topic: /$UAV_NAME/control_manager/cmd_odom, Enabled: true, Covariance: {value: false}, Keep: 1, Angle Tolerance: 0.1, Position Tolerance: 0.1, Shape: {Alpha: 1, Axes Length: 1, Axes Radius: 0.1, Color: 255; 25; 0, Head Length: 0.1, Head Radius: 0.05, Shaft Length: 0.3, Shaft Radius: 0.02, Value: Arrow}},
    {Name: Laser Scan, Class: rviz/LaserScan, Topic: /$UAV_NAME/rplidar/scan, Enabled: false, Size (m): 0.1, Color Transformer: Intensity, Position Transformer: XYZ, Use rainbow: true, Use Fixed Frame: true},
      {Name: Hector map, Class: rviz/Map, Topic: /$UAV_NAME/hector_mapping/map, Enabled: false, Alpha: 0.3},
    {Name: Path, Class: rviz/Path, Topic: /$UAV_NAME/path_finder/path, Enabled: true, Alpha: 1, Color: 25; 255; 0, Head Diameter: 0.3, Head Length: 0.2, Length: 0.3, Line Style: Lines, Line Width: 0.03, Radius: 0.03, Shaft Diameter: 0.1, Shaft Length: 0.1},
    {Name: Shared GPS Poses, Class: mrs_rviz_plugins/PoseWithCovarianceArray, Topic: /$UAV_NAME/swarm_utils/shared_gps_aggr/aggr_poses, Enabled: true, Shape: Axes, Alpha: 1, Axes_Length: 1, Axes Radius: 0.1, Covariance: {Value: false}},
    {Name: UVDAR Poses, Class: mrs_rviz_plugins/PoseWithCovarianceArray, Topic: /$UAV_NAME/uvdar/filteredPoses, Enabled: true, Shape: Axes, Alpha: 1, Axes_Length: 1, Axes Radius: 0.1, Covariance: {Value: false}},
  ]
},
'

# #}

# #{ CONFIG

# params: $frame, $DISPLAYS
CONFIG='
{
"Visualization Manager": 
  {
    Name: root,
    Enabled: true,
    "Global Options": {Background Color: 160; 160; 160, Default Light: true, Fixed Frame: $frame, Frame Rate: 30}, 

    Displays: [$DISPLAYS],
  
    Tools: [
      {Class: rviz/Interact, Hide Inactive Objects: true}, {Class: rviz/MoveCamera}, {Class: rviz/Select}, {Class: rviz/FocusCamera}, {Class: rviz/Measure}, 
      {Class: rviz/SetInitialPose, Theta std deviation: 0.2617993950843811, Topic: /initialpose, X std deviation: 0.5, Y std deviation: 0.5}, {Class: rviz/SetGoal, Topic: /move_base_simple/goal}, {Class: rviz/PublishPoint, Single click: true, Topic: /clicked_point}
    ],

    Views: { 
      Current: {Name: Current View, Class: rviz/Orbit, Distance: 10, Focal Shape Fixed Size: true, Focal Shape Size: 0.05, Near Clip Distance: 0.001, Pitch: 0.78, Target Frame: <Fixed Frame>, Value: Orbit (rviz), Yaw: 0.78},
      Saved: ~,
    }
  },

Panels: [
  {Name: Displays, Class: rviz/Displays, Help Height: 78, Property Tree Widget: {Expanded: [/Global Options1]}, Splitter Ratio: 0.5},
  {Name: Selection, Class: rviz/Selection},
  {Name: Tool Properties, Class: rviz/Tool Properties, Expanded: [], Splitter Ratio: 0.5},
  {Name: Views, Class: rviz/Views, Expanded: [/Current View1], Splitter Ratio: 0.5},
  {Name: Time, Class: rviz/Time}
],
Preferences: {PromptSaveOnExit: false},
Toolbars: {toolButtonStyle: 2},

"Window Geometry": {
  Displays: {collapsed: false},
  Selection: {collapsed: false},
  Time: {collapsed: false},
  "Tool Properties": {collapsed: false},
  Views: {collapsed: false},
  "Hide Left Dock": false,
  "Hide Right Dock": false,
  "QMainWindow State": 000000ff00000000fd00000004000000000000026900000374fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003b00000374000000c700fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000374fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003b00000374000000a000fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000007800000003efc0100000002fb0000000800540069006d00650100000000000007800000023f00fffffffb0000000800540069006d00650100000000000004500000000000000000000003fc0000037400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000,
}
}
'

# #}

# call the main function after defining all the variables
generate_config
