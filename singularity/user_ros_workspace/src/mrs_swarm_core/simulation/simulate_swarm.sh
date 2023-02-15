#!/bin/bash

# #{ get the parameters
uav_num=0
exp_world="none"
odom_type="none"
config_file="none"
got_args=0
swarm_controller="none"
led_freqs=(6 10 15 30)
use_scheduler=false
episodes=0
obj_count=0

usage() 
{ 
  echo "Usage: simulate_swarm.sh [-f <config file path>] [-n <number_of_uavs>] [-w <world name from swarm_gazebo_resources/worlds>] [-o <odometry type "gps" or "hector">] [-s <swarm controller name>] [-e <number of episodes>] [-m <number of models/objects in the world>]"
  exit 1
}

# read command line options
while getopts "hf::n::w::o::s::e::m:" opts
do
  case ${opts} in
    h) usage ;;
    f) config_file=${OPTARG};;
    n) uav_num=${OPTARG} ;;
    w) exp_world=${OPTARG} ;;
    o) odom_type=${OPTARG} ;;
    s) swarm_controller=${OPTARG} ;;
    e) episodes=${OPTARG} ;;
    m) obj_count=${OPTARG} ;;
    \?)usage ;;
  esac
  got_args=1
done

# check if any arg was passed to the script
if [ ${got_args} -eq 0 ]; then
  usage
fi

# init useful variables
sim_resource_path=$(pwd)

# #{ source bash files to use ros-bash commands
source /opt/ros/noetic/setup.bash
source ../../../devel/setup.bash
# #}

# #{check if a config file is provided and its validity
if [ "${config_file}" == "none" ]; then
  echo No config file specified, using command line ARGS
else
  yq v $config_file
  if [ $? -ne 0 ]; then
    echo "Provided file either does not exist or is not a valid YAML file. Please use format:"
    exit 1
  fi
fi
# #}

# #{use command line options and fill the rest from config file
if [[ ${uav_num} == 0 ]]; then
  if [ "${config_file}" == "none" ]; then
    echo "Please specify number of UAVs for simulation"
    exit 1
  else
    uav_num=$(yq r ${config_file} --defaultValue 0 uav_num)
    if [[ ${uav_num} == 0 ]]; then
      echo "Please specify number of UAVs for simulation"
      exit 1
    fi
  fi
fi
echo "Number of uavs: $uav_num"

if [ "${odom_type}" == "none" ]; then
  if [ "${config_file}" == "none" ]; then
    echo "Please specify odometry type for simulation"
    exit 1
  else
    odom_type=$(yq r ${config_file} --defaultValue none odom_type)
    if [ "${odom_type}" == "none" ]; then
      echo "Please specify odometry type for simulation"
      exit 1
    fi
  fi
fi
echo "Using odometry: $odom_type"

if [ "${swarm_controller}" == "none" ]; then
  if [ "${config_file}" == "none" ]; then
    echo "Please specify a swarm_controller pkg for simulation"
    exit 1
  else
    swarm_controller=$(yq r ${config_file} --defaultValue none swarm_controller)
    if [ "${swarm_controller}" == "none" ]; then
      echo "Please specify a swarm_controller pkg for simulation"
      exit 1
    fi
  fi
fi
echo "Using swarm controller: $swarm_controller"

if [ "${config_file}" != "none" ]; then
  use_scheduler=$(yq r ${config_file} --defaultValue false use_scheduler)
  if [ "${use_scheduler}" == true ]; then
    episodes=$(yq r ${config_file} --defaultValue 0 episodes)

    sublog_dir=$(date +"%d_%m_%Y_%H_%M_%S")
    mkdir -p sim_logs
    mkdir -p "sim_logs/$sublog_dir"

    echo "Using scheduler with episodes: $episodes"

    obj_count=$(yq r ${config_file} --defaultValue 0 obj_count)
    if [[ ${obj_count} == 0 ]]; then
      echo "Please specify number of models/objects for world file"
      exit 1
    fi
    echo "Object count: $obj_count"
  fi
fi

if [ "${exp_world}" == "none" ]; then
  if [ "${config_file}" == "none" ]; then
    echo "Please specify world name for simulation"
    exit 1
  else
    exp_world=$(yq r ${config_file} --defaultValue none exp_world)

    if [ "${exp_world}" == "none" ]; then
      echo "Please specify world name for simulation"
      exit 1
    fi
    org_world_file=$(roscd swarm_gazebo_resources && pwd)'/worlds/'$exp_world'.world'

    if [ "${use_scheduler}" == true ]; then
      world_file='/tmp/swarm_custom.world'
    else
      world_file=$org_world_file
    fi
  fi
fi
echo "Using world: $world_file"

# #}

rviz_file=$(cd /tmp && pwd)'/swarm_rviz_config.yaml'
origin_frame='local_origin' # As this frame is the takeoff position in the world and it always exits, irrespective of odom_type
world_params=$sim_resource_path'/config/world_defs/world_'$odom_type'.yaml'

# #}

# #{CREATE THE WINDOWS

# WINDOWS that are common for all the drones (one copy needed)
if [ "${use_scheduler}" == true ]
then
  common_windows=('roscore' 'gazebo' 'rviz' 'scheduler' 'kill_session')
else
  common_windows=('roscore' 'gazebo' 'rviz')
fi

# WINDOWS specific for each of the drones
if [ "${odom_type}" == "gps" ]
then
  uav_windows=('status' 'swarm_controller' 'run_swarm_ctrl' 'goto' 'spawn' 'control' 'takeoff' 'bumper' 'shared_gps_aggr' 'global_origin_tf')
else
  if [ "${odom_type}" == "hector" ]
  then
    uav_windows=('status' 'swarm_controller' 'run_swarm_ctrl' 'goto' 'path_finder' 'hector' 'spawn' 'control' 'takeoff' 'bumper' 'global_origin_tf' 'hector_w_gps_tf')
  else
    echo Incorrect odometry type specified, exit; exit 1
  fi
fi
# #}

# #{THE COMMANDS FOR ALL THE WINDOWS
declare -A window_commands
window_commands['roscore']='roscore'
window_commands['gazebo']='waitForRos; roslaunch mrs_simulation simulation.launch gui:=true world_file:='$world_file''
window_commands['rviz']='waitForRos; sleep 2; rviz -d '$rviz_file''
window_commands['scheduler']='waitForRos; roslaunch swarm_utils scheduler.launch uav_count:='$uav_num' bag_file_dir:='$HOME'/workspace/src/mrs_swarm_core/simulation/sim_logs/'$sublog_dir' leader_count:=4 inter_uav_dist:=2.0 goal_x:=20 goal_y:=0 episode_count:=$EPISODE preflight_timeout:=15.0 max_timeout:=900.0'
window_commands['kill_session']='waitForRos; cd scripts; ./kill_session.sh $EPISODE'
window_commands['slow_gz']='waitForRos; sleep 10; gz physics -u 125'

window_commands['status']='waitForRos; roslaunch mrs_uav_status status.launch'
window_commands['swarm_controller']='waitForControl; roslaunch swarm_control_manager swarm_control_manager.launch controller_pkg:='$swarm_controller' origin_frame:='$origin_frame' uav_frame:=fcu_untilted'
window_commands['run_swarm_ctrl']='history -s rosservice call /$UAV_NAME/uav_manager/land;
history -s rosservice call /$UAV_NAME/swarm_control_manager/activate_controller;
history -s rosservice call /$UAV_NAME/swarm_control_manager/run_controller'
window_commands['goto']='history -s rosservice call /$UAV_NAME/path_finder/path_goto'
window_commands['path_finder']='waitForControl; roslaunch forest_path_finder path_finder.launch'
window_commands['hector']='waitForSimulation; roslaunch mrs_uav_general hector_slam.launch map_size:=3000'
window_commands['spawn']='waitForSimulation; sleep 2;
rosservice call /mrs_drone_spawner/spawn "$UAV_NUM $UAV_TYPE --pos_file '$sim_resource_path'/config/gazebo_config/init_uav_pose/'$exp_world'.yaml --enable-rangefinder --enable-rplidar"'
window_commands['control']='waitForOdometry; roslaunch mrs_uav_general core.launch 
config_uav_manager:='$sim_resource_path'/config/uav_sys_config/uav_manager.yaml
config_mpc_tracker:='$sim_resource_path'/config/uav_sys_config/mpc_tracker.yaml 
config_constraint_manager:='$sim_resource_path'/config/uav_sys_config/constraint_manager.yaml
debug:=true'
window_commands['takeoff']='waitForControl; sleep 2;
rosservice call /$UAV_NAME/control_manager/motors 1;
rosservice call /$UAV_NAME/mavros/cmd/arming 1;
rosservice call /$UAV_NAME/mavros/set_mode 0 offboard;
rosservice call /$UAV_NAME/uav_manager/takeoff'
window_commands['bumper']='waitForOdometry; roslaunch mrs_bumper bumper.launch'
window_commands['global_origin_tf']='waitForControl; roslaunch swarm_utils common_tf_pub.launch uav_frame:=fcu origin_frame:=local_origin global_frame:=global_origin init_pose_file:='$sim_resource_path'/config/gazebo_config/init_uav_pose/'$exp_world'.yaml'
window_commands['shared_gps_aggr']='waitForControl; roslaunch swarm_utils shared_gps_aggr.launch'
window_commands['hector_w_gps_tf']='waitForRos; rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 /$UAV_NAME/local_origin /$UAV_NAME/hector_origin'
# #}

# #{ change_directory()

change_directory() {
  if_is_link=`readlink $0`
  command_name=$0
  if [[ if_is_link ]]; then
    command_name=$if_is_link
  fi

  MY_PATH=`dirname "$if_is_link"`
  MY_PATH=`( cd "$MY_PATH" && pwd )`
  cd $MY_PATH
}

# #}

# #{ COMMON_CONFIG

COMMON_CONFIG='
{
  name: simulation,
  root: ./,
  startup_window: status,
  pre_window: export UAV_TYPE=f450; export ODOMETRY_TYPE='$odom_type'; export RUN_TYPE=simulation; export WORLD_FILE='$world_params'; export SENSORS="garmin_down",
  windows: [$WINDOWS]
}'

# #}

# #{ generate_tmux_config

generate_tmux_config() {
  windows=""
  for window_name in "${common_windows[@]}"; do
    # create the pane
    panes="'export EPISODE=$episodes; ${window_commands[$window_name]}'"
    # create current window and add it to all the others
    current_window="{$window_name: {layout: tiled, panes: [$panes]}}"
    windows="$windows $current_window,"
  done

  for window_name in "${uav_windows[@]}"; do
    panes=""
    for ((i=0; i<uav_num; i++)); do
      # create the pane for current uav and add it to the panes
      uav_name="uav$((i+1))"
      led_freq=${led_freqs[i]}
      panes="$panes 'export UAV_NUM=$((i+1)); export LED_FREQ=$led_freq; export UAV_NAME=$uav_name; ${window_commands[$window_name]}',"
    done

    # create current window and add it to all the windows
    current_window="{$window_name: {layout: tiled, panes: [$panes]}}"
    windows="$windows $current_window,"
  done

  # get the default config and put the windows inside it
  tmux_config=$COMMON_CONFIG
  tmux_config=${tmux_config//\$WINDOWS/$windows}
  # echo $tmux_config
}

# #}

# change the directory to the directory of script
change_directory
# generate the rviz config
gen_rviz_cmd='./generate_rviz_config.sh -n '$uav_num
echo $(cd scripts && $gen_rviz_cmd)
# generate the tmux_config variable
generate_tmux_config

# #{ single exp, w/out record

if [ $use_scheduler == false ]; then

  # generate the tmux_config variable
  generate_tmux_config

  # write to the config and start tmuxinator
  rm .tmuxinator.yml -f
  echo $tmux_config > .tmuxinator.yml
  tmuxinator
else

# #}

# #{ multi episode exp w/ recording

#run each episode and look for tmux ses 'simulation' every second
#restart the simulation when the ses is not present
while [ $episodes != 0 ]; do
  sessions=$(tmux list-sessions -F '#{==:#{session_name},simulation}')  
  # echo $sessions
  new_exp=1

  for flag in $sessions; do
    # echo $flag
    if [ $flag == 1 ]; then
      new_exp=0
      break
    fi
  done

  if [ "$new_exp" == 1 ]; then
    episodes=$(expr $episodes - 1)
    echo 'episodes left: '$episodes

    # creating new world at the location
    create_world='./create_world.py --world '$org_world_file' --new_world /tmp/swarm_custom.world --model /home/mrs/workspace/src/mrs_swarm_core/ros_packages/swarm_gazebo_resources/models/tree_simple/model.sdf --model_count '$obj_count' --cur_config '$sim_resource_path'/config/gazebo_config/init_uav_pose/'$exp_world'.yaml -w 50 -l 50 --thresh_dist 3.5 --uav_count '$uav_num''
    echo $(cd scripts && $create_world)

    # generate the tmux_config variable
    generate_tmux_config

    # write to the config and start tmuxinator
    rm .tmuxinator.yml -f
    echo $tmux_config > .tmuxinator.yml
    tmuxinator
  fi
done;

fi

# #}
