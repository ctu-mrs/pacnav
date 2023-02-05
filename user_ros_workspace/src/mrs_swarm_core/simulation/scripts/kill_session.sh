#!/bin/bash

killp () {

  if [ $# -eq 0 ]; then
    pes=$( cat ) 
  else
    pes=$1
  fi
  
	for child in $(ps -o pid,ppid -ax | \
   awk "{ if ( \$2 == $pes ) { print \$1 }}")
  do
    # echo "Killing child process $child because ppid = $pes"
    killp $child
  done
        
  # echo "killing $1"
  kill -9 "$1" > /dev/null 2> /dev/null
}
alias killp='killp()'

killSession () {
  pids=`tmux list-panes -s -F "#{pane_pid} #{pane_current_command}" | grep -v tmux | awk {'print $1'}`

  for pid in $pids; do
    killp "$pid" &
    # echo $pid
  done
}
alias killSession='killSession()'

ep_count=$1
init_time=`timeout 5s rostopic echo /clock -n 1 | grep secs | head -n 1 | awk {'print $2'}`
# echo "init_time:" $init_time

while true; do

  if [ ${#init_time} == 0 ]; then
    init_time=`timeout 5s rostopic echo /clock -n 1 | grep secs | head -n 1 | awk {'print $2'}`
    # echo "refresh init_time:" $init_time

  fi

  current_time=`timeout 3s rostopic echo /clock -n 1 | grep secs | head -n 1 | awk {'print $2'}`

  # a topic from the candle publisher
  stop_topic=`timeout 5s rostopic echo /stop_sim -n 1 --noarr | grep secs | head -n 1 | awk {'print $2'}`
  # mpc1=`timeout 5s rostopic echo /uav1/trackers_manager/mpc_tracker/cmd_pose -n 1 --noarr | grep secs | head -n 1 | awk {'print $2'}`
  # mpc2=`timeout 5s rostopic echo /uav2/trackers_manager/mpc_tracker/cmd_pose -n 1 --noarr | grep secs | head -n 1 | awk {'print $2'}`
  # mpc3=`timeout 5s rostopic echo /uav3/trackers_manager/mpc_tracker/cmd_pose -n 1 --noarr | grep secs | head -n 1 | awk {'print $2'}`
  # mpc4=`timeout 5s rostopic echo /uav4/trackers_manager/mpc_tracker/cmd_pose -n 1 --noarr | grep secs | head -n 1 | awk {'print $2'}`
  # mpc5=`timeout 5s rostopic echo /uav5/trackers_manager/mpc_tracker/cmd_pose -n 1 --noarr | grep secs | head -n 1 | awk {'print $2'}`

  # echo $stop_topic
  # echo ${#stop_topic}

  # check if stop is publishing, ie len(stop_topic) > 0
  if [ ${#stop_topic} != 0 ]; then

    echo "Stopping..."
    echo "$ep_count $(expr $current_time - $init_time)" >> ../sim_logs/kill_log.txt
    killSession
    break
  fi

  # check for timeouts
  # if [ -z $mpc1 ] || [ -z $mpc2 ] || [ -z $mpc3 ] || [ -z $mpc4 ] || [ -z $mpc5 ]; then

  #   echo "MPC Timeout..."
  #   killSession
  #   break
  # fi

  # dt_since_start=$(( $current_time - $started ))
  # echo "$dt_since_start"
  # # restart once in 10 minutes
  # if [ "$dt_since_start" -gt 900 ]; then
  #   killSession
  #   break
  # fi

  # substract current time
  # collision_topic_dt=$(( $current_time - $collision_topic ))
  # mpc1_dt=$(( $current_time - $mpc1 ))
  # mpc2_dt=$(( $current_time - $mpc2 ))
  # mpc3_dt=$(( $current_time - $mpc3 ))
  # mpc4_dt=$(( $current_time - $mpc4 ))
  # mpc5_dt=$(( $current_time - $mpc5 ))

  # echo "$collision_topic_dt"

  # if [ "$collision_topic_dt" -gt 5 ] || [ "$mpc1_dt" -gt 5 ] || [ "$mpc2_dt" -gt 5 ] || [ "$mpc3_dt" -gt 5 ] || [ "$mpc4_dt" -gt 5 ] || [ "$mpc5_dt" -gt 5 ]; then

  #   echo "Killing, not receiving messages.."
  #   echo "$current_time" >> dts.txt
  #   killSession
  #   break
  # fi

  # if [ "$collision_topic_dt" -gt 5 ]; then

  #   echo "Killing, not receiving messages.."
  #   echo "$current_time" >> dts.txt
  #   killSession
  #   break
  # fi

done;

exit
