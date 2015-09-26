#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'cameras'
tmux new-window -t $SESSION:4 -n 'ui'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'ppl_perception'
tmux new-window -t $SESSION:7 -n 'hrsi'
tmux new-window -t $SESSION:8 -n 'control'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch aaf_bringup aaf_core.launch db_path:=/opt/strands/aaf_datacentre"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_robot.launch with_mux:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_cameras.launch head_camera:=true head_ip:=left-cortex head_user:=strands chest_camera:=true chest_ip:=right-cortex chest_user:=strands"

tmux select-window -t $SESSION:4
tmux send-keys "rosparam set /deployment_language english && HOST_IP=192.168.0.100 DISPLAY=:0 roslaunch aaf_bringup aaf_ui.launch mary_machine:=right-cortex mary_machine_user:=strands"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_navigation.launch map:=/opt/strands/maps/WW_lab_15_09_25-cropped.yaml topological_map:=WW_lab_15_09_25 with_no_go_map:=false no_go_map:=bla with_human_aware:=true with_chest_xtion:=true mon_nav_config_file:=bla positionUpdate:=true"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 roslaunch perception_people_launch people_tracker_robot.launch machine:=left-cortex user:=strands depth_image:=/depth/image_rect_meters"

tmux select-window -t $SESSION:7
tmux send-keys "DISPLAY=:0 roslaunch hri16_experiment hrsi_robot_rec.launch"

tmux select-window -t $SESSION:8
tmux send-keys "DISPLAY=:0 rosrun hri16_experiment hri16_robot_rec.py _out_dir:=/localhome/strands/cdondrup"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
