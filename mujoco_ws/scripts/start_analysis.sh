#!/bin/bash

# tmux new-session \; \
#   send-keys 'tail -f /var/log/monitor.log' C-m \; \
#   split-window -v -p 75 \; \
#   split-window -h -p 30 \; \
#   send-keys 'top' C-m \; \
#   select-pane -t 1 \; \
#   split-window -v \; \
#   send-keys 'weechat' C-m \; \
#   set -g mouse on \;


tmux new-session \; \
  split-window -v \; \
  split-window -h -p 50\; \
  select-pane -t 0 \; \
  send-keys 'source ../../../../devel/setup.bash' Enter \; \
  send-keys 'roslaunch shield_planner profiling_and_coverage.launch --screen' \; \
  select-pane -t 1 \; \
  send-keys 'source ../../../../devel/setup.bash' Enter \; \
  send-keys 'roslaunch shield_planner sim.launch --screen' \; \
  select-pane -t 2 \; \
  send-keys 'source ../../../../devel/setup.bash' Enter \; \
  send-keys 'rosrun shield_planner testbed_compute_projectile.py' \; \
  set -g mouse on \;
