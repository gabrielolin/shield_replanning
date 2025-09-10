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
  select-pane -t 0 \; \
  send-keys 'source ../../../../devel/setup.bash' Enter \; \
  send-keys 'roslaunch shield_planner preprocess_planner_abb.launch --screen preprocess_arg:=true' \; \
  set -g mouse on \;
