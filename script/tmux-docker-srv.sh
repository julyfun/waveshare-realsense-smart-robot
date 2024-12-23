# [chmod]
echo '1234321' | su julyfun
cd ~/ws
serial=$(python3 find_serial_tty.py)
echo '1234321' | sudo -S chmod 666 /dev/ttyUSB*

# [arm server]
hash="a6"
tmux new-sessi -d -s my
tmux send-keys -t my:0 "docker start $hash" C-m
tmux send-keys -t my:0 "docker exec -u julyfun -it $hash bash" C-m
tmux send-keys -t my:0 "echo 1234321 | sudo -S chmod 666 /dev/ttyUSB*" C-m
tmux send-keys -t my:0 "ros2 run roarm_driver roarm_driver" C-m

tmux split-win -v -t my:0
tmux send-keys -t my:0.1 "sleep 3" C-m
tmux send-keys -t my:0.1 "docker exec -u julyfun -it $hash bash" C-m
tmux send-keys -t my:0.1 "ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=false" C-m

tmux split-win -h -t my:0
tmux send-keys -t my:0.2 "sleep 3" C-m
tmux send-keys -t my:0.2 "docker exec -u julyfun -it $hash bash" C-m
tmux send-keys -t my:0.2 "ros2 run roarm_moveit_cmd setgrippercmd" C-m

tmux split-win -v -t my:0
tmux send-keys -t my:0.3 "sleep 3" C-m
tmux send-keys -t my:0.3 "docker exec -u julyfun -it $hash bash" C-m
tmux send-keys -t my:0.3 "ros2 run roarm_moveit_cmd movepointcmd" C-m

tmux split-win -h -t my:0
tmux send-keys -t my:0.4 "sleep 6" C-m
tmux send-keys -t my:0.4 "docker exec -u julyfun -it $hash bash" C-m
tmux send-keys -t my:0.4 "cd /home/julyfun/roarm_ws_em0/src/script" C-m
tmux send-keys -t my:0.4 "python3 arm_server.py" C-m

tmux split-win -v -t my:0
tmux send-keys -t my:0.5 "sleep 6" C-m
tmux send-keys -t my:0.5 "docker exec -u julyfun -it $hash bash" C-m
tmux send-keys -t my:0.5 "cd /home/julyfun/roarm_ws_em0/src/script" C-m
tmux send-keys -t my:0.5 "python3 rs_arm_tf2.py" C-m

tmux select-layout -t my:0 tiled

# [track person]
# tmux split-win -h -t my:0
# tmux send-keys -t my:0.6 "cd" C-m
# tmux send-keys -t my:0.6 "python3 -m ws.ttf_server" C-m

# tmux split-win -v -t my:0
# tmux send-keys -t my:0.7 "cd ~/ws" C-m
# tmux send-keys -t my:0.7 "python3 track_person.py" C-m


# [catch obj]
tmux select-layout -t my:0 tiled
tmux split-win -v -t my:0
catch=$(tmux list-panes -t my:0 -F "#{pane_id}" | tail -n 1)
tmux send-keys -t "$catch" "cd ~/ws" C-m
tmux send-keys -t "$catch" "python3 catch5.py" C-m

# [end]
tmux select-layout -t my:0 tiled
tmux attach-session -t my
