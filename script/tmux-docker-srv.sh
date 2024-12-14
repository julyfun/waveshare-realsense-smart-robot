tmux new-session -d -s my_session
tmux rename-window -t my_session:0 'Window1'
tmux send-keys -t my_session:0 'docker start 7f' C-m
tmux send-keys -t my_session:0 'docker exec -u julyfun -it 7f bash' C-m
tmux send-keys -t my_session:0 'echo 1234321 | sudo -S chmod 666 /dev/ttyUSB0' C-m
tmux send-keys -t my_session:0 'ros2 run roarm_driver roarm_driver' C-m

tmux split-window -v -t my_session:0
tmux send-keys -t my_session:0.1 'sleep 3' C-m
tmux send-keys -t my_session:0.1 'docker exec -u julyfun -it 7f bash' C-m
tmux send-keys -t my_session:0.1 'ros2 launch roarm_moveit_cmd command_control.launch.py use_rviz:=false' C-m

tmux split-window -h -t my_session:0
tmux send-keys -t my_session:0.2 'sleep 3' C-m
tmux send-keys -t my_session:0.2 'docker exec -u julyfun -it 7f bash' C-m
tmux send-keys -t my_session:0.2 'ros2 run roarm_moveit_cmd setgrippercmd' C-m

tmux split-window -h -t my_session:0
tmux send-keys -t my_session:0.3 'sleep 3' C-m
tmux send-keys -t my_session:0.3 'docker exec -u julyfun -it 7f bash' C-m
tmux send-keys -t my_session:0.3 'ros2 run roarm_moveit_cmd movepointcmd' C-m

tmux select-layout -t my_session:0 tiled

# must be the last command
tmux attach-session -t my_session
