tmux new-session -d -s s2
tmux rename-window -t s2:0 "w2"
tmux send-keys -t s2:0 "cd ~/ws" C-m
tmux send-keys -t s2:0 "python3 catch5.py" C-m
tmux send-keys -t s2:0 "echo 1234321 | sudo -S chmod 666 /dev/ttyUSB*" C-m
tmux send-keys -t s2:0 "ros2 run roarm_driver roarm_driver" C-m

tmux select-layout -t s2:0 tiled

# must be the last command
tmux attach-session -t s2
