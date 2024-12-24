tmux new-session -d -s my
tmux rename-window -t my:0 "my-win"
tmux send-keys -t my:0 "cd" C-m
tmux send-keys -t my:0 "python3 -m ws.ttf_server" C-m

tmux split-window -v -t my:0
tmux send-keys -t my:1 "cd ~/ws" C-m
tmux send-keys -t my:1 "python3 track_my.py" C-m


tmux select-layout -t my:0 tiled

# must be the last command
tmux attach-session -t my
