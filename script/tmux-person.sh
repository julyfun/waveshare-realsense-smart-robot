tmux new-session -d -s person
tmux rename-window -t person:0 "person-win"
tmux send-keys -t person:0 "cd" C-m
tmux send-keys -t person:0 "python3 -m ws.ttf_server" C-m

tmux split-window -v -t person:0
tmux send-keys -t person:1 "cd ~/ws" C-m
tmux send-keys -t person:1 "python3 track_person.py" C-m


tmux select-layout -t person:0 tiled

# must be the last command
tmux attach-session -t person
