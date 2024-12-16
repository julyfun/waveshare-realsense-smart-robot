serial=$(python3 ../find_serial_tty.py)
echo '1234321' | sudo -S chmod 666 $serial
