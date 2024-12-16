  364  export UDEV=on
  365  python3 ls.py
  366  sudo apt install udev
  367  /lib/systemd/systemd-udevd --daemon
  368  sudo /lib/systemd/systemd-udevd --daemon
  369  udevadm monitor &
