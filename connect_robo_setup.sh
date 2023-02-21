sudo modprobe can_dev
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ifconfig can0 up