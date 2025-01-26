sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

colcon test --packages-select driver --event-handlers console_direct+