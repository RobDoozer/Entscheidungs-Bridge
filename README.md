Stand von 22.5.2025: Bridge über ROS, Empfangen und Senden über CAN
Stand von 28.5.2025: Bridge über ROS, Empfangen und Senden über ROS (Stand 1)

ROS Workspace aktivieren:
cd ~/ros2_ws
source install/setup.bash

CAN starten:
sudo ip link set can0 up type can bitrate 500000

Bridge Node starten:
ros2 run can_interface can_bridge_node
