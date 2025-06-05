Versionen:
Stand von 22.5.2025: Bridge über ROS, Empfangen und Senden über CAN

Stand von 5.6.2025: Bridge über ROS, Empfangen und Senden über ROS

ROS:

ROS Workspace aktivieren:

cd ~/ros2_ws

source install/setup.bash

Bridge Node starten:

ros2 run can_interface can_bridge_node

CAN:

CAN starten:

sudo ip link set can0 up type can bitrate 500000

CAN Nachrichten über Terminal schicken:

cansend can0 100#DEADBEEF

Geloggte Daten befinden sich in csv Datei im ros2_ws Ordner.

