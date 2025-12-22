# 📘 README — VR Tele-Operated Robot With Autonomous Navigation (ROS2 + Web + VR)

Final Documentation — Abito, KRL Robotics Lab

## 🚀 Project Overview

This project builds a VR-capable tele-operated robot that can be controlled using a browser or VR headset, and can also navigate autonomously using LiDAR + SLAM + Nav2.

It combines:

Gazebo Simulation

ROS2 Humble

TurtleBot3 (Waffle model)

WebSockets

Node.js HTTPS server

WebXR (VR)

Real-time camera streaming

AMCL-based robot tracking

Click-to-navigate map

3D RobotViz inside the browser

## 🏭 Industrial Use Cases

This system mirrors real-world telepresence systems used by:

Boston Dynamics – remote robot control

NASA — tele-operated exploration

NVIDIA Omniverse robotics simulation

Hospitals for surgical teleoperation

Defense robots for hazardous areas


## 🎯 Project Features
1. VR Teleoperation

Enter VR mode using WebXR

HUD buttons for movement

Real-time 3D robot environment

Camera feed inside VR HUD

Click navigation inside VR

2. Manual Control (Web Teleop)

WASD keyboard

On-screen HUD buttons

Real-time velocity display

Camera + map displayed

3. Autonomous Navigation

Click on the map to send Nav2 goal

Yellow path visualization

Robot pose (green arrow)

Goal marker (red circle)

“Robot Started” / “Robot Reached Goal” messages

Prevents goal spam

4. Camera Streaming

ROS2 → cv_bridge → MJPEG

Node.js proxy → HTTPS

Works in browser + VR

5. RobotViz

3D tracked robot

Smooth motion (LERP)

AMCL pose updates

Map-aligned coordinate rendering

6. Multi-Mode UI

✔ Manual HUD mode
✔ Camera-only mode
✔ Autonomous navigation mode

## 🏗 Project Architecture
                   ┌──────────────┐
 VR / Browser  ───▶│ index_vr.html│
                   └──────┬───────┘
                          │  WebSocket Secure (WSS)
                   ┌──────▼─────────┐
                   │   server.js     │  ← HTTPS + WSS + Proxy
                   └──────┬─────────┘
                          │
                  WSS (json messages)
                          │
                   ┌──────▼──────────┐
                   │   bridge.py      │ ← ROS2 Node
                   └──────┬──────────┘
                          │
         ┌───────────────┴───────────────┐
         │                                │
     /cmd_vel                         /amcl_pose
         │                                │
   TurtleBot3 in                     Nav2 + SLAM
     Gazebo                           (autonomy)


## 📂 Project Structure

tele_vr_robot/
│
├── ros2_ws/
│   └── src/rosbridge_ws/
│       ├── bridge.py
│       ├── package.xml
│       └── setup.py
│
├── ros2_sim_ws/
│
├── ros_streamer/
│   └── image_streamer.py   (camera streamer)
│
└── web_vr/
    ├── server.js           (HTTPS + WebSocket server)
    ├── index_vr.html       (web interface)
    ├── vr_app.js           (3D + VR + map)
    ├── cert/               (SSL cert/key)
    └── assets/
        └── map.png         (converted from map.pgm)

## 📦 Full Installation Guide
## 1. Install ROS2 Humble

Follow official guide:


## 2. Install TurtleBot3 Packages
sudo apt install ros-humble-turtlebot3*
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc


## 3. Install Nav2 + SLAM

Already included in TB3 packages.
## 4. Install Node.js (server.js)

curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

Install dependencies:
npm install ws express https

## 5. Install Python camera dependencies
sudo apt install ros-humble-cv-bridge python3-opencv
pip install flask

## 📸 Generate Map (SLAM)
Terminal 1 — Start Gazebo
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Terminal 2 — SLAM
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

Teleop the robot using your web page
Terminal 3 — Save Map
ros2 run nav2_map_server map_saver_cli -f ~/map



Convert map:
convert ~/map.pgm ~/tele_vr_robot/web_vr/assets/map.png

## 🤖 Run Autonomous Navigation
Terminal 1 — Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Terminal 2 — Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

## 🌐 Start the Web Interface
Terminal 3 — HTTPS Server
cd ~/tele_vr_robot/web_vr
node server.js

Terminal 4 — ROS Bridge
python3 ~/tele_vr_robot/ros2_ws/src/rosbridge_ws/rosbridge_ws/bridge.py

Terminal 5 — Camera Streamer
python3 ~/tele_vr_robot/ros_streamer/image_streamer.py

Open Browser
https://localhost:8443/index_vr.html

## 🎮 How to Use the System
Mode 1 — Manual Teleoperation

WASD keys

HUD buttons

Robot moves in real-time

Camera + map shown

Autonomous nav disabled

Mode 2 — Camera Mode (Full Screen)

Entire screen is robot camera

No distractions

Best for VR

Only “Switch Mode” button visible

Mode 3 — Autonomous Mode

Map fully visible

Click to set Nav Goal

Red circle = goal

Green arrow = robot pose

Yellow line = path

While robot is moving → No new goals

## 🧠 Internal Message Format
Browser → Server.js
{ "type": "cmd_vel", "linear": 0.2, "angular": 0.0 }


Server.js → Bridge.py
{ "type": "nav_goal", "x": 1.2, "y": 0.5, "theta": 0 }


Bridge.py → Browser
{ "type": "amcl", "x": 0.54, "y": -1.23, "theta": 1.57 }

## 🎯 Troubleshooting

Camera Not Updating
Check camera streamer:
python3 image_streamer.py

RobotViz Not Moving
Check AMCL:
ros2 topic echo /amcl_pose

Nav Goal Not Working
Check Nav2 status:
ros2 topic echo /navigate_to_pose/_action/status

VR Not Entering
Chrome VR flags must be enabled.

## 🏁 Final Result

You successfully built a full telepresence + autonomy robot system:
VR interface
Manual controls
Autonomous navigation
Camera streaming
Web-based map + click nav
Robot pose visualization
Real-time 3D scene
Secure HTTPS + WSS

This is comparable to real-world industrial teleoperation platforms.

## ⭐ Credits
Developed by: Abito
Under: KRL Lab
Project: 3-1 Robotics / VR Robotics System
