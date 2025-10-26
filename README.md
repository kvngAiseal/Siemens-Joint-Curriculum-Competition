# Advanced Smart Manufacturing System with MPS Sorting Stations
## Siemens Joint Curriculum Competition Project

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Computer%20Vision-orange.svg)](https://github.com/ultralytics/ultralytics)
[![Siemens](https://img.shields.io/badge/Siemens-PLC%20SCL-teal.svg)](https://www.siemens.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An advanced multi-robot systems integration project demonstrating intelligent material handling and color-based sorting in a smart factory environment. This project combines Siemens PLC-controlled MPS sorting stations with robotic manipulation and autonomous mobile transport.

**Competition:** Siemens Joint Curriculum Competition  
**Authors:** Sela Shapira and Adebolanle Okuboyejo  
**Course:** Robotics Systems Integration

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Components](#hardware-components)
- [Software Stack](#software-stack)
- [Installation](#installation)
- [System Workflow](#system-workflow)
- [Color Detection Logic](#color-detection-logic)
- [Remote Integration Solution](#remote-integration-solution)
- [Code Structure](#code-structure)
- [Usage](#usage)
- [ROS2 Topics](#ros2-topics)
- [PLC State Machine](#plc-state-machine)
- [Demonstration Video](#demonstration-video)
- [Troubleshooting](#troubleshooting)
- [Future Improvements](#future-improvements)
- [Acknowledgments](#acknowledgments)

---

## 🎯 Overview

This project represents an advanced evolution of our smart manufacturing system, integrating:

- **2x Siemens MPS Sorting Stations** - Automated lid sorting with color detection
- **1x Turtlebot 4 Mobile Robot** - Autonomous material transport
- **2x UR10 Collaborative Robot Arms** - Pick and place operations with gripper control
- **Siemens PLC (SCL Programming)** - Industrial control logic
- **YOLOv8 Computer Vision** - AI-powered color detection
- **Remote System Integration** - Cross-location coordination via video streaming

### Key Innovations

- ✅ Industrial PLC integration with robotic systems
- ✅ AI-powered computer vision for color detection
- ✅ Multi-sensor fusion (color sensor + metal detection)
- ✅ Remote system coordination via video streaming
- ✅ User-interactive selection system with real-time detection
- ✅ Automated shelf organization by color
- ✅ Industry 4.0 smart factory demonstration

---

## 🏗️ System Architecture

### Physical Layout

```
┌─────────────────────────────────────────────────────────────────┐
│                         REMOTE LOCATION                          │
│  ┌──────────────┐  Conveyor  ┌──────────────┐                  │
│  │  MPS Station │────────────►│  MPS Station │                  │
│  │      #1      │             │      #2      │                  │
│  │  (Lid Pickup)│             │ (Sorting)    │                  │
│  └──────────────┘             └──────────────┘                  │
│         │                            │                           │
│         │                     [Color Sensor]                     │
│         │                     [Metal Detector]                   │
│         │                     [3 Sorting Trays]                  │
│         └────────────── Zoom Stream ────────────────────┐        │
└─────────────────────────────────────────────────────────│────────┘
                                                          │
                                                    [Video Feed]
                                                          │
┌─────────────────────────────────────────────────────────▼────────┐
│                      ROBOTICS LAB                                 │
│                                                                   │
│  ┌─────────┐        ┌────────────┐        ┌─────────┐           │
│  │ Camera  │───────►│ Big Screen │        │  Box    │           │
│  │ (YOLOv8)│        │ (Displays  │        │(Simulates│          │
│  └─────────┘        │   Zoom)    │        │   MPS)  │           │
│                     └────────────┘        └────┬────┘           │
│                                                 │                 │
│      ┌──────────┐        ┌──────────────┐      │                 │
│      │  UR10 #1 │◄──────►│ Turtlebot 4  │◄─────┘                 │
│      │ (Pickup) │        │   (Mobile)   │                        │
│      │ + Gripper│        │              │                        │
│      └──────────┘        └──────┬───────┘                        │
│                                 │                                 │
│                          ┌──────▼──────┐                         │
│                          │   UR10 #2   │                         │
│                          │  (Storage)  │                         │
│                          │   + Shelf   │                         │
│                          │  3 Levels   │                         │
│                          └─────────────┘                         │
└───────────────────────────────────────────────────────────────────┘
```

### Communication Flow

```
┌────────────────────────────────────────────────────────────────┐
│                    ROS2 TOPIC COMMUNICATION                     │
└────────────────────────────────────────────────────────────────┘

     /lid_task              /robot_status         /turtlebot_status
         │                       │                        │
         ▼                       ▼                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  UR10 #1 Node   │◄──►│  Turtlebot Node │◄──►│  UR10 #2 Node   │
│ (Color Detect)  │    │  (Navigation)   │    │  (Storage)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                                              │
         ▼                                              ▼
   Socket Server                              /ur10_2/urscript_interface
   (Port 5000)                                /script_command
```

---

## 🔧 Hardware Components

### Siemens MPS Stations
- **2x MPS Sorting Stations**
  - Siemens S7-1200 PLC
  - Conveyor belt system
  - Color sensor (optical reflection)
  - Metal detector sensor
  - 3 sorting trays
  - Pneumatic actuators
  - Pick and place mechanism with vacuum gripper

### Robotic Systems
- **1x Turtlebot 4 Mobile Robot**
  - ROS2 Jazzy compatible
  - Navigation stack (Nav2)
  - LiDAR for localization
  - Flat top platform for lid transport

- **2x Universal Robots UR10**
  - 6-axis collaborative robot arms
  - RG6 gripper on UR10 #1
  - Socket-based communication
  - URScript programming capability

### Computer Vision
- **Camera System** (USB/IP Camera)
  - YOLOv8 compatible
  - Mounted for screen viewing
  - Real-time detection capability

- **Display System**
  - Large screen for Zoom feed display
  - Proper lighting conditions

### Storage System
- **3-Level Shelf** (UR10 #2)
  - Level 1: Red lids
  - Level 2: Silver lids
  - Level 3: Black lids

---

## 💻 Software Stack

### Core Technologies
- **ROS2 Jazzy Jalisco** - Robot coordination and communication
- **Python 3.10+** - Primary programming language
- **Siemens TIA Portal** - PLC programming environment
- **SCL (Structured Control Language)** - PLC state machine logic
- **YOLOv8 (Ultralytics)** - Computer vision model (`Yolo_4_lids.pt`)
- **OpenCV** - Image processing
- **URScript** - UR10 robot programming
- **Socket Programming** - UR10 communication interface

### Python Dependencies
```bash
# ROS2
rclpy
std_msgs
geometry_msgs

# Computer Vision
ultralytics>=8.0.0  # YOLOv8
opencv-python>=4.5.0
numpy
pillow

# UR10 Control
ur_robot_driver

# Turtlebot 4
turtlebot4_navigation
```

---

## 📦 Installation

### 1. ROS2 Jazzy Setup

```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt update
sudo apt install ros-jazzy-desktop

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/kvngAiseal/Siemens-Joint-Curriculum-Competition.git
cd ~/ros2_ws
```

### 3. Install Computer Vision Dependencies

```bash
# Install YOLOv8 and OpenCV
pip3 install ultralytics opencv-python numpy pillow torch torchvision

# Place your trained model
# Ensure Yolo_4_lids.pt is in the same directory as moving_robot1.py
```

### 4. Install ROS2 Dependencies

```bash
# Install robot packages
sudo apt install ros-jazzy-turtlebot4-desktop ros-jazzy-turtlebot4-navigation
sudo apt install ros-jazzy-ur-robot-driver

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 6. Network Configuration

```bash
# Add to ~/.bashrc
export ROS_DOMAIN_ID=30
source ~/ros2_ws/install/setup.bash

# Configure UR10 IP addresses in the code
# moving_robot1.py: socket server on 0.0.0.0:5000
# adding_conditions_4_gripper_urscript.script: Update IP to your ROS2 machine
```

---

## 🔄 System Workflow

### Complete Process Flow

```
┌────────────────────────────────────────────────────────────────┐
│                    SYSTEM OPERATION SEQUENCE                    │
└────────────────────────────────────────────────────────────────┘

PHASE 1: MPS SORTING (Remote Location - PLC Controlled)
────────────────────────────────────────────────────────
Step 1-2: MPS #1 picks and pushes lid to conveyor
Step 3-4: MPS #2 receives lid, activates pick mechanism
Step 5:   COLOR DETECTION & SORTING:
          ├─ IF Color_sensor=TRUE AND Metal_sensor=FALSE → Sorter_1 (RED)
          ├─ IF Color_sensor=TRUE AND Metal_sensor=TRUE  → Sorter_2 (SILVER)
          └─ IF Color_sensor=FALSE                       → Pass through (BLACK)
Step 6-8: Complete sorting cycle, return to Step 1

PHASE 2: USER SELECTION & DETECTION (Robotics Lab)
──────────────────────────────────────────────────
1. User runs colour_selection.py or moving_robot1.py
2. User selects lid(s): e.g., "red_lid=3,black_lid=2"
3. YOLOv8 detects selected lid colors on screen
4. Publishes to /lid_task topic

PHASE 3: INITIAL COMMUNICATION
───────────────────────────────
5. UR10 #1 publishes "UR10_1 is ready to start." (20x)
6. Turtlebot receives signal and navigates to Position 1 [3.769, 2.637]

PHASE 4: PICKUP (UR10 #1)
──────────────────────────
7. Turtlebot publishes "Turtlebot reached picking place."
8. UR10 #1 executes gripper script:
   - Move to target pose
   - Open gripper (RG6: 110mm)
   - Descend to lid
   - Close gripper (RG6: 0mm)
   - Pick lid from box
9. UR10 sends "done" via socket
10. UR10 #1 publishes "UR10_1: Finished loading the package." (25x)

PHASE 5: TRANSPORT
──────────────────
11. Turtlebot navigates to Position 2 [2.816, 2.832]
12. Turtlebot publishes "Turtlebot reached delivery location."

PHASE 6: STORAGE (UR10 #2)
───────────────────────────
13. UR10 #2 receives /lid_task message
14. Waits for FlexPendant socket message "ready"
15. Moves to appropriate shelf level:
    - Red lid    → movej([1.8, -1.57, -0.86, -0.72, 1.28, 1.62])
    - Silver lid → movej([1.63, -1.38, -1.39, -0.36, 1.52, 1.62])
    - Black lid  → movej([1.64, -1.31, -1.81, -0.05, 1.52, 1.62])
16. Places lid on designated shelf level
17. System ready for next cycle
```

---

## 🎨 Color Detection Logic

### PLC Sensor Logic (Step 5 of State Machine)

```scl
IF "Colour_sensor" AND NOT "Metal_sensor" AND "Timer3_finished" THEN
    "Sorter_1" := TRUE;        // Activate RED tray
    "Stopper" := FALSE;
    "Timer4_start" := 1;
    #Step := 6;

ELSIF "Colour_sensor" AND "Metal_sensor" AND "Timer3_finished" THEN
    "Sorter_2" := TRUE;        // Activate SILVER tray
    "Stopper" := FALSE;
    "Timer4_start" := 1;
    #Step := 6;

ELSIF "Colour_sensor" = FALSE AND "Metal_sensor" = FALSE AND "Timer3_finished" THEN
    "Stopper" := FALSE;        // Pass through - BLACK tray
    "Timer4_start" := 1;
    #Step := 6;
END_IF;
```

### Detection Decision Tree

```
              [Lid Detected at Sensor Position]
                           │
                           ▼
                  ┌────────────────┐
                  │  Color Sensor  │
                  │   (Optical)    │
                  └────────┬───────┘
                           │
              ┌────────────┴────────────┐
              │                         │
         [REFLECTS]              [NO REFLECTION]
              │                         │
              ▼                         ▼
    ┌──────────────────┐        ┌─────────────┐
    │ Metal Detector?  │        │   BLACK     │
    └────────┬─────────┘        │  Tray 3     │
             │                  │(Pass Through)│
        ┌────┴────┐             └─────────────┘
        │         │
    [DETECTED] [NOT DETECTED]
        │         │
        ▼         ▼
   ┌────────┐ ┌────────┐
   │ SILVER │ │  RED   │
   │ Tray 2 │ │ Tray 1 │
   └────────┘ └────────┘
```

### Classification Rules

| Color Sensor | Metal Detector | Result | Sorter Action |
|--------------|----------------|--------|---------------|
| ✅ TRUE (Reflects) | ❌ FALSE | **RED** | Activate Sorter_1 |
| ✅ TRUE (Reflects) | ✅ TRUE | **SILVER** | Activate Sorter_2 |
| ❌ FALSE (No Reflection) | N/A | **BLACK** | Pass through (no sorter) |

---

## 🌐 Remote Integration Solution

### Challenge
MPS stations and robotic systems were in different physical locations, making direct hardware integration impossible.

### Creative Solution

**Video Streaming Bridge:**
```
MPS Sorting (Remote) → Zoom Stream → Screen Display → YOLOv8 Detection → ROS2 System
```

**Physical Synchronization:**
- Replica box placed at UR10 #1 with exact MPS dimensions
- Lids manually arranged in sorted order (LEFT to RIGHT)
- Example: MPS sorts **RED → BLACK → SILVER** → Box arranged identically

### Implementation Details

1. **Zoom Streaming Setup**
   - High-quality video (720p minimum)
   - Stable internet connection
   - Clear view of sorted lids

2. **YOLOv8 Detection** (`moving_robot1.py`)
   - Real-time frame capture from camera
   - Model: `Yolo_4_lids.pt`
   - Classes: `red_lid`, `black_lid`, `silver_lid`
   - Confidence threshold filtering

3. **Physical Box Replica**
   - Exact dimensional match to MPS station
   - Position markers for each tray location
   - Consistent lighting conditions

---

## 💾 Code Structure

### Repository Files

```
Siemens-Joint-Curriculum-Competition/
├── SCL_CODE.txt                              # PLC state machine logic
├── colour_selection.py                       # Simple task publisher
├── moving_robot1.py                          # UR10 #1 + YOLOv8 detection
├── moving_robot2_to_shelves.py              # UR10 #2 shelf controller
├── moving_mobile_robot.py                    # Turtlebot coordinator
├── adding_conditions_4_gripper_urscript.script  # UR10 gripper control
└── link_for_demonstration_video.txt         # Demo video URL
```

### Key Components

#### 1. `moving_robot1.py` - UR10 #1 Controller
**Features:**
- YOLOv8 real-time lid detection
- User input for lid selection (e.g., `red_lid=3,black_lid=2`)
- Socket server (port 5000) for UR10 communication
- ROS2 publishers:
  - `/robot_status` - Status updates
  - `/lid_task` - Detected lid information
- Multi-threaded: detection + socket server

**Key Methods:**
```python
def detect_lid_color():        # YOLOv8 inference loop
def publish_initial_message(): # "UR10_1 is ready to start."
def publish_final_message():   # "Finished loading the package."
def run_socket_server():       # Listen for UR10 "done" signal
```

#### 2. `moving_mobile_robot.py` - Turtlebot Coordinator
**Features:**
- Dual UR10 communication support
- Phase-based state machine (0→1→2)
- Publishers:
  - `/turtlebot_status` - Status for UR10 #1
  - `/turtlebot_status_2` - Status for UR10 #2
- Subscriber: `/robot_status` - Commands from UR10 #1

**Navigation Waypoints:**
```python
Initial: [1.158, 4.177]   # Starting position
Position 1: [3.769, 2.637]  # UR10 #1 pickup
Position 2: [2.816, 2.832]  # UR10 #2 delivery
```

#### 3. `moving_robot2_to_shelves.py` - UR10 #2 Controller
**Features:**
- Predefined shelf poses for each color
- Socket server (port 5000) for FlexPendant communication
- Subscriber: `/lid_task` - Task information
- Publisher: `/ur10_2/urscript_interface/script_command` - URScript commands

**Shelf Positions (Joint Angles):**
```python
'red_lid':    [1.8,  -1.57, -0.86, -0.72, 1.28, 1.62]
'silver_lid': [1.63, -1.38, -1.39, -0.36, 1.52, 1.62]
'black_lid':  [1.64, -1.31, -1.81, -0.05, 1.52, 1.62]
```

#### 4. `adding_conditions_4_gripper_urscript.script` - UR10 Gripper
**URScript Sequence:**
```python
1. Wait for "start" from Python (socket)
2. Move to approach pose
3. Open gripper: RG6(110)
4. Send "Gripper_opened_at_target_1"
5. Descend to lid
6. Close gripper: RG6(0)
7. Send "Gripper_closed_at_target_2"
8. Move to placement pose
9. Open gripper: RG6(110)
10. Send "Gripper_opened_at_target_3"
```

#### 5. `SCL_CODE.txt` - PLC State Machine
**8-Step Sorting Cycle:**
- **Step 1:** Wait for lid at sensor 2
- **Step 2:** Lid at sensor 1, start timer
- **Step 3:** Pick lid with vacuum
- **Step 4:** Start conveyor, activate stopper
- **Step 5:** **COLOR DETECTION** - Read sensors, activate appropriate sorter
- **Step 6:** Wait for tray filled signal
- **Step 7:** Return pick mechanism
- **Step 8:** Release lid, return to Step 1

---

## 🚀 Usage

### Complete System Startup

#### 1. Start MPS Stations (Remote Location)

```bash
# 1. Load PLC program in TIA Portal
# 2. Run sorting program
# 3. Start Zoom meeting and stream MPS view
```

#### 2. Setup Robotics Lab

```bash
# 1. Join Zoom meeting, display on screen
# 2. Position camera to view screen
# 3. Arrange lids on replica box matching sort order
# 4. Verify lighting conditions
```

#### 3. Launch Turtlebot 4

```bash
# Terminal 1: Turtlebot Navigation
ros2 launch turtlebot4_navigation nav2.launch.py
```

#### 4. Launch UR10 Systems

```bash
# Terminal 2: UR10 #1 Driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.1.100

# Terminal 3: UR10 #2 Driver  
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.1.101
```

#### 5. Run Control Nodes

```bash
# Terminal 4: UR10 #1 with YOLOv8
cd ~/ros2_ws/src/Siemens-Joint-Curriculum-Competition
python3 moving_robot1.py

# Follow prompt:
# Select lid tasks (e.g., red_lid=3,black_lid=2):
red_lid=1,black_lid=1

# Terminal 5: Turtlebot Coordinator
python3 moving_mobile_robot.py

# Terminal 6: UR10 #2 Shelf Controller
python3 moving_robot2_to_shelves.py
```

#### 6. Execute UR10 Gripper Script

```bash
# Load adding_conditions_4_gripper_urscript.script on UR10 #1 FlexPendant
# Update IP address: socket_open("YOUR_ROS2_PC_IP", 5000)
# Run program on FlexPendant
```

### Example Interactive Session

```bash
$ python3 moving_robot1.py

Select lid tasks (e.g., red_lid=3,black_lid=2):
red_lid=2,silver_lid=1

Selected tasks: {'red_lid': 2, 'silver_lid': 1}
Waiting for selected lid colors to be detected...

[INFO] Detected selected lid: red_lid
[INFO] Published to /lid_task: red_lid: 2
[INFO] [1/20] Published: UR10_1 is ready to start.
[INFO] [2/20] Published: UR10_1 is ready to start.
...
[INFO] UR10 connected from ('192.168.1.100', 54321)
[INFO] Received from UR10: done
[INFO] [1/25] Published: UR10_1: Finished loading the package.
...
```

---

## 📡 ROS2 Topics

### Topic Communication Map

| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|------------|---------|
| `/lid_task` | `std_msgs/String` | `moving_robot1.py` | `moving_robot2_to_shelves.py` | Lid color and quantity |
| `/robot_status` | `std_msgs/String` | `moving_robot1.py` | `moving_mobile_robot.py` | UR10 #1 status updates |
| `/turtlebot_status` | `std_msgs/String` | `moving_mobile_robot.py` | `moving_robot1.py` | Turtlebot at Position 1 |
| `/turtlebot_status_2` | `std_msgs/String` | `moving_mobile_robot.py` | `moving_robot2_to_shelves.py` | Turtlebot at Position 2 |
| `/ur10_2/urscript_interface/script_command` | `std_msgs/String` | `moving_robot2_to_shelves.py` | UR10 #2 Driver | URScript commands |

### Message Examples

```python
# /lid_task format
"red_lid: 2"
"silver_lid: 1"
"black_lid: 3"

# /robot_status messages
"UR10_1 is ready to start."
"UR10_1: Finished loading the package."

# /turtlebot_status messages
"Turtlebot reached picking place."
"Turtlebot reached delivery location."

# /ur10_2/urscript_interface/script_command
"movej([1.8, -1.57, -0.86, -0.72, 1.28, 1.62], a=1.2, v=0.25)"
```

### Monitoring Topics

```bash
# View all active topics
ros2 topic list

# Echo specific topic
ros2 topic echo /lid_task
ros2 topic echo /robot_status

# Check topic details
ros2 topic info /lid_task
ros2 topic hz /robot_status

# Publish test message
ros2 topic pub /lid_task std_msgs/String "data: 'red_lid: 1'" --once
```

---

## 🔌 PLC State Machine

### Complete SCL Logic Breakdown

```
┌──────────────────────────────────────────────────────────────┐
│                    PLC SORTING CYCLE                          │
└──────────────────────────────────────────────────────────────┘

STEP 1: INITIAL STATE
──────────────────────
Condition: Pusher_sensor_2 = TRUE
Actions:
  ├─ pushout_piece := TRUE
  ├─ Reset all timers
  ├─ Reset sorters (Sorter_1, Sorter_2 := FALSE)
  └─ Reset status LEDs (RED, GREEN, YELLOW := FALSE)
Next: Step 2

STEP 2: LID ARRIVAL
────────────────────
Condition: Pusher_sensor_1 = TRUE
Actions:
  ├─ pushout_piece := FALSE
  ├─ Release_air := FALSE
  ├─ Timer_start := 1
  └─ RED LED ON
Next: Step 3

STEP 3: PICK LID
─────────────────
Condition: Pusher_sensor_2 = TRUE AND Timer_finished
Actions:
  ├─ Pick_piece := TRUE (Activate vacuum)
  ├─ Timer2_start := 1
  └─ RED LED ON
Next: Step 4

STEP 4: START CONVEYOR
───────────────────────
Condition: Drop_piece_sensor = FALSE AND Timer2_finished
Actions:
  ├─ Conveyor := TRUE
  ├─ Stopper := TRUE
  ├─ Timer3_start := 1
  └─ YELLOW LED ON
Next: Step 5

STEP 5: COLOR DETECTION & SORTING ⭐
────────────────────────────────────
Condition A: Colour_sensor = TRUE AND Metal_sensor = FALSE
  → RED LID DETECTED
  Actions:
    ├─ Sorter_1 := TRUE (Activate RED tray)
    ├─ Stopper := FALSE
    └─ Timer4_start := 1

Condition B: Colour_sensor = TRUE AND Metal_sensor = TRUE
  → SILVER LID DETECTED
  Actions:
    ├─ Sorter_2 := TRUE (Activate SILVER tray)
    ├─ Stopper := FALSE
    └─ Timer4_start := 1

Condition C: Colour_sensor = FALSE
  → BLACK LID DETECTED
  Actions:
    ├─ Stopper := FALSE (Pass through)
    └─ Timer4_start := 1

Next: Step 6

STEP 6: WAIT FOR TRAY
──────────────────────
Condition: Filled_up = TRUE AND Timer4_finished
Actions:
  ├─ Suck_air := TRUE
  ├─ Conveyor := FALSE
  ├─ Timer5_start := 1
  └─ GREEN LED ON
Next: Step 7

STEP 7: RETURN GRIPPER
───────────────────────
Condition: Pick_piece_sensor = TRUE AND Timer4_finished
Actions:
  ├─ Pick_piece := FALSE
  ├─ Drop_piece := TRUE
  ├─ Timer6_start := 1
  └─ GREEN LED ON
Next: Step 8

STEP 8: RELEASE & RESET
────────────────────────
Condition: Drop_piece_sensor = TRUE AND Timer5_finished
Actions:
  ├─ Suck_air := FALSE
  ├─ Release_air := TRUE
  ├─ Drop_piece := FALSE
  ├─ pushout_piece := FALSE
  └─ All LEDs OFF
Next: Return to Step 1 (New cycle)