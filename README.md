Autonomous C-sUAS "Slew-to-Cue" Tracking Turret (TRL 4 Prototype)
This repository houses the source code, hardware architecture, and firmware for an Autonomous, Low-Cost Counter-sUAS (C-sUAS) "Slew-to-Cue" Tracking and Ranging Gimbal. Developed under a strict $245 Commercial Off-The-Shelf (COTS) budget, this system serves as a low-latency Point-Defense target acquisition platform capable of processing real-time telemetry, sensor fusion, and high-accuracy servo actuation.
🎥 System Demonstration & HUD
(Insert a high-quality GIF of your turret physically tracking a target side-by-side with your MATLAB/Python HUD tracking feed here)
Live Demo Video: Click here to watch the 60-Second Technical Walkthrough & PID Control Demonstration
🎯 Core Engineering Features
Low-Latency Vision-to-Actuation Pipeline: Employs a dual-routing tracking pipeline. The system runs deep learning object detection (YOLOv4-Tiny/YOLO26) to acquire targets, then hands off tracking to a high-speed KLT (Kanade-Lucas-Tomasi) Optical Flow Tracker running at up to 60 FPS.
Low-Overhead Custom Binary Protocol: Replaced slow ASCII string serial parsing on the microcontroller with a custom 5-byte raw binary telemetry protocol over USB/UART, reducing serial processing overhead on the Arduino R4 from  to .
Velocity-PD Control Loop: Implements a localized, dynamically tuned Velocity-Proportional-Derivative (PD) feedback loop to eliminate physical servo overshoot and mechanical backlash, stabilizing the tracking frame to within  pixels of target centers.
Resolution-Independent Scaling: Features a spatial transformation matrix mapping coordinate errors to a normalized reference grid () to ensure uniform controller gains regardless of sensor upgrades (including 4K feeds).
Sensor Fusion & Telemetry: Integrates a TF-Luna LiDAR sensor over TTL serial, writing custom parsing routines to fuse absolute ranging data with real-time visual coordinate estimation.
🛠️ System Architecture
                       +-----------------------------+
                       |   Targeting / Command PC    |
                       | (MATLAB / Python Engine)    |
                       +--------------+--------------+
                                      |
                                      | 5-Byte Raw Binary Packet
                                      | (115200 Baud, USB-Serial)
                                      v
                       +--------------+--------------+
                       |      Arduino R4 Minima      |
                       +--------------+--------------+
                                      |
                        I2C Bus       | (50 Hz PWM Signals)
                        (0x40)        v
+------------------------+     +------+------+     +-------------------------+
|   PCA9685 PWM Driver   +---->|  Pan Servo  |     |      TF-Luna LiDAR      |
|  (Ext. 5V Power Rails) |     +-------------+     | (Direct Target Ranging) |
+------------------------+     +-------------+     +------------+------------+
                               |  Tilt Servo |                  |
                               +-------------+                  | TTL Serial
                                                                v
                                                       (To Command PC/Arduino)



📋 Hardware Bill of Materials (BOM)
Component
Description
Qty
Cost (COTS)
Insta360 Link 2C
4K High-Resolution Visual Sensor
1
$150.00
TF-Luna LiDAR
Single-Point Optical Ranging Module
1
$30.00
Arduino UNO R4 Minima
Microcontroller / Actuator Relay
1
$25.00
Dual Servos
Pan/Tilt Mechanical Actuators
2
$20.00
PCA9685 Driver
16-Channel PWM Servo Controller
1
$15.00
Misc. Hardware
Breadboard, jumper wires, customized mount plate
-
$5.00
TOTAL
Fully Functional Sensor Node & Actuator Gimbal


$245.00

🚀 Software Installation & Setup
Command PC (Python Environment)
Clone this repository:
git clone https://github.com/YOUR_USERNAME/your-repo-name.git
cd your-repo-name



Create and activate a virtual environment:
python -m venv venv
.\venv\Scripts\activate



Install the required computer vision and robotics packages:
pip install pyserial opencv-python ultralytics numpy



Firmware (Arduino R4 Setup)
Connect your Arduino Uno R4 Minima to your PC.
Open firmware/turret_actuator_binary.ino in the Arduino IDE.
Install the Adafruit PWM Servo Driver Library via the Library Manager.
Upload the code to your Arduino.
Check Device Manager to verify your Arduino's assigned COM port (e.g., COM4).
💻 Running the System
Update the COM_PORT and CAMERA_INDEX variables in src/turret_master_modern.py to match your PC's environment.
Run the master tracking script:
python src/turret_master_modern.py



Focus your camera on a target. Press q in the window to stop tracking and park the servos safely at their neutral coordinates ().
📁 Repository Structure
your-repo-name/
│
├── firmware/
│   └── turret_actuator_binary.ino      # Arduino R4 firmware (reads binary payloads)
│
├── src/
│   ├── turret_master_modern.py         # Production Python tracking controller (YOLO)
│   ├── turret_master_optimized.m       # Optimized MATLAB tracking controller (KLT)
│   ├── train_model.py                  # AI training execution script
│   └── find_camera.py                  # Windows camera index scanner utility
│
├── docs/
│   ├── schematics_wiring.png           # Fritzing/CAD wiring diagram
│   └── system_block_diagram.png        # System-of-systems flow diagram
│
├── README.md                           # Main developer documentation
└── LICENSE                             # MIT License



Standard Copyright (All Rights Reserved).

