# Autonomous C-sUAS "Slew-to-Cue" Tracking Turret (TRL 4 Prototype)

An autonomous, low-cost ($245) C-sUAS Point-Defense tracking and ranging gimbal utilizing YOLO object detection, KLT optical flow, and closed-loop velocity-PD control.

This repository houses the source code, hardware architecture, and firmware for a low-latency Point-Defense target acquisition platform capable of processing real-time telemetry, sensor fusion, and high-accuracy servo actuation.

## 🎥 System Demonstration & HUD

*(Insert a high-quality GIF of your turret physically tracking a target side-by-side with your MATLAB/Python HUD tracking feed here)*

> **Live Demo Video:** https://youtube.com/shorts/hJJrJ_sbNVY?si=u6pjSPRpaa84ya5z

## 🎯 Core Engineering Features

* **Low-Latency Vision-to-Actuation Pipeline:** Employs a dual-routing tracking pipeline. The system runs deep learning object detection to acquire targets, then hands off tracking to a high-speed KLT (Kanade-Lucas-Tomasi) Optical Flow Tracker.
* **Low-Overhead Custom Binary Protocol:** Replaced slow ASCII string serial parsing on the microcontroller with a custom 5-byte raw binary telemetry protocol over USB/UART, reducing serial processing overhead on the Arduino R4 from ~3ms to <20 microseconds.
* **Velocity-PD Control Loop:** Implements a localized, dynamically tuned Velocity-Proportional-Derivative (PD) feedback loop to eliminate physical servo overshoot and mechanical backlash, stabilizing the tracking frame to within +/- 3 pixels of target centers.
* **Resolution-Independent Scaling:** Features a spatial transformation matrix mapping coordinate errors to a normalized reference grid (640 x 360) to ensure uniform controller gains regardless of sensor upgrades (including 4K feeds).
* **Sensor Fusion & Telemetry:** Integrates a TF-Luna LiDAR sensor over TTL serial, writing custom parsing routines to fuse absolute ranging data with real-time visual coordinate estimation.

## 🛠️ System Architecture

```text
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
   
Create and activate a virtual environment:Bash   python -m venv venv
   .\venv\Scripts\activate
   
Install the required computer vision and robotics packages:Bash   pip install pyserial opencv-python ultralytics numpy
   
Firmware (Arduino R4 Setup)Connect your Arduino Uno R4 Minima to your PC.Open firmware/turret_actuator_binary.ino in the Arduino IDE.Install the Adafruit PWM Servo Driver Library via the Library Manager.Upload the code to your Arduino.Check Device Manager to verify your Arduino's assigned COM port.💻 Running the SystemUpdate the COM_PORT and CAMERA_INDEX variables in src/turret_master.py to match your PC's environment.Run the master tracking script:Bash   python src/turret_master.py
   
Focus your camera on a target. Press q in the window to stop tracking and park the servos safely at their neutral coordinates (375, 375).📁 Repository StructurePlaintextautonomous-target-tracking-turret/
│
├── firmware/
│   └── turret_actuator_binary.ino      # Arduino R4 firmware (reads binary payloads)
│
├── src/
│   ├── turret_master.py         # Production Python tracking controller (YOLO)
│   ├── train_model.py                  # AI training execution script
│   └── find_camera.py                  # Windows camera index scanner utility
│
├── docs/
│   ├── schematics_wiring.png           # Fritzing/CAD wiring diagram
│   └── system_block_diagram.png        # System-of-systems flow diagram
│
├── README.md                           # Main developer documentation
└── LICENSE                             # Proprietary License Agreement
🔒 Intellectual Property & Proprietary LicensingThis repository and its contents are the proprietary intellectual property of Willis Aero Tech LLC. All rights are reserved. This source code and structural design are published strictly for educational, portfolio evaluation, and academic demonstration purposes. Commercial or military reproduction, modification, distribution, or deployment of this software or hardware design is strictly prohibited without explicit, written licensing authorization from Willis Aero Tech LLC
