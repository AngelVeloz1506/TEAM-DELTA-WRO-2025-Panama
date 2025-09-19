# TEAM DELTA — WRO 2025 Panama

> Dominican Republic • Future Engineers • 2025

---

## Index

* 1. [Introduction](#introduction)
* 2. [Components](#components)
  * [Materials List](#materials-list)
  * [ESP32 Controller](#esp32-controller)
  * [Servo Motor (HS-485HB)](#servo-motor-hs-485hb)
  * [H-Bridge (DRV8871)](#h-bridge-drv8871)
  * [Pixy2 Camera](#pixy2-camera)
  * [Ultrasonic Sensors (HC-SR04)](#ultrasonic-sensors-hc-sr04)
  * [Inertial Unit (MPU6050)](#inertial-unit-mpu6050)
  * [Battery Pack](#battery-pack)
  * [Motors](#motor)
  * [Voltage Regulators (HW-083, LM2596)](#voltage-regulators-hw-083-lm2596)
* 3. [Robot Floors](#robot-floors)
  * [First Floor](#first-floor)
  * [Front Axle](#front-axle)
  * [Rear Axle](#rear-axle)
  * [Second Floor](#second-floor)
  * [Third Floor](#third-floor)
  * [Fourth Floor](#fourth-floor)
* 4. [3D Design](#3d-design)
  * [Final Robot](#final-robot)
  * [Prototype Versions](#prototype-versions)
  * [Final STL Files](#final-stl-files)
* 5. [Circuit Diagram](#circuit-diagram)
* 6. [Videos](#videos)

---

## Introduction

**TEAM DELTA Δ** represents the Dominican Republic in the **World Robot Olympiad 2025 – Panama**, *Future Engineers* category. This repository documents the design, electronics, code, and iterations of our autonomous vehicle.

The project uses an **ESP32** as the main controller, **Pixy2** for color/shape detection, **three ultrasonic sensors** for distance measurement (front/left/right), **DRV8871** drivers for DC motors, and an **HS-485HB servo** for steering. Power distribution relies on **LM2596/HW-083** regulators.

---

## Components

### Materials List

| **Component**                | **Qty** | **Notes**                                   |
| ---------------------------- | ------- | ------------------------------------------- |
| ESP32 (DevKitC/WROOM/WROVER) | 1       | Main MCU, 3.3 V logic                       |
| Pixy2 Camera                 | 1       | Color/line/object tracking                  |
| HC-SR04 Ultrasonic Sensor    | 3       | Front, Left, Right (Echo shifted to 3.3 V)  |
| DRV8871 H-Bridge driver      | 2       | One per DC motor                            |
| HS-485HB Servo (steering)    | 1       | 5–6 V supply, high torque                   |
| DC Motor 12 V                | 2       | RS555                                       |
| MPU6050 (I2C)                | 1       | IMU (gyro + accel)                          |
| LiPo Battery 11.4 V 3000 mAh | 1       | XT60 recommended                            |
| LM2596 / HW-083 step-down    | 2+      | 12 V→6 V (servo) and 12 V→5 V/3.3 V (logic) |
| Main switch + fuses          | 1 set   | Safety                                      |

> **Tip:** Always maintain a **common GND** between the battery, drivers, sensors, and ESP32.

---

### ESP32 Controller

| Feature           | Value                          |
| ----------------- | ------------------------------ |
| Microcontroller   | Tensilica Xtensa LX6 dual-core |
| Operating Voltage | 3.3 V                          |
| Digital I/O Pins  | 30+                            |
| PWM (LEDC)        | Yes                            |
| Flash Memory      | 4 MB                           |

---

### Servo Motor HS-485HB

Used for steering (Ackermann mechanism). It is recommended to use an **independent 5–6 V (≥2 A) supply** and wiring with **common GND**.

**Library:** `<ESP32Servo.h>`

---

### H-Bridge (DRV8871)

* **VM** → battery (8–12 V typical).  
* **OUT1/OUT2** → motor.  
* **IN1 / IN2** → ESP32 GPIOs.  
* **GND** → common ground.  

> If you want PWM in both directions, use `ledcAttach` on `IN2` as well and alternate which pin carries PWM depending on direction.

---

### Pixy2 Camera

Used for object/color detection and line following. Connect via SPI/I2C/UART depending on your board. See [src/](src/) for reading examples with `pixy.ccc.getBlocks()`.

* Power supply: stable 5 V.  
* LED controllable for feedback (ready blink).  

---

### Ultrasonic Sensors (HC-SR04)

Three sensors: **front, left, right**.  

* **Trig** (output) to GPIO.  
* **Echo** (input) to GPIO.  
* 30 ms timeout for robust readings.  

---

### Inertial Unit (MPU6050)

* Connection via I2C (SDA/SCL).  
* Useful for heading/curvature drift compensation.  

---

### Battery Pack

* **LiPo 3S (11.4 V, 3000 mAh)** with **XT60 connector**.  
* Fuse/Breaker recommended.  
* Never discharge below 3.3 V/cell.  

---

### Motor

* **12 V DC** (e.g. RS555). Adjust **max duty cycle** and **ramp-up** to avoid spikes.  

---

### Voltage Regulators (HW-083, LM2596)

* **LM2596/HW-083** → 6 V for servo; another to 5 V/3.3 V for logic.    

---

## Robot Floors

### First Floor

Support for motors, steering servo, ultrasonic sensors, DRV8871 drivers, and basic electronics.

#### Front Axle

* Steering mechanism (Ackermann).  
* Reinforced servo horn.  

#### Rear Axle

* Direct motor-to-wheel transmission or gearbox.  
* Differential system included to balance wheel speeds during turns.  

### Second Floor

* ESP32, regulators, Pixy2 front-mounted, battery pack mounting, main switch,camera support and wiring distribution.  

### Third Floor

* Reserved space for future expansions.  
---

## 3D Design
> CAD and STL files available in [models/](models/).

The [`models/`](models/) folder contains all CAD assets for the vehicle chassis and accessories.  
It is divided into two categories:

### Final Robot

- **[`RSLN118/`](models/RSLN118/)**  
  This is the **final version** of the robot, used for the official competition build.  
  Inside you will find:  
  - [`STL/`](models/RSLN118/STL/) — Final printable meshes for slicing.  
  - `STEP/` — Editable CAD files for modifications.  
  - `Diff01/`, `Diff02/` — Differential system components.  
  - `Hitec HS-485HB/` — Servo integration parts.  
  - `Test/` — Auxiliary sub-assemblies and test parts.  
  - Individual SolidWorks parts (`Base`, `CenterShaft`, `ESP32 Shield`, etc.) used in the assembly.  

This folder represents the **reference design** for manufacturing and testing.  

---

### Prototype Versions

The following folders contain **experimental iterations** of the robot developed before reaching the final version.  
Each version shows incremental improvements in **mechanical structure, steering system, motor integration, and sensor mounting**.  

- [`V1/`](models/V1/) – First prototype, basic chassis and motor mounts.  
- [`V2/`](models/V2/) – Added steering system and wheel redesign.  
- [`V3/`](models/V3/) – Structural reinforcement and servo integration.  
- [`V3.1/`](models/V3.1/) – Adjustments to motor holders and pulleys.  
- [`V3.2/`](models/V3.2/) – Refinement of wheel and servo arm coupling.  
- [`V4/`](models/V4/) – Optimized shaft and motor holder stability.  
- [`V5/`](models/V5/) – Integration of DRV8871 driver mounts and battery positioning.  
- [`V6/`](models/V6/) – Cable management, full camera and battery assembly.  
- [`V6.5/`](models/V6.5/) – Pre-final candidate with IMU case and improved drivetrain.  

> These folders are preserved for **traceability** and design history, but the official reference design is [`RSLN118`](models/RSLN118/).  

---

### Final STL Files

The final 3D printable parts for the competition robot are stored in:

- [RSLN118 / STL](models/RSLN118/STL/)  

This folder contains the **ready-to-print meshes** of the final chassis, servo mounts, differential components, and sensor holders.  

> Use these STLs for slicing and manufacturing. Earlier versions are only kept for documentation and design history.  

---

## Circuit Diagram

The complete schematic of the robot’s electronic system is available here:

[Delta schematic design (PDF)](schemes/Delta%20schematic%20design.pdf)

### Preview  

![Circuit Diagram Preview](schemes/Delta%20schematic%20design.jpeg)

This diagram illustrates the wiring of the ESP32, DRV8871 drivers, sensors, servo motor, regulators, and power distribution.  

---

## Videos

The [`videos/`](videos/) folder contains multimedia resources documenting the project.  

Inside, you will find a [`README.md`](videos/README.md) with:

- Direct links to performance and testing videos.  
- Documentation of open/closed challenges.  
- Link to the official **YouTube channel** for more updates:  
  [TEAM DELTA – YouTube Channel](https://www.youtube.com/channel/UCRmfdBhCKCmFW21Ekp9HE9g)  
