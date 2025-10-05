# TEAM DELTA — WRO 2025 Panama

> Dominican Republic • Future Engineers • 2025  

---

## Index

1. [Introduction](#introduction)  
2. [The Team](#the-team)  
3. [Components](#components)  
   - [Materials List](#materials-list)  
   - [Raspberry Pi 5 (16 GB)](#raspberry-pi-5-16-gb)  
   - [ESP32 Controller](#esp32-controller)  
   - [Servo Motor (HS-485HB)](#servo-motor-hs-485hb)  
   - [H-Bridge (DRV8871)](#h-bridge-drv8871)  
   - [Logitech C920 Webcam](#logitech-c920-webcam)  
   - [Ultrasonic Sensors (HC-SR04)](#ultrasonic-sensors-hc-sr04)  
   - [Inertial Unit (MPU6050)](#inertial-unit-mpu6050)  
   - [Battery Pack](#battery-pack)  
   - [Motors](#motor)  
   - [Voltage Regulators (HW-083, LM2596)](#voltage-regulators-hw-083-lm2596)  
4. [Robot Floors](#robot-floors)  
   - [First Floor](#first-floor)  
     - [Front Axle](#front-axle)  
     - [Rear Axle](#rear-axle)  
   - [Second Floor](#second-floor)  
   - [Third Floor](#third-floor)  
5. [3D Design](#3d-design)  
   - [Final Robot](#final-robot)  
   - [Prototype Versions](#prototype-versions)  
   - [Final STL Files](#final-stl-files)  
6. [Circuit Diagram](#circuit-diagram)  
7. [Videos](#videos)  

---

## Introduction

**TEAM DELTA Δ** proudly represents the **Dominican Republic** in the **World Robot Olympiad 2025 – Panama**, competing in the *Future Engineers* category.  

This repository documents the development of our autonomous vehicle: its **design process, 3D models, electronic circuits, coding, and improvements**.  

We are committed to combining **engineering, innovation, and teamwork** to achieve outstanding results in the 2025 season.  

---

## The Team

**Team Members — Pontificia Universidad Madre y Maestra (PUCMM):**  
- Ángel Veloz – angelvelozsan1506@gmail.com  
- Luis Lockward – luislockward@gmail.com  
- Pedro Pérez – pedrojosepw@gmail.com  

**Coach — Pontificia Universidad Madre y Maestra (PUCMM):**  
- Álvaro Zapata  

---

## Components

### Materials List

| **Component** | **Qty** | **Notes** |
|----------------|---------|-----------|
| [Raspberry Pi 5 (16 GB)](https://www.amazon.com/-/es/SC1113-Raspberry-Pi-5-16GB/dp/B0DSPYPKRG/ref=sr_1_1?sr=8-1) | 1 | Companion computer for vision & AI tasks |
| [ESP32 (DevKitC/WROOM/WROVER)](https://www.amazon.com/-/es/ESP-WROOM-32-Desarrollo-Microcontrolador-Procesador-Compatible/dp/B08D5ZD528/ref=sr_1_3?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&sr=8-3) | 1 | Main MCU, 3.3 V logic |
| [Logitech C920 Webcam](https://www.amazon.com/-/es/Logitech-correcci%C3%B3n-funciona-Facetime-port%C3%A1til/dp/B085TFF7M1/ref=sr_1_1?sr=8-1) | 1 | 1080p video input for vision processing |
| [HC-SR04 Ultrasonic Sensor](https://www.amazon.com/-/es/ultras%C3%B3nico-HC-SR04-distancia-Mega2560-Conjunto/dp/B01COSN7O6/ref=sr_1_4?sr=8-4) | 3 | Front, Left, Right (Echo shifted to 3.3 V) |
| [DRV8871 H-Bridge driver](https://www.amazon.com/-/es/Teyleten-DRV8871-controlador-H-puente-m%C3%B3dulo/dp/B0DN66P9XW/ref=sr_1_3?sr=8-3) | 2 | One per DC motor |
| [HS-485HB Servo (steering)](https://www.amazon.com/-/es/HS-485HB-Est%C3%A1ndar-Anal%C3%B3gico-Karbonite-Aircraft/dp/B0F9YB8P4B/ref=sr_1_2?sr=8-2) | 1 | 5–6 V supply, high torque |
| [DC Motor 12 V (RS555 model)](https://www.ebay.com/itm/155702183252?_skw=motor+encoder+r555+12v&itmmeta=01K6RY97TX6YGF4EXKX1KNRKAF&hash=item244092c554:g:QAwAAOSwpLtjfHUy&itmprp=enc%3AAQAKAAAA8FkggFvd1GGDu0w3yXCmi1fqIZxqosoijrDaOrwyAHcYrllo%2FcNOmzSE%2B1NxJZfS8PUanF9iX8dDxBkjlbS2Hkb27mddZSHPee%2FgFR6c9CxGe4TfqOkhUpE7DMBluxycmN6IKl3R6kBY1iSzasijBgqWidvmlMoNvcSH2Cwh%2BC9AnmxDQEGf3FZyakoEzWmzWriy%2BK4zawXNBsU0NO2lFguH6j4Al5KHVw%2F2ZCz3tFsEjwxsrrDr1lPk57cA8i%2BevPLG2qnBjuXWdmMjToBwW%2FABGWQBlAiZl7bbnkawSAf69qrRgMSU1XdwQPBGp5onQQ%3D%3D%7Ctkp%3ABFBM0P2knrZm) | 2 | RS555 brushed motor |
| [MPU6050 (I2C)](https://www.amazon.com/-/es/HiLetgo-MPU-6050-Aceler%C3%B3metro-Giroscopio-Convertidor/dp/B00LP25V1A/ref=sr_1_3?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&sr=8-3) | 1 | IMU (gyro + accel) |
| [LiPo Battery 11.4 V 3000 mAh](https://www.ebay.com/itm/395464923168?_skw=zeee+11.4v+lipo+3s+3000mah&itmmeta=01K6RYC9AE0J08QV6FE0ZHYT6V&hash=item5c138bd820:g:cMkAAOSwoThmaVtT&itmprp=enc%3AAQAKAAAA8FkggFvd1GGDu0w3yXCmi1eWSGn%2BjHvw1YOMovL0XlVuPBeq4pvfr%2FvW%2FVnc8Fz4loUFViFk8xhiaRN74MQpKfxRZw4PC%2BRE3C1ZXs4HIRCAiEnstTL8VMC12j85UxgxOP5H5tR3ogAwVwBeHceOh%2Bw5t5SeJrAgz%2Fjr8m4g6fe48iszTwfsIVvxYDcmC1Z0dpfk0EmdHuKaD1IEq7%2B5Ho1S1uhtvucAsTyfDl9jvQWQ%2FJCJ11FEBzGzCMFuBgWs3ocyuxV6avQ9z1awj7YMGKsIHLtUnsoLzfkqkXtKFw%2B04%2FKF5B0aclPbsImTEtLxXw%3D%3D%7Ctkp%3ABFBMqJWxnrZm) | 1 | XT60 recommended |
| [LM2596 / HW-083 step-down regulator](https://www.amazon.com/convertidor-reguladores-ajustables-recomendado-alimentaci%C3%B3n/dp/B0FDB25T5L/ref=sr_1_1_sspa?sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) | 2+ | 12 V→6 V (servo) and 12 V→5 V/3.3 V (logic) |
| [Main switch](https://www.amazon.com/interruptor-basculante-encendido-precableado-autom%C3%B3vil/dp/B07S2QJKTX/ref=sr_1_2_sspa?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) + [Fuses](https://www.amazon.com/-/es/0-197x0-787-0-2x0-78-pulgadas-F1-6AL250V-Fusibles/dp/B07V288K4L/ref=sr_1_16?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&sr=8-16) | 1 set | Safety |

> **Tip:** Always maintain a **common GND** between the battery, drivers, sensors, and ESP32.

---

### Raspberry Pi 5 (16 GB)

The **Raspberry Pi 5 (16 GB)** serves as the **companion computer** for advanced vision processing, AI inference, and high-level navigation.  
It integrates seamlessly with the ESP32 over **UART/I2C/SPI/Ethernet** for sensor fusion and decision-making.

* **CPU:** Quad-core 64-bit Arm Cortex-A76 @ 2.4 GHz  
* **GPU:** VideoCore VII (supports OpenGL ES 3.1, Vulkan 1.2)  
* **RAM:** 16 GB LPDDR4X  
* **Connectivity:** USB 3.0, PCIe, Gigabit Ethernet, HDMI  
* **Operating System:** Raspberry Pi OS / Ubuntu Server  
* **Applications:** OpenCV vision processing, SLAM, AI model inference  

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

Used for steering (Ackermann mechanism).  
It is recommended to use an **independent 5–6 V (≥2 A) supply** and wiring with **common GND**.

**Library:** `<ESP32Servo.h>`

---

### H-Bridge (DRV8871)

* **VM** → battery (8–10 V).  
* **OUT1/OUT2** → motor.  
* **IN1/IN2** → ESP32 GPIOs.  
* **GND** → common ground.  

> If you want PWM in both directions, use `ledcAttach` on `IN2` as well and alternate which pin carries PWM depending on direction.

---

### Logitech C920 Webcam

Used for **real-time vision processing** such as lane detection, obstacle recognition, and video streaming.  
The C920 connects to the **Raspberry Pi 5** via **USB**, where images are processed using **OpenCV** or AI models.

* **Resolution:** up to 1080p (30 FPS)  
* **Interface:** USB 2.0 / 3.0  
* **Wide field of view**, suitable for road detection  
* **Power:** Stable 5 V supply through USB  

---

### Ultrasonic Sensors (HC-SR04)

Three sensors: **front, left, right**.  

* **Trig** (output) → GPIO  
* **Echo** (input) → GPIO  
* 30 ms timeout for robust readings  

---

### Inertial Unit (MPU6050)

* Connection via I2C (SDA/SCL)  
* Useful for heading / curvature drift compensation  

---

### Battery Pack

* **LiPo 3S (11.4 V, 3000 mAh)** with **XT60 connector**  
* Fuse / Breaker recommended  
* Never discharge below 3.3 V per cell  

---

### Motor

* **12 V DC** (e.g. RS555)  
* Adjust **max duty cycle** and **ramp-up** to avoid spikes  

---

### Voltage Regulators (HW-083, LM2596)

* **LM2596/HW-083** → 6 V for servo; another to 5 V / 3.3 V for logic  

---

## Robot Structure

The robot’s mechanical structure is designed in multiple stacked floors, each serving a distinct function and housing specific components.

### First Floor

Support for motors, steering servo, ultrasonic sensors, DRV8871 drivers, and basic electronics.

#### Front Axle

* Steering mechanism (Ackermann)  
* Reinforced servo horn  

#### Rear Axle

* Direct motor-to-wheel transmission or gearbox  
* Differential system to balance wheel speeds during turns  

### Second Floor

* ESP32, regulators, Logitech C920 front-mounted, battery pack, main switch, camera support and wiring distribution  

### Third Floor

* Reserved space for future expansions  

---

## 3D Design
> CAD and STL files available in [models/](models/)

The [`models/`](models/) folder contains all CAD assets for the vehicle chassis and accessories.  
It is divided into two categories:

### Final Robot

- **[`RSLN118/`](models/RSLN118/)**  
  This is the **final version** of the robot used for the official competition build.  
  Inside you’ll find:  
  - [`STL/`](models/RSLN118/STL/) — final printable meshes  
  - `STEP/` — editable CAD files  
  - `Test/` — auxiliary sub-assemblies  
  - individual SolidWorks parts (`Base`, `CenterShaft`, `ESP32 Shield`, etc.)  

This folder represents the **reference design** for manufacturing and testing.  

---

### Prototype Versions

Folders [`V1/`](models/V1/) through [`V6.5/`](models/V6.5/) contain experimental iterations of the robot developed before the final version.  
Each shows improvements in structure, steering, motor integration, and sensor mounting.

> These are kept for traceability, but the official reference is [`RSLN118`](models/RSLN118/).  

---

### Final STL Files

The final 3D printable parts are stored in [`models/RSLN118/STL/`](models/RSLN118/STL/).  
These contain the ready-to-print meshes for the chassis, servo mounts, differential components, and sensor holders.  

> Use these STLs for manufacturing. Earlier versions are kept for documentation.  

---

## Circuit Diagram

The complete schematic of the robot’s electronics is available here:  
[Delta Schematic Design (PDF)](schemes/Delta%20schematic%20design.pdf)

### Preview  

![Circuit Diagram Preview](schemes/Delta%20schematic%20design.jpg)

This diagram illustrates the wiring of the ESP32, DRV8871 drivers, Raspberry Pi, C920 camera, sensors, servo motor, regulators, and power distribution.  

---

## Videos

The [`videos/`](videos/) folder contains multimedia resources documenting the project.  

Inside, you’ll find a [`README.md`](videos/README.md) with:  

- Direct links to performance and testing videos  
- Documentation of open / closed challenges  
- Link to the official **YouTube channel** for updates:  
  [TEAM DELTA – YouTube Channel](https://www.youtube.com/channel/UCRmfdBhCKCmFW21Ekp9HE9g)
