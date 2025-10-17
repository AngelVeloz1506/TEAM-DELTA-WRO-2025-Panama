# Hardware

This section covers the **electronics, components (BOM), power**, and **schematics**.

---

## Bill of Materials (BOM)

| **Component** | **Qty** | **Notes** |
|----------------|---------|-----------|
| [Raspberry Pi 5 (16 GB)](https://www.amazon.com/-/es/SC1113-Raspberry-Pi-5-16GB/dp/B0DSPYPKRG/ref=sr_1_1?sr=8-1) | 1 | Companion computer for vision & AI tasks |
| [ESP32 (DevKitC/WROOM/WROVER)](https://www.amazon.com/-/es/ESP-WROOM-32-Desarrollo-Microcontrolador-Procesador-Compatible/dp/B08D5ZD528/ref=sr_1_3) | 1 | Main MCU, 3.3 V logic |
| [Logitech C920 Webcam](https://www.amazon.com/-/es/Logitech-correcci%C3%B3n-funciona-Facetime-port%C3%A1til/dp/B085TFF7M1/ref=sr_1_1?sr=8-1) | 1 | 1080p video input for vision processing |
| [HC-SR04 Ultrasonic Sensor](https://www.amazon.com/-/es/ultras%C3%B3nico-HC-SR04-distancia-Mega2560-Conjunto/dp/B01COSN7O6/ref=sr_1_4?sr=8-4) | 3 | Front, Left, Right (Echo shifted to 3.3 V) |
| [DRV8871 H-Bridge driver](https://www.amazon.com/-/es/Teyleten-DRV8871-controlador-H-puente-m%C3%B3dulo/dp/B0DN66P9XW/ref=sr_1_3?sr=8-3) | 2 | One per DC motor |
| [HS-485HB Servo (steering)](https://www.amazon.com/-/es/HS-485HB-Est%C3%A1ndar-Anal%C3%B3gico-Karbonite-Aircraft/dp/B0F9YB8P4B/ref=sr_1_2?sr=8-2) | 1 | 5–6 V supply, high torque |
| [DC Motor 12 V (RS555 model)](https://www.ebay.com/itm/155702183252?_skw=motor+encoder+r555+12v) | 2 | RS555 brushed motor |
| [MPU6050 (I2C)](https://www.amazon.com/giroscopio-magn%C3%A9tico-aceleraci%C3%B3n-acelerador-magnet%C3%B3metro/dp/B01I1J0Z7Y/ref=sr_1_2_sspa?sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY) | 1 | IMU (gyro + accel) |
| [LiPo Battery 11.4 V 3000 mAh](https://www.ebay.com/itm/395464923168?_skw=zeee+11.4v+lipo+3s+3000mah) | 1 | XT60 recommended |
| [LM2596 / HW-083 step-down regulator](https://www.amazon.com/convertidor-reguladores-ajustables-recomendado-alimentaci%C3%B3n/dp/B0FDB25T5L/ref=sr_1_1_sspa?sr=8-1-spons) | 2+ | 12→6 V (servo) and 12→5/3.3 V (logic) |
| [Main switch](https://www.amazon.com/interruptor-basculante-encendido-precableado-autom%C3%B3vil/dp/B07S2QJKTX/ref=sr_1_2_sspa?sr=8-2-spons) + [Fuses](https://www.amazon.com/-/es/0-197x0-787-0-2x0-78-pulgadas-F1-6AL250V-Fusibles/dp/B07V288K4L/ref=sr_1_16?sr=8-16) | 1 set | Safety |

> **Tip:** Always maintain a **common GND** across battery, drivers, sensors, and MCU.

---

## Key Modules

### Raspberry Pi 5 (16 GB)

![Raspberry Pi 5](../other/photos/raspberrypi.webp)

The **Raspberry Pi 5** serves as the **companion computer** for AI and computer vision tasks.  
It communicates with the ESP32 via UART, I2C, or Ethernet for real-time data exchange.  
Capable of running OpenCV, TensorFlow, and SLAM algorithms for perception and control.

---

### ESP32 Controller (DevKitC/WROOM/WROVER)

![ESP32](../other/photos/esp32.jpg)

The **ESP32** functions as the **main microcontroller unit (MCU)**, managing motor control, sensor readings, and data exchange with the Raspberry Pi.  
Its dual-core architecture allows simultaneous task management for reliable real-time performance.

| Feature | Value |
|--|--|
| Cores | Xtensa LX6 dual-core |
| Voltage | 3.3 V |
| I/O Pins | 30+ |
| PWM (LEDC) | Yes |
| Flash | 4 MB |

---

### Camera (Logitech C920)

![Camera](../other/photos/cam.jpg)

A **1080p USB camera** used by the Raspberry Pi for lane and object detection.  
Provides clear video input under different lighting conditions for visual AI processing.

---

### Ultrasonic Sensor (HC-SR04)

![Ultrasonic Sensor](../other/photos/ultrasonic.webp)

Three ultrasonic modules are placed on the **front**, **left**, and **right** sides of the chassis.  
They measure distance to obstacles for safety and navigation.  
Each sensor uses a 30 ms timeout and operates at 5 V with logic shifted to 3.3 V.

---

### H-Bridge (DRV8871)

![Motor Driver](../other/photos/driver.jpg)

The **DRV8871 motor driver** is responsible for controlling the 12 V DC motors using PWM signals.  
Each module drives one motor, supporting bidirectional rotation and current limiting for protection.

- **VM** → battery (8–10 V)  
- **OUT1/OUT2** → motor terminals  
- **IN1/IN2** → ESP32 GPIOs  
- **GND** → shared ground

---

### Servo (HS-485HB)

![Servo](../other/photos/servo.webp)

The **HS-485HB** servo motor handles steering via **Ackermann geometry**.  
It operates independently at 5–6 V (≥2 A), ensuring responsive turns without overloading logic power.  
Durable Karbonite gears make it suitable for continuous operation.

---

### DC Motor (RS555 Model)

![DC Motor](../other/photos/motor.webp)

Two **12 V brushed DC motors** drive the vehicle’s rear wheels.  
Each is connected to a DRV8871 H-Bridge for speed and direction control.  
These motors deliver strong torque and consistent speed under load.

---

### IMU (MPU6050)

![IMU](../other/photos/IMU.jpg)

The **MPU6050** combines a **3-axis accelerometer** and **3-axis gyroscope**.  
It provides feedback for yaw correction and helps maintain trajectory stability.  
Data is transmitted via I2C to the ESP32 for sensor fusion.

---

### Battery (LiPo 11.4 V 3000 mAh)

![Battery](../other/photos/battery.webp)

A **3-cell LiPo** powers all components.  
It provides 11.4 V nominal voltage with high current capacity, connected through an XT60 plug.  
Protected with inline fuses for safety.

---

### Step-Down Regulator (LM2596 / HW-083)

![Buck Converter](../other/photos/buck-converter.jpg)

The **LM2596/HW-083** buck converters reduce the main supply voltage from 12 V down to  
6 V for the servo and 5/3.3 V for logic systems.  
They include output adjustment and thermal protection features.

---

## Schematics

- [Delta Schematic Design (PDF)](../schemes/Delta%20schematic%20design.pdf)

**Preview**

![Circuit Diagram](../schemes/Delta%20schematic%20design.jpg)

This diagram shows the ESP32, DRV8871 drivers, Raspberry Pi, C920 camera, sensors, servo, regulators, and power distribution.

---
