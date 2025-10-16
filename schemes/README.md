# Hardware

This section covers the **electronics, components (BOM), power**, and **schematics**.

---

## Bill of Materials (BOM)

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
| [LM2596 / HW-083 step-down regulator](https://www.amazon.com/convertidor-reguladores-ajustables-recomendado-alimentaci%C3%B3n/dp/B0FDB25T5L/ref=sr_1_1_sspa?sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) | 2+ | 12→6 V (servo) and 12→5/3.3 V (logic) |
| [Main switch](https://www.amazon.com/interruptor-basculante-encendido-precableado-autom%C3%B3vil/dp/B07S2QJKTX/ref=sr_1_2_sspa?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1) + [Fuses](https://www.amazon.com/-/es/0-197x0-787-0-2x0-78-pulgadas-F1-6AL250V-Fusibles/dp/B07V288K4L/ref=sr_1_16?__mk_es_US=%C3%85M%C3%85%C5%BD%C3%95%C3%91&sr=8-16) | 1 set | Safety |

> **Tip:** Always maintain a **common GND** across battery, drivers, sensors, and MCU.

---

## Key Modules

### Raspberry Pi 5 (16 GB)
Companion computer for **vision/AI**. Connects to ESP32 via **UART/I2C/SPI/Ethernet**.

- CPU: Quad-core 64-bit Arm Cortex-A76 @ 2.4 GHz  
- GPU: VideoCore VII (OpenGL ES 3.1, Vulkan 1.2)  
- RAM: 16 GB LPDDR4X  
- I/O: USB 3.0, PCIe, GbE, HDMI  
- OS: Raspberry Pi OS / Ubuntu Server  
- Use: OpenCV, SLAM, AI inference

---

### ESP32 Controller (DevKitC/WROOM/WROVER)

| Feature | Value |
|--|--|
| Cores | Xtensa LX6 dual-core |
| Voltage | 3.3 V |
| I/O Pins | 30+ |
| PWM (LEDC) | Yes |
| Flash | 4 MB |

---

### Servo (HS-485HB)
Used for steering (Ackermann).  
Use **independent 5–6 V ≥2 A** power, with common GND.  
**Library:** `ESP32Servo.h`

---

### H-Bridge (DRV8871)
- **VM** → battery (8–10 V)  
- **OUT1/OUT2** → motor  
- **IN1/IN2** → ESP32 GPIOs  
- **GND** → common ground  

> For bidirectional PWM, drive both inputs with PWM selectively.

---

### Sensors
- **Ultrasonic (HC-SR04):** 3 units (front/left/right), 30 ms timeout.  
- **MPU6050 (I2C):** heading drift compensation and smoothing.

---

### Power
- **LiPo 3S 11.4 V 3000 mAh**, XT60, fuse/breaker.  
- **LM2596/HW-083:** 12→6 V (servo), 12→5/3.3 V (logic).  
- Never discharge below **3.3 V per cell**.

---

## Schematics

- [Delta Schematic Design (PDF)](schemes/Delta%20schematic%20design.pdf)

**Preview**

![Circuit Diagram](schemes/Delta%20schematic%20design.jpg)

This diagram shows ESP32, DRV8871 drivers, Raspberry Pi, C920 camera, sensors, servo, regulators, and power distribution.

---
