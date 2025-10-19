# Open Challenge — Code Evolution

This folder documents the **progressive versions** of Team Delta’s control code for the  
**WRO 2025 Future Engineers — Open Challenge**.  

Each version builds upon the previous one, introducing new sensors, control strategies, and optimizations for autonomous driving stability, accuracy, and response.

---

## **Version 1 — Basic Motion Control (Reverse-Kick and Power Boost)**

**File:** `open-v1.ino`  
**Main Components:** ESP32 + Servo (steering) + DRV8871 motor driver  

This first version establishes the **fundamental movement routine** for the robot — combining PWM motor control and steering using a servo.

### **Core Features**
- Modular helper functions for acceleration, braking, and turning.  
- Implements **reverse-kick braking** in every turn (short reverse pulses).  
- Adds a **power boost** after each braking event for smooth reacceleration.  
- Performs **three sequential right turns**, with duty cycle increase after the first turn.  
- On the **final turn**, the robot performs a full stop sequence:  
  - Reverse + brake + coast sequence for a complete halt.  

### **Code Breakdown**
- **Pin and PWM setup:**  
  Defines two motor pins (`IN_PWM`, `IN_OTHER`) controlling the DRV8871 H-bridge.  
  The PWM signal on `IN_PWM` drives forward speed, while `IN_OTHER` can toggle direction.  
  PWM is configured with `ledcAttach(channel, freq, res)` and commanded with `ledcWrite(channel, duty)`.
- **Motor control helpers:**  
  - `setForwardDuty(duty)` sets normal forward torque.  
  - `reverseKick(duty, ms)` momentarily switches PWM to the opposite pin to create a **short reverse pulse**, effectively braking the motor.  
  - `brakeHard()` energizes both inputs for strong electrical braking.  
  - `coast()` releases both pins for a free-spin stop.
- **Servo motion:**  
  The servo turns fully right for a corner (`turnRightMaxPower()`) and returns to center (`returnCenterMaxPower()`), both under controlled PWM drive.  
- **Duty logic:**  
  After the first turn, the work duty increases (`+30`) to compensate for frictional slowdown.  
- **Final Stop:**  
  The final loop executes `finalBrakeStop()` — a hard brake, reverse pulse, and final coast to stop completely.

---

## **Version 2 — MPU6050 Integration + PID Pitch Stabilization**

**File:** `open-v2.ino`  
**Main Components:** ESP32 + **MPU6050** Accelerometer + Servo + DRV8871  

This iteration introduced the **first sensor-based feedback control system** to measure inclination and stabilize the platform in real time.

### **Key Additions**
- **PID controller** (`PID_v1`) for pitch stabilization.  
- **Moving average filtering** to smooth sensor readings.  
- **Manual reset button** to re-zero sensor offsets.  
- **Boost phase** at startup for initial acceleration, then regulated power.  

### **Code Breakdown**
- **Sensor initialization & readout (I²C):**  
  Initializes the IMU over I²C, then reads raw acceleration on X/Y/Z every loop.  
  Data is converted to g’s and fed into a **moving-average filter** (ring buffer) for noise reduction.
- **Pitch calculation:**  
  Uses trigonometry to compute tilt:  
  `pitch = atan2(x, sqrt(y² + z²)) * 180/π` (degrees).  
  The filtered **pitch** becomes the **PID Input**.
- **PID control → steering:**  
  `Setpoint = 0` (level). `myPID.Compute()` calculates a correction sent to the servo with a safe clamp (e.g., 50–130°).  
  Proper tuning of **Kp/Ki/Kd** balances reaction speed and overshoot.
- **Motor drive policy:**  
  The motor runs at a defined **work duty** while the servo continuously adjusts heading based on the PID output to maintain stability.

---

## **Version 3 — Pixy2 Vision Integration**

**File:** `open-v3.ino`  
**Main Components:** ESP32 + Pixy2 Camera + Servo + DRV8871  

This version added **computer vision** capabilities for **color-based object detection** using the Pixy2 sensor.

### **Main Functionalities**
- Detects color blocks (signatures 1–2) via SPI communication.  
- Implements a **search behavior** through short forward pulses.  
- When a target is detected, performs a **turn and boost** maneuver.  
- Keeps the servo centered when idle.

### **Code Breakdown**
- **Camera initialization & reading:**  
  `pixy.init()` followed by `pixy.ccc.getBlocks()` each cycle reads detected objects.  
  Iterates `pixy.ccc.blocks[]` and checks `m_signature` for target classes (1 or 2).
- **Search gait (pulse & read):**  
  The robot advances in **short PWM bursts** (e.g., 20 ms on / 30 ms off) to reduce motion blur and give the camera stable frames.
- **Target reaction:**  
  On detection: pause → steer to a fixed angle → **boost** for `BOOST_TIME` → settle to a cruise duty.  
  This produces a decisive “detect → orient → accelerate” behavior.
- **Idle posture:**  
  With no detections, the servo **re-centers** (e.g., 90°) to maintain a straight scan line.

---

## **Version 4 — MPU6050 Gyro Integration + Yaw Reset Logic**

**File:** `open-v4.ino`  
**Main Components:** ESP32 + MPU6050 IMU + Servo + DRV8871  

This iteration integrates a **6-axis IMU (MPU6050)** combining gyroscope and accelerometer data to manage **orientation and heading** in real time.

### **Major Features**
- Real-time **yaw integration** from gyroscope data (`gz`).  
- Automatic **yaw reset** every ~1.5 s to mitigate drift.  
- **Autonomous stop** after six resets (lap completion logic).  
- **Active braking** with a brief **reverse** before final halt.  
- **Dynamic speed mapping** from longitudinal acceleration.

### **Code Breakdown**
- **IMU reading & integration:**  
  Reads `ax, ay, az, gx, gy, gz`.  
  Computes loop `dt` and integrates yaw: `yaw += (gz/131.0) * dt`.
- **Control mapping:**  
  - **Steering:** maps yaw to servo angle (e.g., 60–120°, clamped) to correct heading.  
  - **Speed:** maps longitudinal acceleration to PWM duty for forward thrust.
- **Yaw reset & stop logic:**  
  Every ~1.5 s: re-center yaw baseline and increment a **reset counter**.  
  After **six resets**: active brake → motor off → servo center → short reverse to clear the line → final idle.
- **Braking detail:**  
  DRV8871 **active brake** drives both inputs (brief pulse) to dissipate kinetic energy, enabling a crisp stop.

---

Each code version represents a **milestone** in the robot’s autonomous development —  
progressing from **manual timing** to **sensor fusion and vision-based control**, building a strong foundation for the final **competition-ready system**.

---

*© 2025 TEAM DELTA — PUCMM · All rights reserved.*
