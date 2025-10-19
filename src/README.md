   This module implements the control logic for an autonomous robot car using:
   - **ESP32** as the main microcontroller  
   - **Servo motor** for Ackermann steering  
   - **1 DC motor** driven by a **DRV8871 H-Bridge** (PWM + brake)  
   - **2 ultrasonic sensors (HC-SR04)** for left/right distance measurement  
   - **MPU6050 IMU** for yaw estimation (gyro integration)  
   - **Incremental encoder** (channel A + channel B for direction)  
   - **Start button** with blocking logic at startup  

   ---

   ## System Overview

   1. **Sensing**
      - **Ultrasonics**: measure left/right distances with a **median of 3** for robustness.  
      - **IMU (MPU6050)**: reads raw accelerometer/gyroscope and integrates **gz** to estimate **yaw angle** in degrees.  
      - **Encoder**: ISR on channel A, with channel B defining the rotation direction. Used to calculate **position pulses** and **RPM** every 300 ms.

   2. **Control Strategy**
      - **Steering**: computed from the **difference** between left/right distances (`diff = d1 - d2`) mapped to servo angles.  
      - **Speed**: mapped from **accelerometer X-axis** (accelX in m/s²) to PWM duty cycle (115–255).  

   3. **Actuation**
      - **Servo**: controlled with `Servo.write(angle)`.  
      - **Motor**: controlled via `ledcWrite(motorPin, duty)` at **10 kHz**; `otherPin` is used for active brake.  

   4. **State Management**
      - **Startup lock**: waits for button press before motor activation.  
      - **Stop condition**: if `pos >= 3200` pulses, motor stops permanently.  

   ---

   ## Hardware and Pin Mapping

   ### Encoder
   - `SA = 27` → Channel A (interrupt, RISING)  
   - `SB = 14` → Channel B (direction sense)  

   ### Start Button
   - `BUTTON_PIN = 23` (INPUT_PULLUP; pressed = LOW)  

   ### Ultrasonics
   - Right sensor → `trigPin1 = 25`, `echoPin1 = 26`  
   - Left sensor → `trigPin2 = 32`, `echoPin2 = 33`  
   > **Note:** readings use **median of 3** with timeout handling.  

   ### Servo
   - `servoPin = 15`  
   - Default center (to be calibrated): `SERVO_CENTER = 87`  

   ### Motor + DRV8871
   - **PWM (speed)**: `motorPin = 12` (LEDC @ 10 kHz, 8-bit)  
   - **Direction/Brake**: `otherPin = 13`  

   ### IMU (MPU6050, I2C)
   - Connected via ESP32 default SDA/SCL  
   - Initialized with `mpu.initialize();`  

   ---

   ## Key Variables and Constants

   - `PULSOS_POR_REV = 12` → pulses per revolution of encoder  
   - `ENCODER_INTERVAL = 300 ms` → window for RPM calculation  
   - `PRINT_INTERVAL = 200 ms` → interval for logging yaw (currently commented)  
   - `yaw` → integrated heading in degrees  
   - `pos` → encoder pulses (signed)  
   - `rpm` → calculated every 300 ms  
   - `YAW_THRESHOLD = 5°`, `DIST_TRIGGER = 100 cm` (declared, not used yet)  

   ---

   ## IMU (Yaw Calculation)

   1. Read raw values:  
      ```cpp
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      ```
   2. Convert `gz` to deg/s:  
      ```cpp
      yaw_rate = gz / 131.0;
      ```
   3. Integrate:  
      ```cpp
      yaw += yaw_rate * dt;  // dt in seconds
      ```
   > No drift correction (no complementary or Kalman filter). Long runs will accumulate error.  

   ---

   ## Encoder: Position and RPM

   - **ISR** (`readEncoder`) increments or decrements `pos` depending on channel B.  
   - RPM calculation every 300 ms:  
   ```cpp
   delta = pos - lastPos;
   rpm = (delta / PULSOS_POR_REV) * (60000.0 / ENCODER_INTERVAL);
   ```  

   ---

   ## Ultrasonic Readings

   - `medirUna(trig, echo)`:  
   - Sends 10 µs trigger pulse  
   - Reads echo with timeout (25 ms max)  
   - Converts duration to cm  
   - Returns 400 if invalid (timeout or <3 cm)  

   - `medirDistancia(trig, echo)`:  
   - Takes **3 readings** with delays  
   - Returns the **median value**  

   ---

   ## Steering Logic

   - Computes difference: `diff = d1 - d2`  
   - Maps difference to servo angle:  
   ```cpp
   servoAngle = map(diff, -10, 10, 70, 110);
   servoAngle = constrain(servoAngle, 50, 120);
   serv.write(servoAngle);
   ```

   ---

   ## Motor Control (DRV8871)

   - Configured at **10 kHz, 8-bit resolution**  
   - Speed control via:  
   ```cpp
   ledcWrite(motorPin, duty);
   digitalWrite(otherPin, LOW);
   ```
   - Brake:  
   ```cpp
   digitalWrite(otherPin, HIGH);
   ledcWrite(motorPin, 255);
   delay(100);
   ledcWrite(motorPin, 0);
   ```

   ---

   ## Program Lifecycle

   ### `setup()`
   1. Initialize Serial, encoder pins, attach ISR  
   2. Start I2C and IMU  
   3. Configure ultrasonic pins  
   4. Configure start button (INPUT_PULLUP)  
   5. Attach servo, move to neutral  
   6. Setup motor PWM (LEDC)  
   7. **Wait loop**: blocks until button pressed for 2 s  

   ### `loop()`
   1. Measure distances (left/right ultrasonic)  
   2. Update yaw from IMU integration  
   3. Update RPM from encoder (every 300 ms)  
   4. Update steering angle (from ultrasonic diff)  
   5. Compute motor velocity (from accelX) and apply PWM  
   6. Stop permanently if encoder pulses exceed 3200  

   ---

   ## Calibration

   - **Servo center**: adjust `SERVO_CENTER` or apply offset until robot drives straight.  
   - **Ultrasonic mapping**: adjust `map(diff, -10..10)` to match your robot’s scale.  
   - **Motor scaling**: ensure accel axis matches forward movement; rescale if needed.  
   - **Encoder PULSOS_POR_REV**: must match your encoder hardware.  

   ---

   ## Safety Notes

   - Startup is blocked until button press (prevents accidental motion).  
   - Servo is constrained to 50–120° to avoid hardware damage.  
   - Motor brake is available but not actively used in main loop.  
   - Ultrasonics use timeout + median filtering to reduce noise.  
   - IMU yaw has drift; not suitable for long-term navigation without fusion.  

   ---

   ## Function Reference

   - `medirUna(trig, echo)` → single ultrasonic read  
   - `medirDistancia(trig, echo)` → median of 3 ultrasonic reads  
   - `readEncoder()` → ISR to update encoder pulses  
   - `setMotorSpeed(speed)` → set motor duty cycle (basic)  
   - `brakeMotor()` → apply active brake  

---

   ## Suggested Improvements

   - Add **PID speed control** using encoder feedback  
   - Use **complementary filter** for yaw correction  
   - Implement **fail-safe stop** when ultrasonic < threshold  
   - Expand to **bi-directional motor control** (two PWM inputs)  
   - Add **LED or buzzer** for startup/stop feedback  

---

## Previous Versions

For earlier iterations of the robot’s control system — including motion-only logic, PID stabilization, Pixy2 vision integration, and IMU-based yaw reset — see the  
**[Releases documentation](../releases/README.md)** located in the `src/releases` directory.

This section details the evolution of the system through multiple code versions (`open-v1.ino` to `open-v4.ino`), showing how control strategies and sensor integration improved toward the final design.
