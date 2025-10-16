#include <Arduino.h>
#include <ESP32Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

#define SA 27
#define SB 14
#define BUTTON_PIN 17

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;

float v1Filt = 0;
float v1Prev = 0;

const int PULSOS_POR_REV = 12;

// --- IMU ---
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

float yaw = 0;
unsigned long lastIMUTime = 0;    // for integration
unsigned long lastPrintTime = 0;  // for printing
const unsigned long PRINT_INTERVAL = 200; // ms

// --- Pines de los ultrasonicos ---
const int trigPin1 = 25;
const int echoPin1 = 26;
const int trigPin2 = 32;
const int echoPin2 = 33;

// --- Servo ---
Servo serv;
const int servoPin = 15;

// --- Motor DC ---
const int motorPin = 12;   // PWM velocidad
const int otherPin = 13;   // Dirección / freno
const int freq = 10000;    // 10 kHz
const int resolution = 8;  // 8 bits (0-255)

static inline long medirUna(int trigPin, int echoPin) {
  unsigned long t = micros();
  while (digitalRead(echoPin) == HIGH && (micros() - t) < 2000) {}
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long dur = pulseInLong(echoPin, HIGH, 25000UL);
  if (dur == 0) return 400;
  long cm = (long)(dur * 0.0343f * 0.5f);
  if (cm < 3) return 400;
  return cm;
}
long medirDistancia(int trigPin, int echoPin) {
  long a = medirUna(trigPin, echoPin); delayMicroseconds(500);
  long b = medirUna(trigPin, echoPin); delayMicroseconds(500);
  long c = medirUna(trigPin, echoPin);
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b;
}

// ===== PID Variables =====
double Setpoint, Input, Output;

// Initial guess for PID tuning
double Kp = 0.48, Ki = 0.4375, Kd = 0.109375;

// Create PID object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ===== Encoder ISR =====
void IRAM_ATTR readEncoder() {
  int b = digitalRead(SB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;
}

enum State { NORMAL, GREEN, RED};
State state = NORMAL;

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  pinMode(SA, INPUT_PULLUP);
  pinMode(SB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SA), readEncoder, RISING);

  Wire.begin(); 
  mpu.initialize();

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  // Configuración del botón
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  serv.attach(servoPin);
  serv.write(87);

  pinMode(otherPin, OUTPUT);
  digitalWrite(otherPin, LOW);  
  ledcAttach(motorPin, freq, resolution);

    // ===== PID Setup =====
  Setpoint = 500; // Target RPM
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range

  while(true){
    if(digitalRead(BUTTON_PIN) == LOW){
      delay(500);
      Serial.println("Started...");
      return;
    }
  }
}

void loop() {
  String data = Serial.readStringUntil('\n');

  int ID = data.substring(0, 1).toInt();
  int X    = data.substring(1, 4).toInt();
  int W = data.substring(4, 7).toInt();

  switch(state){
    case NORMAL: {
      
      Serial.println("NORMAL");
      digitalWrite(2, LOW);
      int pos = 0;
      noInterrupts();
      pos = pos_i;
      interrupts();

      long currT = micros();
      float deltaT = (currT - prevT) / 1.0e6;
      float velocity1 = (pos - posPrev) / deltaT;
      posPrev = pos;
      prevT = currT;

      float v1 = velocity1 / PULSOS_POR_REV * 60.0;

      v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
      v1Prev = v1;

      Input = v1Filt;
      
      myPID.Compute();
      ledcWrite(motorPin, (int)Output);

      // ====== Read ultrasonics ======
      long d1 = medirDistancia(trigPin2, echoPin2); // left
      long d2 = medirDistancia(trigPin1, echoPin1); // right

      unsigned long now = millis();
      unsigned long nowMicros = micros();

      // ====== IMU update (always, with real Δt) ======
      float dt = (nowMicros - lastIMUTime) / 1000000.0;  // seconds
      lastIMUTime = nowMicros;

      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      float yaw_rate = (float)gz / 131.0; // deg/s
      yaw += yaw_rate * dt;

      // --- ultrasonic steering ---
      int diff = d1 - d2;
      int servoAngle = map(diff, -20, 20, 47, 127);
      serv.write(constrain(servoAngle, 57, 117));

      // ====== Motor speed control ======
      float accelX_g = (float)ax / 16384.0;
      float accelX_ms2 = accelX_g * 9.81;
      int motorVel = map(accelX_ms2, -5, 5, 115, 255);
      motorVel = constrain(motorVel, 0, 255);

      unsigned long currentMillis = millis();

      if(yaw > 1105 && diff < 10){
        delay(350);
        serv.write(87);    
        ledcWrite(motorPin, 0);
        while(true){
        }
      }

      if(yaw < -1030 && diff < 10){
        delay(150);
        serv.write(87);    
        ledcWrite(motorPin, 0);
        while(true){
        }
      }
      if (ID == 0 && W > 20) state = GREEN;
      else if (ID == 1 && W > 20) state = RED;
      break;
    }

    case GREEN: {

      Serial.println("GREEN");
      digitalWrite(2, HIGH);
      unsigned long now = millis();
      unsigned long nowMicros = micros();

      // ====== IMU update (always, with real Δt) ======
      float dt = (nowMicros - lastIMUTime) / 1000000.0;  // seconds
      lastIMUTime = nowMicros;

      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      float yaw_rate = (float)gz / 131.0; // deg/s
      yaw += yaw_rate * dt;

      int pos = 0;
      noInterrupts();
      pos = pos_i;
      interrupts();

      long currT = micros();
      float deltaT = (currT - prevT) / 1.0e6;
      float velocity1 = (pos - posPrev) / deltaT;
      posPrev = pos;
      prevT = currT;

      float v1 = velocity1 / PULSOS_POR_REV * 60.0;

      v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
      v1Prev = v1;

      Input = v1Filt;
        
      myPID.Compute();
      ledcWrite(motorPin, (int)Output);

      int steer = map(X, 0, 1700, 47, 127);
      serv.write(steer);

      if(yaw > 1105){
        delay(350);
        serv.write(87);    
        ledcWrite(motorPin, 0);
        while(true){
        }
      }

      if(yaw < -1030){
        delay(150);
        serv.write(87);    
        ledcWrite(motorPin, 0);
        while(true){
        }
      }

      if (X > 750) state = NORMAL;
      break;
    }

    case RED: {

      Serial.println("RED");
      digitalWrite(2, HIGH);
      unsigned long now = millis();
      unsigned long nowMicros = micros();

      // ====== IMU update (always, with real Δt) ======
      float dt = (nowMicros - lastIMUTime) / 1000000.0;  // seconds
      lastIMUTime = nowMicros;

      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      float yaw_rate = (float)gz / 131.0; // deg/s
      yaw += yaw_rate * dt;

      int pos = 0;
      noInterrupts();
      pos = pos_i;
      interrupts();

      long currT = micros();
      float deltaT = (currT - prevT) / 1.0e6;
      float velocity1 = (pos - posPrev) / deltaT;
      posPrev = pos;
      prevT = currT;

      float v1 = velocity1 / PULSOS_POR_REV * 60.0;

      v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
      v1Prev = v1;

      Input = v1Filt;
        
      myPID.Compute();
      ledcWrite(motorPin, (int)Output);

      int steer = map(X, -870, 830, 47, 127);
      serv.write(steer);

      if(yaw > 1105){
        delay(350);
        serv.write(87);    
        ledcWrite(motorPin, 0);
        while(true){
        }
      }

      if(yaw < -1030){
        delay(150);
        serv.write(87);    
        ledcWrite(motorPin, 0);
        while(true){
        }
      }

      // Return to normal if all objects are far
      if (X < 50) state = NORMAL;
      break;
    }
  }
}