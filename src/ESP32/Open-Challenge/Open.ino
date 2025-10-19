#include <Arduino.h>
#include <ESP32Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>

unsigned long previousMillis = 0; // Stores the last time the action was run
const unsigned long interval = 1000; // Interval in milliseconds (e.g., 1000 ms = 1 second)

#define SA 27
#define SB 14

// ===== Globals =====
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;

float v1Filt = 0;
float v1Prev = 0;

const int PULSOS_POR_REV = 12;

#define BUTTON_PIN 17

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
bool dirc = false;

const int SERVO_CENTER = 87;  // prueba: ajústalo hasta que vaya recto

bool out;

// Lectura robusta HC-SR04 (mediana de 3, pulldown)
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
  long a = medirUna(trigPin, echoPin); delayMicroseconds(100);
  long b = medirUna(trigPin, echoPin); delayMicroseconds(100);
  long c = medirUna(trigPin, echoPin);
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b;
}

// ===== PID Variables =====
double Setpoint, Input, Output;

// Initial guess for PID tuning
double Kp = 0.54, Ki = 0.05, Kd = 0.0125;

// Create PID object
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ===== Encoder ISR =====
void IRAM_ATTR readEncoder() {
  int b = digitalRead(SB);
  int increment = (b > 0) ? 1 : -1;
  pos_i += increment;
}

void setup() {
  Serial.begin(115200);
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
  serv.write(90);

  pinMode(otherPin, OUTPUT);
  digitalWrite(otherPin, LOW);  
  ledcAttach(motorPin, freq, resolution);

    // ===== PID Setup =====
  Setpoint = 1300; // Target RPM
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(10, 255); // PWM range

  while(true){
    if(digitalRead(BUTTON_PIN) == LOW){
      delay(500);
      return;
    }
  }
}

void loop() {
  // Safely read encoder position
  int pos = 0;
  noInterrupts();
  pos = pos_i;
  interrupts();

  // Compute velocity from position difference
  long currT = micros();
  float deltaT = (currT - prevT) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert to RPM
  float v1 = velocity1 / PULSOS_POR_REV * 60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;

  Input = v1Filt;

  //delay(10);

  // ====== Run PID continuously ======
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
  int servoAngle = map(diff, -15, 15, 47, 127);
  serv.write(constrain(servoAngle, 50, 130));

  // ====== Motor speed control ======
  float accelX_g = (float)ax / 16384.0;
  float accelX_ms2 = accelX_g * 9.81;
  int motorVel = map(accelX_ms2, -5, 5, 115, 255);
  motorVel = constrain(motorVel, 0, 255);

  unsigned long currentMillis = millis();

  /*if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Reset timer
    yaw = 90;
  }*/

  /*if(pos >= 4000){
    //serv.write(90);
    ledcWrite(motorPin, 0);
    while(true){
    }
  }*/

  if(yaw > 1105 && diff < 5){
    delay(350);
    serv.write(87);    
    ledcWrite(motorPin, 0);
    while(true){
    }
  }

  if(yaw < -1030 && diff < 5){
    delay(150);
    serv.write(87);    
    ledcWrite(motorPin, 0);
    while(true){
    }
  }

  /*if (v1Filt == 0) {
    serv.write(90);
    digitalWrite(otherPin, HIGH);
    delay(2000);
    if(yaw > 0){
      serv.write(115);
      digitalWrite(otherPin, LOW);
      ledcWrite(motorPin, 255);
      delay(200);      
    }
    else if(yaw < 0){
      serv.write(70);
      digitalWrite(otherPin, LOW);
      ledcWrite(motorPin, 255);
      delay(200);   
    }
  }*/
}