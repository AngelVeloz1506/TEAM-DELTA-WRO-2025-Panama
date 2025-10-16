#include <Arduino.h>
#include <ESP32Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define SA 27
#define SB 14

int pos = 0;   // posición absoluta (pulsos)
int lastPos = 0;
int rpm = 1;

// Ajusta según tu encoder
const int PULSOS_POR_REV = 12;
unsigned long lastTimeza = 0;   // encoder timer
const unsigned long ENCODER_INTERVAL = 300; // ms

// --- Global variables ---
float targetYaw = 0;
bool correctingYaw = false;

const int YAW_THRESHOLD = 5;   // tolerance in degrees
const int DIST_TRIGGER = 100;  // cm

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
  long a = medirUna(trigPin, echoPin); delayMicroseconds(500);
  long b = medirUna(trigPin, echoPin); delayMicroseconds(500);
  long c = medirUna(trigPin, echoPin);
  if (a > b) { long t=a; a=b; b=t; }
  if (b > c) { long t=b; b=c; c=t; }
  if (a > b) { long t=a; a=b; b=t; }
  return b;
}

int counter = 0;

void IRAM_ATTR readEncoder() {
  int b = digitalRead(SB);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}

// --- Funciones Motor ---
void setMotorSpeed(int speed) {   
  digitalWrite(otherPin, LOW);
  ledcWrite(motorPin, speed);
}

void brakeMotor() {
  digitalWrite(otherPin, HIGH);
  ledcWrite(motorPin, 255);
  delay(100);
  ledcWrite(motorPin, 0);
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

  serv.attach(servoPin);
  serv.write(90);

  pinMode(otherPin, OUTPUT);
  ledcAttach(motorPin, freq, resolution);
  digitalWrite(otherPin, LOW);
  ledcWrite(motorPin, 255);
}

void loop() {
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

  // ====== Encoder update every 300 ms ======
  if (now - lastTimeza >= ENCODER_INTERVAL) {
    int delta = pos - lastPos;
    lastPos = pos;

    rpm = (delta / (float)PULSOS_POR_REV) * (60000.0 / ENCODER_INTERVAL);

    /*Serial.print("Posicion (pulsos): ");
    Serial.print(pos);
    Serial.print(" | RPM: ");
    Serial.println(rpm);*/

    lastTimeza = now;
  }

  // ====== Print yaw every 200 ms ======
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    /*Serial.print("Yaw: ");
    Serial.println(yaw);*/
    lastPrintTime = now;
  }

  // --- ultrasonic steering ---
  int diff = d1 - d2;
  int servoAngle = map(diff, -20, 20, 80, 100);
  serv.write(constrain(servoAngle, 50, 120));

  // ====== Motor speed control ======
  float accelX_g = (float)ax / 16384.0;
  float accelX_ms2 = accelX_g * 9.81;
  int motorVel = map(accelX_ms2, -5, 5, 115, 255);
  motorVel = constrain(motorVel, 0, 255);
  digitalWrite(otherPin, LOW);
  ledcWrite(motorPin, motorVel);

  /*if (rpm == 0) {
    if(yaw > 0){
      serv.write(120);
      digitalWrite(otherPin, HIGH);
      ledcWrite(motorPin, 255);
      delay(2000);      
    }
    else if (yaw < 0){
      serv.write(60);
      digitalWrite(otherPin, HIGH);
      ledcWrite(motorPin, 255);
      delay(2000);        
    }
  }*/
}