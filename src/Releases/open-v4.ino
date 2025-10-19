#include <ESP32Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

Servo serv;
MPU6050 mpu;

// Variables para almacenar las lecturas raw de aceleracion y giro
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Variables para almacenar los angulos estimados
float yaw = 0;

// Variables para la estimacion de tiempo
unsigned long last_time;

// Variables para el reinicio del Yaw
unsigned long last_yaw_reset_time;
const unsigned long YAW_RESET_INTERVAL = 1550; // 1.5s

// Contador de resets de yaw
int yawResetCount = 0;

const int motorPin = 12;   // IN del DRV8871 (PWM)
const int otherPin = 14;   // IN2 del DRV8871 (dirección/freno)
const int freq = 10000;    // 10 kHz
const int resolution = 8;  // 8 bits (0-255)

bool stopped = false;  // bandera para saber si el robot ya se detuvo

void setMotorSpeed(int speed) {
  // speed: 0-255
  digitalWrite(otherPin, LOW);    // dirección (LOW = adelante)
  ledcWrite(motorPin, speed);     
}

void brakeMotor() {
  // Frenado rápido
  digitalWrite(otherPin, HIGH);
  ledcWrite(motorPin, 255); // máxima conducción para freno
  delay(100);               // tiempo corto de frenado (ajustable)
  ledcWrite(motorPin, 0);   // luego soltamos
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  // Servo
  serv.attach(18);

  // Motor DC
  pinMode(otherPin, OUTPUT);
  digitalWrite(otherPin, LOW);

  // Configurar PWM en el pin del motor
  ledcAttach(motorPin, freq, resolution);
  delay(10000);
  setMotorSpeed(255);
  delay(20);

  last_time = micros(); 
  last_yaw_reset_time = millis(); 
}

void loop() {
  if (stopped) {
    // Si ya alcanzó 6 resets → robot detenido con freno
    //brakeMotor();        // primero frena
    setMotorSpeed(0);    // luego asegura el motor apagado
    serv.write(90);      // centramos el servo
    return;
  }

  // Leer los valores de los 6 ejes
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  float yaw_rate = (float)gz / 131.0;
  yaw += yaw_rate * dt;

  // --- Mostrar aceleración en eje X ---
  float accelX_g = (float)ax / 16384.0;   // en g
  float accelX_ms2 = accelX_g * 9.81;     // en m/s²

  int ang = map(yaw, -90, 90, 60, 120);
  int Cvel = map(accelX_ms2, -20, 20, 150, 255);

  // --- Reinicio automatico del Yaw ---
  if (millis() - last_yaw_reset_time >= YAW_RESET_INTERVAL) {
    yaw = 90; // Reiniciamos el angulo de yaw a 90
    setMotorSpeed(255);
    last_yaw_reset_time = millis(); 

    yawResetCount++;  // incrementamos el contador
    Serial.print("Yaw reset #: ");
    Serial.println(yawResetCount);

// Si ya lo hizo 6 veces → detener robot
if (yawResetCount >= 6) {
  stopped = true;
  brakeMotor();       // frena antes de parar
  setMotorSpeed(0);   // apaga motor
  serv.write(90);
  Serial.println("Robot detenido tras 6 resets de yaw (con freno).");

  // 🚗 Retroceder ~5 cm antes de detenerse
  digitalWrite(otherPin, HIGH);   // invertir dirección → atrás
  ledcWrite(motorPin, 180);       // velocidad moderada
  delay(250);                     // ~5 cm (ajusta según pruebas)
  ledcWrite(motorPin, 0);         // apagar motor
  digitalWrite(otherPin, LOW);    // volver a dirección normal
}
  }

  serv.write(constrain(ang, 50, 130));
  setMotorSpeed(Cvel);

  //Serial.println(ang);
}