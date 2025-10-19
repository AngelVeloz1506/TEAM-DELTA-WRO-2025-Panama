#include <ESP32Servo.h>
#include "Wire.h"
#include <Pixy2.h>
#include <SPI.h>

Servo serv;
Pixy2 pixy;

const int motorPin = 12;   // IN1 PWM
const int otherPin = 14;   // IN2 dirección
const int freq = 10000;    // 10 kHz
const int resolution = 8;  // 8 bits (0-255)

// Velocidades
const int SPEED_SEARCH = 219;   // torque fuerte
const int SPEED_BOOST  = 255;   // turbo máximo
const int SPEED_AFTER  = 255;   // velocidad después del boost

// Tiempo de boost (ms)
const int BOOST_TIME = 900;

void setMotorSpeed(int speed) {
  digitalWrite(otherPin, LOW);    
  ledcWrite(motorPin, speed);
}

void setup() {
  serv.attach(17);

  pinMode(otherPin, OUTPUT);
  digitalWrite(otherPin, LOW);
  ledcAttach(motorPin, freq, resolution);

  pixy.init();
  serv.write(90);
}

void loop() {
  bool detected = false;

  // Avanza un paso corto
  setMotorSpeed(SPEED_SEARCH);
  delay(20);   // paso muy breve
  setMotorSpeed(0);
  delay(30);   // pausa para lectura

  // Lectura de Pixy
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks > 0) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      uint8_t sig = pixy.ccc.blocks[i].m_signature;
      if (sig == 1 || sig == 2) {
        detected = true;
        break;
      }
    }
  }

  if (detected) {
    setMotorSpeed(0);          
    delay(80);

    serv.write(135);           // giro

    // Boost
    setMotorSpeed(SPEED_BOOST);
    delay(BOOST_TIME);

    setMotorSpeed(SPEED_AFTER);
  } else {
    serv.write(90);  // centrado mientras busca
  }
}
'