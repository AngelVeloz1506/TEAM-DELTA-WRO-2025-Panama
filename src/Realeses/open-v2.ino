/*
 * ADXL345 con ESP32
 * Lectura estable de aceleraciones y ángulos (Pitch y Roll)
 * con botón de reinicio de offset y filtrado de lecturas.
 */
#include <PID_v1.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>

// ==== PID ====                         <<< NUEVO BLOQUE
// Variables del PID
double Setpoint, Input, Output;
double Kp = 3.0, Ki = 0.5, Kd = 1.2;  // Ajusta estos valores con prueba y error

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// ==== Pines ====
const int IN_PWM    = 12;   // Pin PWM para controlar el motor
const int IN_OTHER  = 14;   // Pin fijo en LOW (configuración del puente H)

// ==== PWM ====
const int PWM_FREQ  = 7000;     // Frecuencia del PWM en Hz
const int PWM_RES   = 8;        // Resolución del PWM (0-255)
const uint8_t DUTY_WORK = 190;  // Duty normal de trabajo (~47%)
const uint8_t DUTY_MAX  = 210;  // Duty máximo (~82%)

// ==== Tiempos de maniobra ====
const uint32_t BOOST_TIME_MS = 1000;  // Tiempo de boost al inicio

Servo serv;

// Dirección I2C del ADXL345
const int ADXL345_ADDRESS = 0x53;

// Pin del botón de reinicio
const int BUTTON_PIN = 25; // GPIO del ESP32

// Variables raw
int16_t rawX, rawY, rawZ;

// Offsets
int16_t offsetX = 0;
int16_t offsetY = 0;
int16_t offsetZ = 0;

// Promedio móvil (para suavizar valores)
const int FILTER_SIZE = 10;
float xBuffer[FILTER_SIZE], yBuffer[FILTER_SIZE], zBuffer[FILTER_SIZE];
int filterIndex = 0;

// Valores finales en g
float x, y, z;

// Ángulos
float pitch, roll;

// Función para inicializar filtro
void resetFilter() {
  for (int i = 0; i < FILTER_SIZE; i++) {
    xBuffer[i] = 0;
    yBuffer[i] = 0;
    zBuffer[i] = 0;
  }
  filterIndex = 0;
}

float movingAverage(float *buffer, float newValue) {
  buffer[filterIndex] = newValue;
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SIZE;
}

void restart(){
    offsetX = rawX;
    offsetY = rawY;
    offsetZ = rawZ;
    Serial.println("Offset reiniciado.");
    delay(500);

  // Aplicar offset
  rawX -= offsetX;
  rawY -= offsetY;
  rawZ -= offsetZ;
}

int rn;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  serv.attach(19);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Poner ADXL345 en modo de medición
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();

  resetFilter();

  Serial.println("ADXL345 inicializado. Presione el botón para reiniciar el offset.");

    // Configuración del pin "IN_OTHER" en LOW
  pinMode(IN_OTHER, OUTPUT);
  digitalWrite(IN_OTHER, LOW);

  // Configuración del PWM en el pin IN_PWM
  ledcAttach(IN_PWM, PWM_FREQ, PWM_RES);

  // Boost inicial (motor a máxima potencia por 1s)
  ledcWrite(IN_PWM, DUTY_MAX);
  //delay(BOOST_TIME_MS);

  // ==== PID config ====                <<< AGREGADO
  Setpoint = 0; // queremos mantener pitch = 0°
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(30, 150); // límites del servo


}

unsigned long previousMillis = 0;   // stores last time function was called
const long interval = 5000;         // interval in ms (5 seconds)

void loop() {
  // Pasar a potencia de trabajo
  ledcWrite(IN_PWM, DUTY_WORK);
  //Serial.println("Motor en marcha con potencia de trabajo.");
  rn += 1;
  // Leer datos del ADXL345
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(0x32);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDRESS, 6, true);

  rawX = Wire.read() | (Wire.read() << 8);
  rawY = Wire.read() | (Wire.read() << 8);
  rawZ = Wire.read() | (Wire.read() << 8);



  // Convertir a g’s
  float gx = rawX / 256.0;
  float gy = rawY / 256.0;
  float gz = rawZ / 256.0;

  // Filtrar (promedio móvil)
  x = movingAverage(xBuffer, gx);
  y = movingAverage(yBuffer, gy);
  z = movingAverage(zBuffer, gz);

  filterIndex = (filterIndex + 1) % FILTER_SIZE;

  // Calcular ángulos (usamos radianes → grados al final)
  pitch = atan2(x, sqrt(y * y + z * z)) * 180.0 / PI;
  roll  = atan2(y, z) * 180.0 / PI;
    // ==== PID ====                       <<< CAMBIO
  Input = pitch;         // valor actual
  myPID.Compute();       // calcula la corrección
  serv.write(constrain(Output, 50, 130));    // mueve el servo

  // Debug


  unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
    // save the last time
    previousMillis = currentMillis;

    // run your function
    //serv.write(120);
    //delay(1000);
    //restart();
  }


  // Mostrar resultados
  Serial.print("Aceleración [g]: X=");
  Serial.print(x, 2);
  Serial.print(", Y=");
  Serial.print(y, 2);
  Serial.print(", Z=");
  Serial.println(z, 2);

  Serial.print("Ángulos [°]: Pitch=");
  Serial.print(pitch, 2);
  Serial.print(", Roll=");
  Serial.println(roll, 2);
  Serial.print(Output);

  Serial.println("--------------------");
  
  delay(100); // respuesta más fluida
}