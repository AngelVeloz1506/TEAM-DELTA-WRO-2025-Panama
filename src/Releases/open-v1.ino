// VUELTA: funciones + reverse-kick breve en cada giro + boost tras freno (3 primeros)
//          frenado completo solo en el último + dutyWork +30 tras el primer giro

#include <ESP32Servo.h>

// ===== Pines =====
const int IN_PWM    = 12;   // Entrada PWM activa (adelante)
const int IN_OTHER  = 14;   // La otra entrada: LOW en avance; PWM en reversa
const int SERVO_PIN = 19;   // Servo

// ===== PWM =====
const int PWM_FREQ  = 7000;     // (sube a 20000 si quieres inaudible)
const int PWM_RES   = 8;        // 0..255
const uint8_t DUTY_WORK = 70;   // Duty base de trabajo
const uint8_t DUTY_MAX  = 200;  // Boost / potencia máxima
int currentDutyWork = DUTY_WORK;

// ===== Servo / Giros =====
Servo servo;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT  = 120;

// ===== Tiempos =====
const uint32_t WAIT_BEFORE_TURN_MS = 200;
const uint32_t HOLD_TURN_MS        = 790;
const uint32_t RETURN_TRAVEL_MS    = 350;
const uint32_t BOOST_AFTER_CENTER  = 300;

// ===== Frenados =====
const uint16_t REVERSE_KICK_TURN_MS = 120; // reverse-kick breve en CADA giro (1..3)
const uint16_t BOOST_AFTER_BRAKE_MS = 150; // boost corto después del freno en giros 1..3
const uint16_t BRAKE_MS             = 180; // freno activo corto (solo último giro)
const uint16_t REVERSE_KICK_MS      = 280; // reverse-kick largo (solo último giro)

// ================= Helpers de motor (IN1/IN2) =================
inline void coast() {
  pinMode(IN_OTHER, OUTPUT);
  digitalWrite(IN_OTHER, LOW);
  ledcWrite(IN_PWM, 0);
}

inline void brakeHard() {         // IN1=1, IN2=1 (ráfagas cortas)
  digitalWrite(IN_OTHER, HIGH);
  ledcWrite(IN_PWM, 255);
}

inline void setForwardDuty(uint8_t duty) {
  digitalWrite(IN_OTHER, LOW);
  ledcAttach(IN_PWM, PWM_FREQ, PWM_RES);
  ledcWrite(IN_PWM, duty);
}

inline void boostForward(uint32_t ms, uint8_t duty = DUTY_MAX) {
  setForwardDuty(duty);
  delay(ms);
}

inline void reverseKick(uint8_t duty, uint32_t ms) {
  ledcWrite(IN_PWM, 0);                      // IN_PWM=LOW
  ledcAttach(IN_OTHER, PWM_FREQ, PWM_RES);   // PWM en IN_OTHER (reversa)
  ledcWrite(IN_OTHER, duty);
  delay(ms);
  ledcWrite(IN_OTHER, 0);
  pinMode(IN_OTHER, OUTPUT);                 // restaurar avance
  digitalWrite(IN_OTHER, LOW);
  ledcAttach(IN_PWM, PWM_FREQ, PWM_RES);
}

// ================= Helpers de maniobra =================
inline void waitBeforeTurn()      { delay(WAIT_BEFORE_TURN_MS); }
inline void turnRightMaxPower()   { servo.write(SERVO_RIGHT); boostForward(HOLD_TURN_MS); }
inline void returnCenterMaxPower(){ servo.write(SERVO_CENTER); delay(RETURN_TRAVEL_MS); }
inline void settleAtWorkDuty()    { delay(BOOST_AFTER_CENTER); setForwardDuty(currentDutyWork); }

// Freno breve por giro (1..3) + boost corto para reanudar
inline void perTurnBrakeAndBoost() {
  reverseKick(DUTY_MAX, REVERSE_KICK_TURN_MS);     // frena con reversa breve
  boostForward(BOOST_AFTER_BRAKE_MS, DUTY_MAX);    // pequeño empujón adelante
  setForwardDuty(currentDutyWork);                 // vuelve al duty de trabajo
}

// Frenado final (solo en el último giro)
inline void finalBrakeStop() {
  brakeHard();                          delay(BRAKE_MS);
  reverseKick(DUTY_MAX, REVERSE_KICK_MS);
  brakeHard();                          delay(BRAKE_MS);
  coast();
}

// ================= Setup / Loop =================
void setup() {
  Serial.begin(115200);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(SERVO_CENTER);

  pinMode(IN_OTHER, OUTPUT);
  digitalWrite(IN_OTHER, LOW);

  ledcAttach(IN_PWM, PWM_FREQ, PWM_RES);

  // Boost de arranque y duty inicial
  boostForward(1000);
  setForwardDuty(currentDutyWork);

  Serial.println("Listo: funciones separadas, reverse-kick + boost en giros 1..3, frenado completo en el último.");
}

void loop() {
  static int ciclos = 0;

  if (ciclos < 3) {
    waitBeforeTurn();
    turnRightMaxPower();        // giro a la derecha con potencia máxima
    returnCenterMaxPower();     // vuelve a 90° manteniendo potencia
    perTurnBrakeAndBoost();     // frena breve (reversa) y boost corto

    ciclos++;

    // +30 de duty tras el primer giro
    if (ciclos == 1) {
      currentDutyWork = min(currentDutyWork + 30, 255);
      Serial.printf("Duty de trabajo aumentado a %d tras el primer giro.\n", currentDutyWork);
    }
  }
  else if (ciclos == 3) {
    // Último giro: mismo flujo pero termina con frenado completo y sin boost final
    waitBeforeTurn();
    turnRightMaxPower();
    returnCenterMaxPower();

    finalBrakeStop();           // brake -> reverse kick largo -> brake -> coast
    ciclos++;
    Serial.println("Frenado final completado. Detenido.");
  }
  else {
    // Fin
    delay(50);
  }
}