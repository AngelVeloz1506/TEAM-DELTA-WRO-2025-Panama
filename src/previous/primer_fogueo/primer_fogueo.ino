// 6 turns: angle increases +SERVO_STEP_DEG° ONLY during the first 4 turns
// Features: reverse-kick on each turn, post-brake boost,
// dutyWork +30 after the first turn, and full brake at the 6th turn.
// Guarantee: servo at 90° before the motor spins at startup.

#include <ESP32Servo.h>

// ===== Pins =====
const int IN_PWM    = 12;   // Active PWM input (forward)
const int IN_OTHER  = 14;   // The other input: LOW in forward; PWM in reverse
const int SERVO_PIN = 19;   // Servo

// ===== PWM =====
const int PWM_FREQ  = 7000;     // (increase to 20000 if you want inaudible PWM)
const int PWM_RES   = 8;        // 0..255
const uint8_t DUTY_WORK = 70;   // Base duty for forward motion
const uint8_t DUTY_MAX  = 200;  // Maximum duty (boost)
int currentDutyWork = DUTY_WORK;

// ===== Servo / Turns =====
Servo servo;
const int SERVO_CENTER     = 90;
const int SERVO_RIGHT_BASE = 120; // base angle for the first turn
const int SERVO_STEP_DEG   = 2;   // increment per turn (limited to 4 turns)

// ===== Timings =====
const uint32_t WAIT_BEFORE_TURN_MS = 200;
const uint32_t HOLD_TURN_MS        = 790;
const uint32_t RETURN_TRAVEL_MS    = 350;
const uint32_t BOOST_AFTER_CENTER  = 300;
const uint16_t SERVO_SETTLE_MS     = 800;  // ensure 90° at startup

// ===== Brakes =====
const uint16_t REVERSE_KICK_TURN_MS = 120; // short reverse-kick on each turn
const uint16_t BOOST_AFTER_BRAKE_MS = 150; // short boost after braking
const uint16_t BRAKE_MS             = 180; // active brake (final stop)
const uint16_t REVERSE_KICK_MS      = 280; // long reverse-kick (final stop)

// ================= Motor helpers (IN1/IN2) =================
inline void coast_gpio_lowlow() {   // use BEFORE attaching PWM
  pinMode(IN_PWM, OUTPUT);
  pinMode(IN_OTHER, OUTPUT);
  digitalWrite(IN_PWM, LOW);
  digitalWrite(IN_OTHER, LOW);
}

inline void brakeHard() {           // IN1=1, IN2=1 (short bursts)
  digitalWrite(IN_OTHER, HIGH);
  ledcWrite(IN_PWM, 255);
}

inline void setForwardDuty(uint8_t duty) {
  digitalWrite(IN_OTHER, LOW);
  ledcWrite(IN_PWM, duty);
}

inline void boostForward(uint32_t ms, uint8_t duty = DUTY_MAX) {
  setForwardDuty(duty);
  delay(ms);
}

inline void reverseKick(uint8_t duty, uint32_t ms) { // short reverse
  ledcWrite(IN_PWM, 0);                      // IN_PWM=LOW
  ledcAttach(IN_OTHER, PWM_FREQ, PWM_RES);   // PWM on IN_OTHER (reverse)
  ledcWrite(IN_OTHER, duty);
  delay(ms);
  ledcWrite(IN_OTHER, 0);
  // restore forward mode
  pinMode(IN_OTHER, OUTPUT);
  digitalWrite(IN_OTHER, LOW);
  ledcAttach(IN_PWM, PWM_FREQ, PWM_RES);
}

// ================= Maneuver helpers =================
inline void waitBeforeTurn() { delay(WAIT_BEFORE_TURN_MS); }

inline void turnRightMaxPowerAngle(int angleDeg) {
  angleDeg = constrain(angleDeg, 0, 180);
  servo.write(angleDeg);
  boostForward(HOLD_TURN_MS);      // drive strongly while turning
}

inline void returnCenterMaxPower() {
  servo.write(SERVO_CENTER);
  delay(RETURN_TRAVEL_MS);         // still with strong power
}

inline void perTurnBrakeAndBoost() {
  reverseKick(DUTY_MAX, REVERSE_KICK_TURN_MS);   // short brake with reverse
  boostForward(BOOST_AFTER_BRAKE_MS, DUTY_MAX);  // small push forward
  setForwardDuty(currentDutyWork);               // return to work duty
}

inline void finalBrakeStop() {
  brakeHard();                          delay(BRAKE_MS);
  reverseKick(DUTY_MAX, REVERSE_KICK_MS);
  brakeHard();                          delay(BRAKE_MS);
  // stop in low/low state
  ledcWrite(IN_PWM, 0);
  digitalWrite(IN_OTHER, LOW);
}

// Ensure servo is at 90° before motor spins (only at startup)
inline void ensureServoCenteredBeforeStart() {
  servo.write(SERVO_CENTER);
  coast_gpio_lowlow();          // motor completely stopped (GPIO LOW/LOW)
  delay(SERVO_SETTLE_MS);       // time for servo to settle at 90°
}

// ================= Setup / Loop =================
void setup() {
  Serial.begin(115200);

  // Servo
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(SERVO_CENTER);

  // Ensure both driver inputs are LOW as GPIO (no PWM yet)
  coast_gpio_lowlow();

  // Guarantee servo at 90° BEFORE enabling motor PWM
  ensureServoCenteredBeforeStart();

  // Now: attach PWM to the active pin
  ledcAttach(IN_PWM, PWM_FREQ, PWM_RES);

  // Startup boost and initial duty
  boostForward(1000);
  setForwardDuty(currentDutyWork);

  Serial.println("Ready: servo centered before start; 6 turns with increment only in the first 4.");
}

void loop() {
  static int giro = 0;

  if (giro < 6) {
    // Increment angle only in the first 4 turns (index 0..3)
    int incIndex = (giro < 4) ? giro : 3;
    int angleThisTurn = SERVO_RIGHT_BASE + incIndex * SERVO_STEP_DEG;

    waitBeforeTurn();
    turnRightMaxPowerAngle(angleThisTurn);
    returnCenterMaxPower();
    perTurnBrakeAndBoost();

    // +30 duty after the FIRST turn
    if (giro == 0) {
      currentDutyWork = min(currentDutyWork + 30, 255);
      Serial.printf("Work duty increased to %d after first turn.\n", currentDutyWork);
    }

    giro++;

    // On the 6th turn, apply full stop and end
    if (giro == 6) {
      finalBrakeStop();
      Serial.println("Sequence of 6 turns completed and final brake applied. Stopped.");
    }
  } else {
    // Stopped
    delay(100);
  }
}
