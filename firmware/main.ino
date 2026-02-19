/*
 * Robot Shield Firmware
 * Hardware: Arduino Nano Every
 * Verified against schematic image
 *
 * Pin Assignments (confirmed from schematic):
 * ─────────────────────────────────────────────────────
 * L293D Motor Driver:
 *   D2  -> 1A  (Motor A, Input 1)
 *   D7  -> 2A  (Motor A, Input 2)
 *   D5  -> EN1,2 (Motor A Enable/PWM)
 *   D8  -> 3A  (Motor B, Input 1)
 *   D4  -> 4A  (Motor B, Input 2)
 *   D3  -> EN3,4 (Motor B Enable/PWM)
 *   J1  -> Motor A terminals (1Y, 2Y)
 *   J2  -> Motor B terminals (3Y, 4Y)
 *
 * HC-05 Bluetooth (J7, SoftwareSerial):
 *   D11 -> TX to HC-05 RX  (pin 3)
 *   D12 -> RX from HC-05 TX (pin 4)
 *
 * HC-SR04 Ultrasonic (J8):
 *   A3  -> Trig  (pin 3)
 *   A6  -> Echo  (pin 4)
 *
 * MPU6050 IMU (J11, I2C):
 *   A5  -> SCL   (pin 3)
 *   A4  -> SDA   (pin 4)
 *
 * Servo Headers:
 *   D6  -> Servo 1 signal (J9, pin 3)  ✓ confirmed from image
 *   D9  -> Servo 2 signal (J10, pin 3) ✓ confirmed from image
 *
 * IR Sensors (J4, J5, J6):
 *   A0  -> IR Sensor 1 signal (J4, pin 2)
 *   A1  -> IR Sensor 2 signal (J5, pin 2)
 *   A2  -> IR Sensor 3 signal (J6, pin 2)
 *
 * Buzzer (BZ1):
 *   D10 -> Buzzer + (pin 1)
 *
 * LED (D1):
 *   D13 -> LED anode
 *
 * Power:
 *   J3  -> Raw 7-12V input -> LM7805 (U2) -> +5V rail
 */

#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ─── Pin Definitions ─────────────────────────────────────────────────────────

// L293D Motor Driver
#define MOTOR_A_IN1   2    // 1A
#define MOTOR_A_IN2   7    // 2A
#define MOTOR_A_EN    5    // EN1,2 (PWM)
#define MOTOR_B_IN1   8    // 3A
#define MOTOR_B_IN2   4    // 4A
#define MOTOR_B_EN    3    // EN3,4 (PWM)

// HC-05 Bluetooth
#define BT_TX_PIN     11   // Arduino TX → HC-05 RX
#define BT_RX_PIN     12   // Arduino RX ← HC-05 TX

// HC-SR04 Ultrasonic
#define ULTRASONIC_TRIG  A3
#define ULTRASONIC_ECHO  A6   // Nano Every: A6 can be used with digitalRead()

// Servos — confirmed from schematic image
#define SERVO1_PIN    6    // J9  pin 3
#define SERVO2_PIN    9    // J10 pin 3

// IR Sensors
#define IR_SENSOR_1   A0
#define IR_SENSOR_2   A1
#define IR_SENSOR_3   A2

// Buzzer & LED
#define BUZZER_PIN    10
#define LED_PIN       13

// MPU6050 I2C address
#define MPU6050_ADDR  0x68
#define MPU_PWR_MGMT  0x6B
#define MPU_ACCEL_OUT 0x3B

// IR obstacle threshold (tune per sensor; lower = obstacle detected)
#define IR_THRESHOLD  500

// ─── Objects ─────────────────────────────────────────────────────────────────
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);
Servo servo1;
Servo servo2;

// ─── State ───────────────────────────────────────────────────────────────────
uint8_t motorSpeedA = 200;   // 0–255
uint8_t motorSpeedB = 200;
bool    autoMode    = true;  // true = obstacle-avoidance, false = BT manual

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);   // HC-05 default baud rate

  // Motor driver pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_A_EN,  OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_EN,  OUTPUT);

  // Ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Buzzer & LED
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN,    OUTPUT);

  // Servos — centre on startup
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(90);
  servo2.write(90);

  // MPU6050
  Wire.begin();
  mpu6050Init();

  stopMotors();

  // Startup sequence: 3 beeps + LED blinks
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    beep(80);
    digitalWrite(LED_PIN, LOW);
    delay(120);
  }

  Serial.println(F("=== Robot Shield Ready ==="));
  Serial.println(F("BT Commands:"));
  Serial.println(F("  F/B/L/R/S  - Move forward/back/left/right/stop"));
  Serial.println(F("  +/-        - Increase/decrease speed"));
  Serial.println(F("  A/M        - Auto / Manual mode"));
  Serial.println(F("  P          - Beep"));
  Serial.println(F("  1/2/3      - Servo1: 0/90/180 deg"));
  Serial.println(F("  4/5/6      - Servo2: 0/90/180 deg"));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  handleBluetooth();

  if (autoMode) {
    runAutoMode();
  }

  delay(30);
}

// ─── Autonomous Obstacle Avoidance ───────────────────────────────────────────
void runAutoMode() {
  long dist = getDistance();

  bool irL = (analogRead(IR_SENSOR_1) < IR_THRESHOLD);  // left IR
  bool irC = (analogRead(IR_SENSOR_2) < IR_THRESHOLD);  // centre IR
  bool irR = (analogRead(IR_SENSOR_3) < IR_THRESHOLD);  // right IR

  if ((dist > 0 && dist < 15) || irC) {
    // Obstacle straight ahead
    stopMotors();
    beep(80);
    delay(150);
    moveBackward();
    delay(400);
    if (irL) {
      turnRight();
    } else {
      turnLeft();
    }
    delay(350);
    stopMotors();

  } else if (irL) {
    // Obstacle on left — veer right
    turnRight();
    delay(200);

  } else if (irR) {
    // Obstacle on right — veer left
    turnLeft();
    delay(200);

  } else {
    moveForward();
  }
}

// ─── Bluetooth Command Handler ────────────────────────────────────────────────
void handleBluetooth() {
  if (!btSerial.available()) return;

  char cmd = (char)btSerial.read();
  Serial.print(F("BT: ")); Serial.println(cmd);

  switch (cmd) {
    // ── Movement ──────────────────────────────────────────
    case 'F': autoMode = false; moveForward();  break;
    case 'B': autoMode = false; moveBackward(); break;
    case 'L': autoMode = false; turnLeft();     break;
    case 'R': autoMode = false; turnRight();    break;
    case 'S': autoMode = false; stopMotors();   break;

    // ── Speed ─────────────────────────────────────────────
    case '+':
      motorSpeedA = min(255, motorSpeedA + 20);
      motorSpeedB = min(255, motorSpeedB + 20);
      Serial.print(F("Speed: ")); Serial.println(motorSpeedA);
      break;
    case '-':
      motorSpeedA = max(40,  motorSpeedA - 20);
      motorSpeedB = max(40,  motorSpeedB - 20);
      Serial.print(F("Speed: ")); Serial.println(motorSpeedA);
      break;

    // ── Mode ──────────────────────────────────────────────
    case 'A':
      autoMode = true;
      Serial.println(F("AUTO mode"));
      break;
    case 'M':
      autoMode = false;
      stopMotors();
      Serial.println(F("MANUAL mode"));
      break;

    // ── Buzzer ────────────────────────────────────────────
    case 'P': beep(300); break;

    // ── Servo 1 ───────────────────────────────────────────
    case '1': servo1.write(0);   break;
    case '2': servo1.write(90);  break;
    case '3': servo1.write(180); break;

    // ── Servo 2 ───────────────────────────────────────────
    case '4': servo2.write(0);   break;
    case '5': servo2.write(90);  break;
    case '6': servo2.write(180); break;

    default: break;
  }
}

// ─── Motor Control ────────────────────────────────────────────────────────────
void moveForward() {
  analogWrite(MOTOR_A_EN, motorSpeedA);
  analogWrite(MOTOR_B_EN, motorSpeedB);
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(LED_PIN, HIGH);
}

void moveBackward() {
  analogWrite(MOTOR_A_EN, motorSpeedA);
  analogWrite(MOTOR_B_EN, motorSpeedB);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  digitalWrite(LED_PIN, HIGH);
}

void turnLeft() {
  analogWrite(MOTOR_A_EN, motorSpeedA);
  analogWrite(MOTOR_B_EN, motorSpeedB);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);  // Motor A reverse
  digitalWrite(MOTOR_B_IN1, HIGH);  // Motor B forward
  digitalWrite(MOTOR_B_IN2, LOW);
}

void turnRight() {
  analogWrite(MOTOR_A_EN, motorSpeedA);
  analogWrite(MOTOR_B_EN, motorSpeedB);
  digitalWrite(MOTOR_A_IN1, HIGH);  // Motor A forward
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);   // Motor B reverse
  digitalWrite(MOTOR_B_IN2, HIGH);
}

void stopMotors() {
  analogWrite(MOTOR_A_EN, 0);
  analogWrite(MOTOR_B_EN, 0);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  digitalWrite(LED_PIN, LOW);
}

// ─── HC-SR04 Distance (cm) ───────────────────────────────────────────────────
long getDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // 25 ms timeout = ~4 m max range
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 25000UL);
  if (duration == 0) return -1;   // no echo / out of range
  return duration / 58L;          // microseconds → centimetres
}

// ─── Buzzer ───────────────────────────────────────────────────────────────────
void beep(int duration_ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, LOW);
}

// ─── MPU6050 ─────────────────────────────────────────────────────────────────
void mpu6050Init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_PWR_MGMT);
  Wire.write(0x00);  // Clear sleep bit — wake up sensor
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) {
    Serial.println(F("MPU6050 not found! Check A4/A5 wiring."));
  } else {
    Serial.println(F("MPU6050 OK"));
  }
  delay(100);
}

// Read raw accelerometer and gyroscope values
void mpu6050Read(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

  ax = ((int16_t)Wire.read() << 8) | Wire.read();
  ay = ((int16_t)Wire.read() << 8) | Wire.read();
  az = ((int16_t)Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Temperature — skip
  gx = ((int16_t)Wire.read() << 8) | Wire.read();
  gy = ((int16_t)Wire.read() << 8) | Wire.read();
  gz = ((int16_t)Wire.read() << 8) | Wire.read();
}
