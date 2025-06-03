// Librerías:
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>

// Pines motores
#define DIR_PIN 15
#define STEP_PIN 2
#define DIR_PIN2 16
#define STEP_PIN2 4
#define ENAB_PIN 17
#define ENAB_PIN2 5

// Pines sensores infrarrojos
#define IR_LEFT 13
#define IR_RIGHT 12

// MPU6050
MPU6050 mpu;

// Filtro complementario
double accAngle = 0, gyroRate = 0, angle = 0;
unsigned long lastTime = 0;

// PID
double setpoint = 0.0, input, output;
double Kp = 14.0, Ki = 0.0, Kd = 0.8;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motores
int motorSpeed = 0;
unsigned long lastStepTime1 = 0, lastStepTime2 = 0;
volatile long stepCount1 = 0, stepCount2 = 0;

// Velocidad angular
unsigned long lastSpeedPrintTime = 0;
long lastStepCount1 = 0, lastStepCount2 = 0;

// Bluetooth
BluetoothSerial SerialBT;
char btCommand = '0';
char speedMode = 'l'; // 'l' = lento, 'r' = rápido

// Modo seguidor de línea
bool lineFollowMode = false;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Balanzinipagliaccini");
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 no conectado.");
    while (1);
  }

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(ENAB_PIN, OUTPUT);
  pinMode(ENAB_PIN2, OUTPUT);
  digitalWrite(ENAB_PIN, LOW);
  digitalWrite(ENAB_PIN2, LOW);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-15000, 15000);

  lastTime = millis();
}

void loop() {
  if (SerialBT.available()) {
    char received = SerialBT.read();

    if (received == 'l') {
      speedMode = 'l';
      SerialBT.println("Modo velocidad: LENTO");
    } else if (received == 'r') {
      speedMode = 'r';
      SerialBT.println("Modo velocidad: RÁPIDO");
    } else if (received == 'i') {
      lineFollowMode = !lineFollowMode;
      SerialBT.println(lineFollowMode ? "Modo: Seguir línea ACTIVADO" : "Modo: Seguir línea DESACTIVADO");
    } else {
      btCommand = received;
    }
  }

  if (lineFollowMode) {
    followLine();
  } else {
    motorControl();
  }

  btCommand = '0'; // Reset comando cada ciclo
}

void followLine() {
  int irLeft = digitalRead(IR_LEFT);
  int irRight = digitalRead(IR_RIGHT);

  int lineSpeed = (speedMode == 'l') ? 600 : 1200;

  if (irLeft == 0 && irRight == 0) {
    // Adelante
    stepMotors(lineSpeed, lineSpeed);
  } else if (irLeft == 1 && irRight == 0) {
    // Gira izquierda
    stepMotors(0, lineSpeed);
  } else if (irLeft == 0 && irRight == 1) {
    // Gira derecha
    stepMotors(lineSpeed, 0);
  } else {
    // Parar
    stepMotors(0, 0);
  }

  reportSpeed();
}

void motorControl() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0;
  lastTime = now;

  accAngle = atan2(ay, az) * 180.0 / PI;
  gyroRate = gx / 131.0;
  angle = 0.995 * (angle + gyroRate * dt) + 0.005 * accAngle;

  if (btCommand == 'w') angle = (speedMode == 'l') ? -6 : -12;
  if (btCommand == 's') angle = (speedMode == 'l') ? 6 : 12;

  if (btCommand == 'a') {
    motorSpeed = (speedMode == 'l') ? 800 : 1500;
    stepMotors(0, motorSpeed);
    reportSpeed();
    return;
  }

  if (btCommand == 'd') {
    motorSpeed = (speedMode == 'l') ? 800 : 1500;
    stepMotors(motorSpeed, 0);
    reportSpeed();
    return;
  }

  if (angle > 70.0 || angle < -70.0) {
    digitalWrite(ENAB_PIN, HIGH);
    digitalWrite(ENAB_PIN2, HIGH);
    SerialBT.println("¡Ángulo fuera de rango! Motores deshabilitados.");
    return;
  } else {
    digitalWrite(ENAB_PIN, LOW);
    digitalWrite(ENAB_PIN2, LOW);
  }

  if (abs(angle) < 0.5) {
    motorSpeed = 0;
    return;
  } else {
    input = angle;
  }

  myPID.Compute();
  motorSpeed = (int)output;

  if (abs(motorSpeed) < 50) {
    motorSpeed = 0;
    return;
  }

  stepMotors(motorSpeed, motorSpeed);
  reportSpeed();
}

void stepMotors(int speed1, int speed2) {
  unsigned long nowMicros = micros();

  if (speed1 != 0) {
    digitalWrite(DIR_PIN, speed1 > 0 ? HIGH : LOW);
    unsigned long interval1 = 1000000 / abs(speed1);
    if (nowMicros - lastStepTime1 >= interval1) {
      lastStepTime1 = nowMicros;
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      stepCount1++;
    }
  }

  if (speed2 != 0) {
    digitalWrite(DIR_PIN2, speed2 > 0 ? HIGH : LOW);
    unsigned long interval2 = 1000000 / abs(speed2);
    if (nowMicros - lastStepTime2 >= interval2) {
      lastStepTime2 = nowMicros;
      digitalWrite(STEP_PIN2, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN2, LOW);
      stepCount2++;
    }
  }
}

void reportSpeed() {
  if (millis() - lastSpeedPrintTime >= 100) {
    long deltaSteps1 = stepCount1 - lastStepCount1;
    long deltaSteps2 = stepCount2 - lastStepCount2;

    lastStepCount1 = stepCount1;
    lastStepCount2 = stepCount2;
    lastSpeedPrintTime = millis();

    double pps1 = deltaSteps1 / 0.1;
    double pps2 = deltaSteps2 / 0.1;

    double radPerSec1 = (pps1 * 2 * PI) / 200.0;
    double radPerSec2 = (pps2 * 2 * PI) / 200.0;

    double rpm1 = radPerSec1 * 60.0 / (2 * PI);
    double rpm2 = radPerSec2 * 60.0 / (2 * PI);

    SerialBT.print("angular1:"); SerialBT.println(radPerSec1, 2);
    SerialBT.print("angular2:"); SerialBT.println(radPerSec2, 2);
    SerialBT.print("RPM1:"); SerialBT.println(rpm1, 1);
    SerialBT.print("RPM2:"); SerialBT.println(rpm2, 1);
  }
}
