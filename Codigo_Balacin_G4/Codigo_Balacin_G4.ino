
// Librerías utilizadas:
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>

// Definición de pines para control de motores paso a paso
// Motor 1
#define DIR_PIN 15     // Pin de dirección
#define STEP_PIN 2     // Pin de paso
#define ENAB_PIN 17    // Pin de habilitación
// Motor 2
#define DIR_PIN2 16
#define STEP_PIN2 4
#define ENAB_PIN2 5

// Pines para sensores infrarrojos de seguimiento de línea
#define IR_LEFT 13     // Sensor izquierdo
#define IR_RIGHT 12    // Sensor derecho

// Objeto para el sensor MPU6050 (acelerómetro + giroscopio)
MPU6050 mpu;

// Variables para el filtro complementario
double accAngle = 0;    // Ángulo calculado del acelerómetro
double gyroRate = 0;    // Tasa de giro del giroscopio
double angle = 0;       // Ángulo filtrado final
unsigned long lastTime = 0; // Último tiempo de medición

// Variables para el control PID
double setpoint = 0.0;  // Punto de referencia (equilibrio)
double input, output;   // Entrada y salida del PID
// Constantes del controlador PID (Proporcional, Integral, Derivativo)
double Kp = 14.0, Ki = 0.0, Kd = 0.8;
// Objeto PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Variables para control de motores
int motorSpeed = 0;     // Velocidad actual de los motores
unsigned long lastStepTime1 = 0, lastStepTime2 = 0; // Tiempos del último paso
volatile long stepCount1 = 0, stepCount2 = 0; // Contadores de pasos

// Variables para cálculo de velocidad
unsigned long lastSpeedPrintTime = 0; // Último tiempo de reporte
long lastStepCount1 = 0, lastStepCount2 = 0; // Pasos anteriores

// Configuración Bluetooth
BluetoothSerial SerialBT; // Objeto Bluetooth
char btCommand = '0';     // Comando recibido por Bluetooth
char speedMode = 'l';     // Modo de velocidad ('l' = lento, 'r' = rápido)

// Modo de operación
bool lineFollowMode = false; // false = modo balancín, true = modo seguidor de línea

void setup() {
  // Inicialización comunicación serial
  Serial.begin(115200);
  
  // Configuración Bluetooth
  SerialBT.begin("Balanzinipagliaccini"); // Nombre del dispositivo BT
  
  // Inicialización comunicación I2C
  Wire.begin();

  // Inicialización MPU6050
  mpu.initialize();
  // Verificación de conexión del sensor
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 no conectado.");
    while (1); // Bucle infinito si no hay sensor
  }

  // Configuración de pines de motores como salidas
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(ENAB_PIN, OUTPUT);
  pinMode(ENAB_PIN2, OUTPUT);
  
  // Deshabilitar motores inicialmente (LOW = habilitado)
  digitalWrite(ENAB_PIN, LOW);
  digitalWrite(ENAB_PIN2, LOW);

  // Configuración de pines de sensores IR como entradas
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Configuración del controlador PID
  myPID.SetMode(AUTOMATIC);      // Modo automático
  myPID.SetOutputLimits(-15000, 15000); // Límites de salida

  // Inicialización del tiempo de referencia
  lastTime = millis();
}

void loop() {
  // Comprobación de comandos Bluetooth entrantes
  if (SerialBT.available()) {
    char received = SerialBT.read();

    // Cambio de modo de velocidad
    if (received == 'l') {
      speedMode = 'l';
      SerialBT.println("Modo velocidad: LENTO");
    } else if (received == 'r') {
      speedMode = 'r';
      SerialBT.println("Modo velocidad: RÁPIDO");
    } 
    // Cambio entre modo balancín y seguidor de línea
    else if (received == 'i') {
      lineFollowMode = !lineFollowMode;
      SerialBT.println(lineFollowMode ? "Modo: Seguir línea ACTIVADO" : "Modo: Seguir línea DESACTIVADO");
    } 
    // Comandos de movimiento
    else {
      btCommand = received;
    }
  }

  // Selección del modo de operación
  if (lineFollowMode) {
    followLine(); // Ejecuta modo seguidor de línea
  } else {
    motorControl(); // Ejecuta modo balancín
  }

  // Reinicio del comando BT para el próximo ciclo
  btCommand = '0';
}

void followLine() {
  // Lectura de sensores infrarrojos
  // 0 = detecta línea (blanco), 1 = no detecta (negro)
  int irLeft = digitalRead(IR_LEFT);
  int irRight = digitalRead(IR_RIGHT);

  // Configuración de velocidad según modo
  int lineSpeed = (speedMode == 'l') ? 600 : 1200; // Pasos por segundo

  // Lógica de seguimiento de línea
  if (irLeft == 0 && irRight == 0) {
    // Ambos sensores sobre la línea - avanza recto
    stepMotors(lineSpeed, lineSpeed);
  } 
  else if (irLeft == 1 && irRight == 0) {
    // Solo sensor izquierdo fuera - gira a izquierda
    stepMotors(0, lineSpeed);
  } 
  else if (irLeft == 0 && irRight == 1) {
    // Solo sensor derecho fuera - gira a derecha
    stepMotors(lineSpeed, 0);
  } 
  else {
    // Ambos sensores fuera - detiene motores
    stepMotors(0, 0);
  }

  // Reporte de velocidad actual
  reportSpeed();
}

void motorControl() {
  // Lectura de datos del MPU6050 (6 ejes)
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Cálculo del tiempo transcurrido desde última lectura
  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0; // Convertir a segundos
  lastTime = now;

  // Cálculo de ángulo con acelerómetro (en grados)
  accAngle = atan2(ay, az) * 180.0 / PI;
  
  // Cálculo de tasa de giro con giroscopio (en grados/segundo)
  gyroRate = gx / 131.0;
  
  // Filtro complementario para combinar ambas mediciones
  angle = 0.995 * (angle + gyroRate * dt) + 0.005 * accAngle;

  // Comandos manuales por Bluetooth
  if (btCommand == 'w') angle = (speedMode == 'l') ? -6 : -12; // Inclinación adelante
  if (btCommand == 's') angle = (speedMode == 'l') ? 6 : 12;  // Inclinación atrás

  // Comandos de giro manual
  if (btCommand == 'a') {
    motorSpeed = (speedMode == 'l') ? 800 : 1500;
    stepMotors(0, motorSpeed); // Giro izquierdo
    reportSpeed();
    return;
  }

  if (btCommand == 'd') {
    motorSpeed = (speedMode == 'l') ? 800 : 1500;
    stepMotors(motorSpeed, 0); // Giro derecho
    reportSpeed();
    return;
  }

  // Protección por inclinación excesiva
  if (angle > 70.0 || angle < -70.0) {
    digitalWrite(ENAB_PIN, HIGH);  // Deshabilita motores
    digitalWrite(ENAB_PIN2, HIGH);
    SerialBT.println("¡Ángulo fuera de rango! Motores deshabilitados.");
    return;
  } else {
    digitalWrite(ENAB_PIN, LOW);   // Habilita motores
    digitalWrite(ENAB_PIN2, LOW);
  }

  // Zona muerta para pequeños ángulos
  if (abs(angle) < 0.5) {
    motorSpeed = 0;
    return;
  } else {
    input = angle; // Establece el ángulo como entrada al PID
  }

  // Cálculo del control PID
  myPID.Compute();
  motorSpeed = (int)output; // Convierte salida a velocidad

  // Aplicación de umbral mínimo de velocidad
  if (abs(motorSpeed) < 50) {
    motorSpeed = 0;
    return;
  }

  // Aplicación de velocidad a motores
  stepMotors(motorSpeed, motorSpeed);
  
  // Reporte de velocidad

  repo<rtSpeed();
}

void stepMotors(int speed1, int speed2) {
  unsigned long nowMicros = micros(); // Tiempo actual en microsegundos

  // Control motor 1
  if (speed1 != 0) {
    // Establece dirección según signo de velocidad
    digitalWrite(DIR_PIN, speed1 > 0 ? HIGH : LOW);
    
    // Calcula intervalo entre pasos (inversamente proporcional a velocidad)
    unsigned long interval1 = 1000000 / abs(speed1);
    
    // Genera pulso si ha pasado el tiempo necesario
    if (nowMicros - lastStepTime1 >= interval1) {
      lastStepTime1 = nowMicros;
      digitalWrite(STEP_PIN, HIGH); // Pulso alto
      delayMicroseconds(2);         // Pequeña espera
      digitalWrite(STEP_PIN, LOW);  // Pulso bajo
      stepCount1++;                 // Incrementa contador
    }
  }

  // Control motor 2 (similar al motor 1)
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
  // Reporta cada 100ms
  if (millis() - lastSpeedPrintTime >= 100) {
    // Calcula pasos desde último reporte
    long deltaSteps1 = stepCount1 - lastStepCount1;
    long deltaSteps2 = stepCount2 - lastStepCount2;

    // Actualiza referencias
    lastStepCount1 = stepCount1;
    lastStepCount2 = stepCount2;
    lastSpeedPrintTime = millis();

    // Calcula pasos por segundo
    double pps1 = deltaSteps1 / 0.1; // 0.1s = 100ms
    double pps2 = deltaSteps2 / 0.1;

    // Conversión a radianes/segundo (200 pasos por revolución)
    double radPerSec1 = (pps1 * 2 * PI) / 200.0;
    double radPerSec2 = (pps2 * 2 * PI) / 200.0;

    // Conversión a RPM
    double rpm1 = radPerSec1 * 60.0 / (2 * PI);
    double rpm2 = radPerSec2 * 60.0 / (2 * PI);

    // Envío de datos por Bluetooth
    SerialBT.print("angular1:"); SerialBT.println(radPerSec1, 2);
    SerialBT.print("angular2:"); SerialBT.println(radPerSec2, 2);
    SerialBT.print("RPM1:"); SerialBT.println(rpm1, 1);
    SerialBT.print("RPM2:"); SerialBT.println(rpm2, 1);
  }
}
