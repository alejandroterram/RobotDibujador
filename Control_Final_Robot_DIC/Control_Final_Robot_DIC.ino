// === Robot dibujador 2DOF — Micro-rampa trapezoidal rápida ===
// NEMA23 (Motor 1) + DM556
// NEMA17 (Motor 2) + TB6600
// 6400 microsteps/rev
//
// Formato G-code (una línea):
//   G1 X<angulo1> A<angulo2> Y<vel1_deg_s> Z<vel2_deg_s>
//
// Ejemplo:
//   G1 X21.99 A10.69 Y35 Z35
//
// X = ángulo motor 1 (NEMA23) en grados
// A = ángulo motor 2 (NEMA17) en grados
// Y = velocidad motor 1 en grados/s
// Z = velocidad motor 2 en grados/s
//
// Control de pluma (relevador en D4):
//   M3 -> relevador ON (baja pluma)
//   M5 -> relevador OFF (sube pluma)

#include <math.h>
#include <stdlib.h>

// -------- PINES --------
// Motor 1 (NEMA23 + DM556)
const int STEP1_PIN = 7;
const int DIR1_PIN  = 6;
const int EN1_PIN   = 5;

// Motor 2 (NEMA17 + TB6600)
const int STEP2_PIN = 10;
const int DIR2_PIN  = 9;
const int EN2_PIN   = 8;

// Relevador (actuador pluma)
const int RELAY_PIN = 4;

// Si el NEMA17 está montado "de cabeza"
const bool INVERT_DIR_MOTOR2 = true;

// -------- PARÁMETROS DEL MOTOR --------
const float STEPS_PER_REV  = 6400.0;
const float DEG_TO_STEPS   = STEPS_PER_REV / 360.0; // ≈17.777 steps/°

float pos1_deg = 0.0;   // Posición actual (grados) motor 1
float pos2_deg = 0.0;   // Posición actual (grados) motor 2

// -------- LÍMITES Y RAMPA (OPCIÓN C: RÁPIDA) --------
const float MIN_SPEED_DEG_S   = 5.0;
const float MAX_SPEED_DEG_S   = 100.0;   // velocidad máxima permitida

// 0.10 = 10% acel, 10% freno, 80% crucero
const float ACCEL_FRAC        = 0.10;

// Velocidad mínima relativa en rampa (0.4 = 40% de la velocidad de crucero)
const float RAMP_MIN_SCALE    = 0.40;

// Intervalo mínimo entre pasos (µs).
const unsigned long MIN_INTERVAL_US = 150;

// -------- BUFFER DE LÍNEA G-CODE --------
const int LINE_BUF_SIZE = 80;
char lineBuf[LINE_BUF_SIZE];
int  linePos = 0;

// -------- PROTOTIPOS --------
void processLine(char *line);
void moveToAnglesTrapezoid(float target1_deg, float target2_deg,
                           float speed1_deg_s, float speed2_deg_s);

// ======================================================
void setup() {
  Serial.begin(115200);     // Debe coincidir con baudRate en MATLAB

  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN,  OUTPUT);
  pinMode(EN1_PIN,   OUTPUT);

  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN,  OUTPUT);
  pinMode(EN2_PIN,   OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);  // relevador apagado al inicio
  digitalWrite(STEP1_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);

  // Habilitar drivers (HIGH típico en DM556/TB6600)
  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, HIGH);

  Serial.println("Robot listo — Micro-rampa trapezoidal RAPIDA + control de pluma (M3/M5).");
}

// ======================================================
void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      // ignorar CR
      continue;
    }

    if (c == '\n') {
      // Fin de línea
      if (linePos > 0) {
        lineBuf[linePos] = '\0';
        processLine(lineBuf);
        linePos = 0;
      }
    } else {
      // Acumular en el buffer si hay espacio
      if (linePos < LINE_BUF_SIZE - 1) {
        lineBuf[linePos] = c;
        linePos++;
      }
    }
  }
}

// ======================================================
// Procesar una línea tipo:
//   G1 X.. A.. Y.. Z..
//   M3 / M5 (pluma)
// ======================================================
void processLine(char *line) {
  // --- Comandos M (pluma) ---
  if (line[0] == 'M' || line[0] == 'm') {
    int mnum = atoi(line + 1);

    if (mnum == 3) {
      // M3 -> pluma abajo (relevador ON)
      // Si tu módulo es activo bajo, invierte HIGH/LOW
      digitalWrite(RELAY_PIN, HIGH);
    } else if (mnum == 5) {
      // M5 -> pluma arriba (relevador OFF)
      digitalWrite(RELAY_PIN, LOW);
    }

    Serial.println("ok");
    return;
  }

  // --- Comando G1 (movimiento) ---
  float targetX = pos1_deg;
  float targetA = pos2_deg;
  float speedY  = 30.0;
  float speedZ  = 30.0;

  // Aceptamos solo G1
  if (line[0] != 'G' && line[0] != 'g') {
    Serial.println("ok");
    return;
  }

  int gnum = atoi(line + 1);
  if (gnum != 1) {
    Serial.println("ok");
    return;
  }

  // Parseo sencillo: buscar X, A, Y, Z
  char *p = line;
  while (*p != '\0') {
    if (*p == 'X' || *p == 'x') {
      targetX = atof(p + 1);
    } else if (*p == 'A' || *p == 'a') {
      targetA = atof(p + 1);
    } else if (*p == 'Y' || *p == 'y') {
      speedY = atof(p + 1);
    } else if (*p == 'Z' || *p == 'z') {
      speedZ = atof(p + 1);
    }
    p++;
  }

  moveToAnglesTrapezoid(targetX, targetA, speedY, speedZ);

  // Handshake para MATLAB / PC
  Serial.println("ok");
}

// ======================================================
// Movimiento con micro-rampa trapezoidal por segmento
// ======================================================
void moveToAnglesTrapezoid(float target1_deg, float target2_deg,
                           float speed1_deg_s, float speed2_deg_s) {
  float delta1 = target1_deg - pos1_deg;
  float delta2 = target2_deg - pos2_deg;

  long steps1 = (long)(fabs(delta1) * DEG_TO_STEPS + 0.5);
  long steps2 = (long)(fabs(delta2) * DEG_TO_STEPS + 0.5);

  if (steps1 == 0 && steps2 == 0) return;

  int dir1 = (delta1 >= 0.0) ? HIGH : LOW;
  int dir2 = (delta2 >= 0.0) ? HIGH : LOW;

  if (INVERT_DIR_MOTOR2) {
    dir2 = (dir2 == HIGH) ? LOW : HIGH;
  }

  digitalWrite(DIR1_PIN, dir1);
  digitalWrite(DIR2_PIN, dir2);
  delayMicroseconds(50);

  // Limitar velocidades a rangos razonables
  if (speed1_deg_s < MIN_SPEED_DEG_S) speed1_deg_s = MIN_SPEED_DEG_S;
  if (speed2_deg_s < MIN_SPEED_DEG_S) speed2_deg_s = MIN_SPEED_DEG_S;
  if (speed1_deg_s > MAX_SPEED_DEG_S) speed1_deg_s = MAX_SPEED_DEG_S;
  if (speed2_deg_s > MAX_SPEED_DEG_S) speed2_deg_s = MAX_SPEED_DEG_S;

  // Tiempo que tardaría cada motor si fuera solo
  float t1 = (steps1 > 0) ? (fabs(delta1) / speed1_deg_s) : 0.0;
  float t2 = (steps2 > 0) ? (fabs(delta2) / speed2_deg_s) : 0.0;

  // Queremos que terminen a la vez
  float moveTime = (t1 > t2) ? t1 : t2;
  if (moveTime <= 0.0) moveTime = 0.01;

  // Intervalos base (crucero) para cada motor
  unsigned long baseInt1_us =
      (steps1 > 0) ? (unsigned long)((moveTime / steps1) * 1e6) : 0;
  unsigned long baseInt2_us =
      (steps2 > 0) ? (unsigned long)((moveTime / steps2) * 1e6) : 0;

  if (baseInt1_us < MIN_INTERVAL_US && steps1 > 0) baseInt1_us = MIN_INTERVAL_US;
  if (baseInt2_us < MIN_INTERVAL_US && steps2 > 0) baseInt2_us = MIN_INTERVAL_US;

  unsigned long now   = micros();
  unsigned long next1 = now;
  unsigned long next2 = now;

  long done1 = 0, done2 = 0;

  // Para micro-rampa, medimos progreso global según el motor que más pasos dé
  long maxSteps = (steps1 > steps2) ? steps1 : steps2;
  if (maxSteps <= 0) maxSteps = 1;

  while (done1 < steps1 || done2 < steps2) {
    now = micros();

    long progressSteps = (done1 > done2) ? done1 : done2;
    if (progressSteps < 0)       progressSteps = 0;
    if (progressSteps > maxSteps) progressSteps = maxSteps;

    float p = (float)progressSteps / (float)maxSteps; // 0..1

    // ---- MICRO-RAMPA TRAPEZOIDAL ----
    float scale = 1.0;
    if (p < ACCEL_FRAC) {
      // fase de aceleración
      float f = p / ACCEL_FRAC;  // 0..1
      scale = RAMP_MIN_SCALE + (1.0 - RAMP_MIN_SCALE) * f;
    } else if (p > (1.0 - ACCEL_FRAC)) {
      // fase de frenado
      float f = (1.0 - p) / ACCEL_FRAC; // 0..1
      if (f < 0.0) f = 0.0;
      scale = RAMP_MIN_SCALE + (1.0 - RAMP_MIN_SCALE) * f;
    } else {
      // crucero
      scale = 1.0;
    }

    if (scale < RAMP_MIN_SCALE) scale = RAMP_MIN_SCALE;
    if (scale > 1.0)            scale = 1.0;

    unsigned long int1_us = baseInt1_us;
    unsigned long int2_us = baseInt2_us;

    if (steps1 > 0) {
      int1_us = (unsigned long)((float)baseInt1_us / scale);
      if (int1_us < MIN_INTERVAL_US) int1_us = MIN_INTERVAL_US;
    }
    if (steps2 > 0) {
      int2_us = (unsigned long)((float)baseInt2_us / scale);
      if (int2_us < MIN_INTERVAL_US) int2_us = MIN_INTERVAL_US;
    }

    // Motor 1
    if (done1 < steps1 && now >= next1) {
      digitalWrite(STEP1_PIN, HIGH);
      delayMicroseconds(3);
      digitalWrite(STEP1_PIN, LOW);
      done1++;
      next1 = now + int1_us;
    }

    // Motor 2
    if (done2 < steps2 && now >= next2) {
      digitalWrite(STEP2_PIN, HIGH);
      delayMicroseconds(3);
      digitalWrite(STEP2_PIN, LOW);
      done2++;
      next2 = now + int2_us;
    }
  }

  pos1_deg = target1_deg;
  pos2_deg = target2_deg;

  delay(2);   // pequeño colchón entre segmentos
}
