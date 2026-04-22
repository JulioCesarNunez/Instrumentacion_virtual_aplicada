#include <Arduino.h>

// ===================== CONFIGURACIÓN ===================== //
const int sen1    = 9;
const int sen2    = 10;
const int LED_PIN = 2;

float u    = 0;
float usat = 0;
float PWM  = 0;

// Zona muerta sobre u  →  deadband_u = Kp × 4°
// Ajusta este valor según tu Kp en LabVIEW
// Ejemplo: si Kp = 0.5  →  deadband = 0.5 × 4 = 2.0
const float deadband = 2.0f;

String received;

// ===================== SETUP ===================== //
void setup() {
  Serial.begin(115200);

  pinMode(sen1,    OUTPUT);
  pinMode(sen2,    OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  analogWrite(sen1, 0);
  analogWrite(sen2, 0);
}

// ===================== LOOP ===================== //
void loop() {
  digitalWrite(LED_PIN, LOW);

  // -------- RECIBIR u DESDE LABVIEW -------- //
  if (Serial.available() > 0) {
    received = Serial.readStringUntil('\n');
    u = received.toFloat();         // LabVIEW manda u en ±6 V

    // Saturación de seguridad ±6 V
    usat = constrain(u, -6.0f, 6.0f);

    // -------- ZONA MUERTA DE 4° -------- //
    if (abs(usat) < deadband) {
      // Error dentro de ±4°: detener motor
      analogWrite(sen1, 0);
      analogWrite(sen2, 0);
    } else {
      // Conversión a PWM: 6 V → 255  (255/6 = 42.5)
      PWM = usat * 42.5f;

      // Aplicar PWM al puente H
      if (PWM >= 0) {
        analogWrite(sen1, (int) PWM);
        analogWrite(sen2, 0);
      } else {
        analogWrite(sen1, 0);
        analogWrite(sen2, (int)(-PWM));
      }
    }
  }
}
