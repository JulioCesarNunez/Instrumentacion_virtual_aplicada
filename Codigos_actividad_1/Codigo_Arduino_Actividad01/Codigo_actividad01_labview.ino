#include <Arduino.h>

/*
  ===================== CABLEADO (ARDUINO UNO) =====================
  Encoder incremental:
  - VCC  -> 5V
  - GND  -> GND
  - CH A -> D2 (INT0)
  - CH B -> D3 (INT1)

  LM35 (cara plana al frente):
  - Pin izquierdo  (+Vs) -> 5V
  - Pin central    (Vout)-> A0
  - Pin derecho    (GND) -> GND

  Nota:
  - Todas las tierras deben estar en comun (GND).
*/

// Pines para Arduino Uno
const uint8_t ENCODER_A_PIN = 2;   // INT0
const uint8_t ENCODER_B_PIN = 3;   // INT1
const uint8_t LM35_PIN = A1;

volatile long contador = 0;

// ===================== INTERRUPCIONES DEL ENCODER =====================
void CH_A() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);

  if (a == b) contador++;
  else contador--;
}

void CH_B() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);

  if (a != b) contador++;
  else contador--;
}

void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), CH_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), CH_B, CHANGE);
}

void loop() {
  noInterrupts();
  long contadorLocal = contador;
  interrupts();

  int raw = analogRead(LM35_PIN);
  float voltaje = raw * (5.0f / 1023.0f);  // ADC Uno (0-5V)
  float temperaturaC = voltaje * 100.0f;   // LM35: 10mV/°C

  // Formato solicitado: <12.23,110>
  Serial.print('<');
  Serial.print(temperaturaC, 2);
  Serial.print(',');
  Serial.print(contadorLocal);
  Serial.println('>');

  delay(1000);
}
