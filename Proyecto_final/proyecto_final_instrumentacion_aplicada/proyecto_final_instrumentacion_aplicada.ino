#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// ---------------------------------------------------------
// CREDENCIALES
// ---------------------------------------------------------
#define WIFI_SSID       "Sernalap"
#define WIFI_PASSWORD   "6#0S5k52"
#define DATABASE_SECRET "xlKoOi0MWauOGa7AuI24mf46kd9ua5DR9Ghvbkpd"
#define DATABASE_URL    "iva-jcnq-default-rtdb.firebaseio.com"


// ---------------------------------------------------------
// PINES
// ---------------------------------------------------------
const int pinEncA = 18;
const int pinEncB = 19;
const int pinENA  = 25;   // PWM
const int pinIN1  = 26;   // Dirección
const int pinIN2  = 27;   // Dirección

const int pwmFrecuencia = 5000;
const int pwmResolucion = 8;

// ---------------------------------------------------------
// VARIABLES COMPARTIDAS ENTRE NÚCLEOS (volatile)
// Core 1 escribe th_send, Core 0 la envía a Firebase
// Core 0 escribe las ganancias, Core 1 las lee para el PID
// ---------------------------------------------------------
volatile float th_send = 0.0f;   // Posición → Firebase
volatile float th_des  = 0.0f;   // SetPoint ← Firebase
volatile float kp_fb   = 1.25f;  // Ganancia Kp ← Firebase
volatile float ki_fb   = 0.07f;  // Ganancia Ki ← Firebase
volatile float kd_fb   = 0.25f;  // Ganancia Kd ← Firebase
volatile int   Np      = 0;      // Pulsos del encoder (ISR)
volatile int   heartbeat = 0;

// ---------------------------------------------------------
// OBJETOS FIREBASE
// ---------------------------------------------------------
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ---------------------------------------------------------
// INTERRUPCIONES DEL ENCODER (IRAM = ejecución ultrarrápida)
// ---------------------------------------------------------
void IRAM_ATTR CH_A() {
  if (digitalRead(pinEncB) == LOW) Np++;
  else                              Np--;
}

void IRAM_ATTR CH_B() {
  if (digitalRead(pinEncA) == HIGH) Np++;
  else                               Np--;
}

// ---------------------------------------------------------
// TAREA FIREBASE — Corre en Core 0 (sin tocar el PID)
// ---------------------------------------------------------
void tareaFirebase(void *pvParameters) {
  for (;;) {
    if (Firebase.ready()) {

      // LEER ← /Variables
      if (Firebase.getFloat(fbdo, "/Variables/Kp"))       kp_fb  = fbdo.floatData();
      if (Firebase.getFloat(fbdo, "/Variables/Ki"))       ki_fb  = fbdo.floatData();
      if (Firebase.getFloat(fbdo, "/Variables/Kd"))       kd_fb  = fbdo.floatData();
      if (Firebase.getFloat(fbdo, "/Variables/SetPoint")) th_des = fbdo.floatData();

      // ENVIAR → /Estado
      heartbeat++;
      if (heartbeat > 10000) heartbeat = 0;

      FirebaseJson jsonEstado;
      jsonEstado.set("Posicion",  th_send);
      jsonEstado.set("Heartbeat", heartbeat);
      Firebase.setJSON(fbdo, "/Estado", jsonEstado);
    }

    // Esperar 300ms antes de la próxima sincronización
    // vTaskDelay NO bloquea el Core 1 (el PID sigue corriendo)
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  ledcAttach(pinENA, pwmFrecuencia, pwmResolucion);

  // Encoder
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncA), CH_A, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncB), CH_B, RISING);

  // WiFi
  Serial.print("Conectando a WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi conectado! IP: " + WiFi.localIP().toString());

  // Firebase
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("✅ Firebase inicializado");

  // Lanzar la tarea de Firebase en el Core 0
  xTaskCreatePinnedToCore(
    tareaFirebase,  // Función de la tarea
    "Firebase",     // Nombre (para debug)
    8192,           // Stack en bytes (Firebase necesita bastante)
    NULL,           // Parámetros
    1,              // Prioridad (1 = baja)
    NULL,           // Handle (no necesitamos referencia)
    0               // ← CORE 0
  );
}

// ---------------------------------------------------------
// LOOP — Control PID en Core 1 (cada 2ms exactos)
// ---------------------------------------------------------
void loop() {
  // Parámetros del muestreo
  const int   dt_us = 2000;
  const float dt    = dt_us * 0.000001f;
  //  Fix: la versión original usaba enteros → 360/11/130 = 0
  //         Aquí se usa float correctamente
  const float R     = 360.0f / 7.0f / 140.0f;
  const float alpha = 0.05f;

  // Variables de estado (static = persisten entre ciclos)
  static float thp   = 0.0f;
  static float dth_f = 0.0f;
  static float inte  = 0.0f;

  unsigned long t1 = micros();

  // Copiar ganancias del Core 0 (lectura atómica en ESP32)
  float kp = kp_fb;
  float ki = ki_fb;
  float kd = kd_fb;
  float setpoint = th_des;

  // --- Posición y velocidad ---
  float th    = R * Np;
  float dth_d = (th - thp) / dt;
  dth_f = alpha * dth_d + (1.0f - alpha) * dth_f;  // Filtro pasa-bajas

  // --- Error PID ---
  float e = setpoint - th;
  float de = -dth_f;          // Derivada del error
  inte += e * dt;              // Integral del error

  // Anti-windup
  if (inte >  255.0f) inte =  255.0f;
  if (inte < -255.0f) inte = -255.0f;

  // --- Ley de control ---
  float u    = kp * e + kd * de + ki * inte;
  float usat = constrain(u, -12.0f, 12.0f);  // Saturación en voltios
  float pwm  = usat * 21.25f;                 // 255 / 12V = 21.25

  // --- Comando al puente H ---
  if (pwm > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
    ledcWrite(pinENA, (int)pwm);
  } else if (pwm < 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
    ledcWrite(pinENA, (int)(-pwm));
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    ledcWrite(pinENA, 0);
  }

  // Actualizar estado
  thp     = th;
  th_send = th;  // Compartir posición con la tarea Firebase

  // Debug por Serial (para graficar en Arduino IDE)
  Serial.print(th);
  Serial.print(',');
  Serial.print((float)ki_fb, 4);
  Serial.print(',');
  Serial.print((float)kd_fb, 4);
  Serial.print(',');
  Serial.print((float)kp_fb, 4);
  Serial.print(',');
  Serial.println(pwm);

  // Esperar hasta completar exactamente los 2ms del ciclo
  unsigned long t2 = micros();
  while ((t2 - t1) < (unsigned long)dt_us) {
    t2 = micros();
  }
}
