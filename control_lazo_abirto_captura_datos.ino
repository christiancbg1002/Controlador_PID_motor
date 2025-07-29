#define PIN_PULSES 2
#define PIN_PWM    3
#define PIN_CTRL0  5
#define PIN_CTRL1  6

volatile int countPulses = 0;

unsigned long prevMillis = 0;
unsigned long t0 = 0;
float elapsed = 0;

float rpms = 0;
int   u    = 0;      // PWM inicial: motor detenido
// ref_rpm ya no se usa para envío, pero lo dejamos por claridad
float ref_rpm = 0;    

bool escalonAplicado = false;
bool started         = false;  // Espera comando "START" desde MATLAB

// --- Conversión de pulsos a RPM (muestreo = 50 ms) -----------------------
float convertP(int pulses) {
  return (float)pulses / 0.05 / 240.0 * 60.0;  // encoder: 240 pulsos/rev
}

// --- Interrupción para contar pulsos ------------------------------------
void pulses() {
  countPulses++;
}

void setup() {
  pinMode(PIN_PULSES, INPUT_PULLUP);
  pinMode(PIN_PWM,    OUTPUT);
  pinMode(PIN_CTRL0,  OUTPUT);
  pinMode(PIN_CTRL1,  OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_PULSES), pulses, RISING);

  Serial.begin(115200);

  digitalWrite(PIN_CTRL0, HIGH);  // Dirección fija
  digitalWrite(PIN_CTRL1, LOW);

  analogWrite(PIN_PWM, u);        // Motor detenido
}

void loop() {
  // Esperar comando START
  if (!started) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input == "START") {
        t0 = millis();
        prevMillis = t0;
        started = true;
      }
    }
    return;
  }

  // Tiempo transcurrido en segundos
  unsigned long now = millis();
  elapsed = (now - t0) / 1000.0;

  // Escalón a los 1.0 s
  if (elapsed >= 1.0 && !escalonAplicado) {
    ref_rpm = 300;          // solo informativo
    analogWrite(PIN_PWM, 255);
    u = 255;
    escalonAplicado = true;
  }

  // Muestreo y envío cada 50 ms
  if (now - prevMillis >= 50) {
    rpms = convertP(countPulses);
    countPulses = 0;
    prevMillis  = now;

    // --- ENVIAR SOLO tiempo y RPM ---
    Serial.print(elapsed, 3);
    Serial.print(',');
    Serial.println(rpms, 2);
  }
}
