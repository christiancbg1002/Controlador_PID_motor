#define PIN_PULSES 2
#define PIN_PWM    3
#define PIN_CTRL0  5
#define PIN_CTRL1  6
#define PIN_POT    A0   // Potenciómetro conectado a A0

volatile int countPulses = 0;

unsigned long prevMillis = 0;
unsigned long t0 = 0;
float elapsed = 0;

// --- Controlador PID (Incremental) ---
float Kp = 2.1931;
float Ki = 16.2837;
float Kd = 0.0237755;
float dt = 0.05; // 50 ms

float a0 = Kp + Ki * dt + Kd / dt;
float a1 = -Kp - 2 * Kd / dt;
float a2 = Kd / dt;

float e[3] = {0, 0, 0};  // e[k], e[k-1], e[k-2]
int u = 0;               // PWM inicial
float rpms = 0;
float ref_rpm = 0;       // Referencia leída desde potenciómetro

bool started = false;

// --- Conversión de pulsos a RPM ---
float convertP(int pulses) {
  return (float)pulses / dt / 240.0 * 60.0;  // encoder: 240 pulsos/rev
}

// --- ISR para contar pulsos ---
void pulses() {
  countPulses++;
}

void setup() {
  pinMode(PIN_PULSES, INPUT_PULLUP);
  pinMode(PIN_PWM,    OUTPUT);
  pinMode(PIN_CTRL0,  OUTPUT);
  pinMode(PIN_CTRL1,  OUTPUT);
  pinMode(PIN_POT,    INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_PULSES), pulses, RISING);

  Serial.begin(115200);

  digitalWrite(PIN_CTRL0, HIGH);
  digitalWrite(PIN_CTRL1, LOW);
  analogWrite(PIN_PWM, u);
}

void loop() {
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

  unsigned long now = millis();
  elapsed = (now - t0) / 1000.0;

  if (now - prevMillis >= 50) {
    prevMillis = now;

    // Leer potenciómetro y mapear a 0–300 RPM
    int raw_pot = analogRead(PIN_POT);
    ref_rpm = map(raw_pot, 0, 1023, 0, 300);

    // Convertir pulsos a RPM
    rpms = convertP(countPulses);
    countPulses = 0;

    // PID incremental
    e[2] = e[1];
    e[1] = e[0];
    e[0] = ref_rpm - rpms;

    float du = a0 * e[0] + a1 * e[1] + a2 * e[2];
    u += (int)du;

    // Saturación del PWM
    u = constrain(u, 0, 255);
    analogWrite(PIN_PWM, u);

    // Enviar tiempo, RPM, referencia y error
    Serial.print(elapsed, 3);
    Serial.print(',');
    Serial.print(rpms, 2);
    Serial.print(',');
    Serial.print(ref_rpm, 2);
    Serial.print(',');
    Serial.println(e[0], 2);  // Error actual
  }
}
