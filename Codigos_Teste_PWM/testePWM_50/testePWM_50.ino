#define PWM_PIN 25

void setup() {
  ledcAttach(PWM_PIN, 500, 8);
  ledcWrite(PWM_PIN, 128);
}

void loop() {
}