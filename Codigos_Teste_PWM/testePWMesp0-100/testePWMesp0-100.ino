#define PWM_PIN 25

const int freq = 500;
const int resolution = 8;

void setup() {
  ledcAttach(PWM_PIN, freq, resolution);
}

void loop() {
  // 0%
  ledcWrite(PWM_PIN, 0);
  delay(3000);

  // 25%
  ledcWrite(PWM_PIN, 64);
  delay(3000);

  // 50%
  ledcWrite(PWM_PIN, 128);
  delay(3000);

  // 75%
  ledcWrite(PWM_PIN, 191);
  delay(3000);

  // 100%
  ledcWrite(PWM_PIN, 255);
  delay(3000);
}