#define PWM_PIN 25

const int PWM_FREQ = 500;       // 500 Hz
const int PWM_RESOLUTION = 8;   // 8 bits -> 0 a 255

void setup() {
  Serial.begin(115200);

  // Configura PWM no ESP32 (API atual)
  if (!ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("ERRO ao configurar PWM.");
    while (true);
  }

  // Começa desligado
  ledcWrite(PWM_PIN, 0);

  Serial.println();
  Serial.println("=== TESTE PWM DO DRIVER ===");
  Serial.println("Digite um duty cycle:");
  Serial.println("0   -> 0%");
  Serial.println("25  -> 25%");
  Serial.println("50  -> 50%");
  Serial.println("75  -> 75%");
  Serial.println("100 -> 100%");
  Serial.println();
}

void loop() {

  if (Serial.available()) {

    int dutyPercent = Serial.parseInt();

    // Limpa caracteres restantes
    while (Serial.available()) {
      Serial.read();
    }

    if (dutyPercent >= 0 && dutyPercent <= 100) {

      int dutyValue = round((dutyPercent / 100.0) * 255.0);

      ledcWrite(PWM_PIN, dutyValue);

      Serial.print("Duty configurado: ");
      Serial.print(dutyPercent);
      Serial.print("%  |  Valor PWM: ");
      Serial.println(dutyValue);

    } else {
      Serial.println("Digite apenas valores entre 0 e 100.");
    }
  }
}