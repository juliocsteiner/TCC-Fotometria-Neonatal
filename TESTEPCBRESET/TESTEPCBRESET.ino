#include <Wire.h>

#define ENDERECO_ALVO 0x70
#define SDA_PIN 17     // TX2
#define SCL_PIN 16     // RX2
#define PINO_RESET 4 // Pino do ESP32 ligado ao pino RST/Reset do módulo

void executarReset() {
  Serial.println("\n[!] Executando reset do módulo e do barramento I2C...");
  
  // 1. Reset físico do módulo (pulso HIGH para LOW)
  pinMode(PINO_RESET, OUTPUT);
  digitalWrite(PINO_RESET, HIGH);
  delay(5000); // 
  digitalWrite(PINO_RESET,LOW );
  delay(50); // 
  
  // 2. Reset lógico do barramento I2C do ESP32
  Wire.end();   // Desliga a comunicação I2C atual
  Wire.begin(SDA_PIN, SCL_PIN); // Reinicia a comunicação I2C (SDA=21, SCL=22)
}

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  
  Serial.println("\nIniciando sistema...");
  executarReset(); // Realiza um reset de segurança logo ao ligar
}

void loop() {
  Serial.print("Buscando dispositivo no endereço 0x70... ");

  Wire.beginTransmission(ENDERECO_ALVO);
  byte erro = Wire.endTransmission();

  if (erro == 0) {
    Serial.println("SUCESSO: Dispositivo encontrado!");
  }
  else {
    Serial.println("FALHA: Dispositivo não respondeu.");
    // Se o dispositivo falhar, executa a rotina de reset para tentar destravá-lo
    executarReset(); 
  }

  delay(3000); // Aguarda 3 segundos antes do próximo teste
  
  executarReset();
  }