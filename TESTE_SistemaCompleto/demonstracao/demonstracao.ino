#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define TCA_ADDR 0x71

bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  return Wire.endTransmission() == 0;
}

void scanI2C() {
  Serial.println("\n--- Barramento principal ---");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0) {
      Serial.print("Encontrado: 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

void scanChannel(uint8_t channel) {

  Serial.print("\n--- TCA canal ");
  Serial.print(channel);
  Serial.println(" ---");

  if (!tcaSelect(channel)) {
    Serial.println("ERRO: TCA nao respondeu.");
    return;
  }

  delay(10);

  bool found = false;

  for (uint8_t address = 1; address < 127; address++) {

    // Ignora o proprio TCA
    if (address == TCA_ADDR)
      continue;

    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0) {

      found = true;

      Serial.print("Encontrado: 0x");

      if (address < 16)
        Serial.print("0");

      Serial.println(address, HEX);
    }
  }

  if (!found)
    Serial.println("Nenhum dispositivo encontrado.");
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("\n============================");
  Serial.println(" TESTE I2C / TCA9548A");
  Serial.println("============================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Primeiro verifica o barramento principal
  scanI2C();

  // Depois verifica exatamente os canais usados
  scanChannel(7);
  scanChannel(5);
  scanChannel(3);

  Serial.println("\nTeste concluido.");
}

void loop() {
}