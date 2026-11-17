#include <Wire.h>
#include <Adafruit_AS7343.h>

// ===============================
// CONFIGURAÇÕES DO ESP32
// ===============================
#define I2C_SDA 21
#define I2C_SCL 22

// ===============================
// OBJETO DO SENSOR
// ===============================
Adafruit_AS7343 as7343;

// Contador de leituras
unsigned long contador = 0;

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" TESTE FUNCIONAL INDIVIDUAL - AS7343 ");
  Serial.println("======================================");
  Serial.println();

  // Inicializa I2C do ESP32
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Inicializando sensor AS7343...");

  // Tenta encontrar/inicializar o sensor
  if (!as7343.begin()) {

    Serial.println();
    Serial.println("[ERRO] AS7343 nao encontrado!");
    Serial.println("Verifique:");
    Serial.println("- VIN / 3V3");
    Serial.println("- GND");
    Serial.println("- SDA -> GPIO 21");
    Serial.println("- SCL -> GPIO 22");
    Serial.println();

    while (1) {
      delay(1000);
    }
  }

  Serial.println("[OK] AS7343 detectado!");
  Serial.println();

  // ==================================
  // CONFIGURAÇÃO INICIAL DO SENSOR
  // ==================================

  // Ganho moderado para primeiro teste
  as7343.setGain(AS7343_GAIN_64X);

  // Tempo de integração
  as7343.setATIME(29);
  as7343.setASTEP(599);

  Serial.print("Tempo de integracao: ");
  Serial.print(as7343.getIntegrationTime());
  Serial.println(" ms");

  Serial.println();
  Serial.println("Sensor pronto.");
  Serial.println("Aproxime e afaste uma fonte luminosa");
  Serial.println("e observe a variacao das leituras.");
  Serial.println();

  delay(1000);
}


void loop() {

  uint16_t leituras[18];

  // ==================================
  // REALIZA A LEITURA
  // ==================================

  if (!as7343.readAllChannels(leituras)) {

    Serial.println("[ERRO] Falha na leitura do AS7343!");

    delay(500);

    return;
  }

  contador++;

  // ==================================
  // CABEÇALHO
  // ==================================

  Serial.println("--------------------------------------");

  Serial.print("Leitura numero: ");
  Serial.println(contador);

  Serial.println();

  // ==================================
  // CANAIS ESPECTRAIS
  // ==================================

  Serial.print("F1  - 405 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F1]);

  Serial.print("F2  - 425 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F2]);

  Serial.print("FZ  - 450 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_FZ]);

  Serial.print("F3  - 475 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F3]);

  Serial.print("F4  - 515 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F4]);

  Serial.print("F5  - 550 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F5]);

  Serial.print("FY  - 555 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_FY]);

  Serial.print("FXL - 600 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_FXL]);

  Serial.print("F6  - 640 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F6]);

  Serial.print("F7  - 690 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F7]);

  Serial.print("F8  - 745 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_F8]);

  Serial.print("NIR - 855 nm : ");
  Serial.println(leituras[AS7343_CHANNEL_NIR]);

  // Canal de luz visível / clear
  Serial.print("VIS - Clear  : ");
  Serial.println(leituras[AS7343_CHANNEL_VIS_TL_0]);

  Serial.println();

  // Uma leitura por segundo
  delay(5000);
}