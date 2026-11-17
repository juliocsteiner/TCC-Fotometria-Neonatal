#include <Wire.h>
#include <Adafruit_AS7343.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define N_AMOSTRAS 100
#define INTERVALO_MS 200

Adafruit_AS7343 as7343;

// Lista dos canais que vamos analisar
const int NUM_CANAIS = 13;

uint8_t canais[NUM_CANAIS] = {
  AS7343_CHANNEL_F1,
  AS7343_CHANNEL_F2,
  AS7343_CHANNEL_FZ,
  AS7343_CHANNEL_F3,
  AS7343_CHANNEL_F4,
  AS7343_CHANNEL_F5,
  AS7343_CHANNEL_FY,
  AS7343_CHANNEL_FXL,
  AS7343_CHANNEL_F6,
  AS7343_CHANNEL_F7,
  AS7343_CHANNEL_F8,
  AS7343_CHANNEL_NIR,
  AS7343_CHANNEL_VIS_TL_0
};

const char* nomes[NUM_CANAIS] = {
  "F1 405nm",
  "F2 425nm",
  "FZ 450nm",
  "F3 475nm",
  "F4 515nm",
  "F5 550nm",
  "FY 555nm",
  "FXL 600nm",
  "F6 640nm",
  "F7 690nm",
  "F8 745nm",
  "NIR 855nm",
  "VIS Clear"
};

// Acumuladores
double soma[NUM_CANAIS];
double somaQuadrados[NUM_CANAIS];

uint16_t minimo[NUM_CANAIS];
uint16_t maximo[NUM_CANAIS];

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" TESTE 2 - LEITURA NO ESCURO - AS7343 ");
  Serial.println("======================================");
  Serial.println();

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!as7343.begin()) {
    Serial.println("[ERRO] AS7343 nao encontrado!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("[OK] Sensor detectado!");

  // MESMA CONFIGURACAO DO TESTE 1
  as7343.setGain(AS7343_GAIN_64X);
  as7343.setATIME(29);
  as7343.setASTEP(599);

  Serial.println();
  Serial.println("Configuracao:");
  Serial.println("GAIN  = 64X");
  Serial.println("ATIME = 29");
  Serial.println("ASTEP = 599");
  Serial.println();

  Serial.println("Cubra completamente o sensor.");
  Serial.println("Aguardando 5 segundos...");
  delay(5000);

  // Inicializa acumuladores
  for (int i = 0; i < NUM_CANAIS; i++) {
    soma[i] = 0;
    somaQuadrados[i] = 0;
    minimo[i] = 65535;
    maximo[i] = 0;
  }

  realizarTeste();

  Serial.println();
  Serial.println("Teste concluido.");
  Serial.println("Para repetir, reinicie o ESP32.");
}

void loop() {
  // Nada aqui
}

void realizarTeste() {

  uint16_t leituras[18];

  for (int amostra = 0; amostra < N_AMOSTRAS; amostra++) {

    if (!as7343.readAllChannels(leituras)) {
      Serial.print("[ERRO] Falha na leitura ");
      Serial.println(amostra + 1);
      delay(500);
      continue;
    }

    Serial.print("Amostra ");
    Serial.print(amostra + 1);
    Serial.print("/");
    Serial.println(N_AMOSTRAS);

    for (int i = 0; i < NUM_CANAIS; i++) {

      uint16_t valor = leituras[canais[i]];

      soma[i] += valor;
      somaQuadrados[i] += ((double)valor * valor);

      if (valor < minimo[i]) {
        minimo[i] = valor;
      }

      if (valor > maximo[i]) {
        maximo[i] = valor;
      }
    }

    delay(INTERVALO_MS);
  }

  Serial.println();
  Serial.println("======================================");
  Serial.println(" RESULTADOS DO TESTE NO ESCURO");
  Serial.println("======================================");
  Serial.println();

  Serial.println("Canal\tMedia\tDP\tMin\tMax");

  for (int i = 0; i < NUM_CANAIS; i++) {

    double media = soma[i] / N_AMOSTRAS;

    double variancia =
      (somaQuadrados[i] / N_AMOSTRAS)
      - (media * media);

    if (variancia < 0) {
      variancia = 0;
    }

    double desvioPadrao = sqrt(variancia);

    Serial.print(nomes[i]);
    Serial.print("\t");

    Serial.print(media, 2);
    Serial.print("\t");

    Serial.print(desvioPadrao, 2);
    Serial.print("\t");

    Serial.print(minimo[i]);
    Serial.print("\t");

    Serial.println(maximo[i]);
  }
}