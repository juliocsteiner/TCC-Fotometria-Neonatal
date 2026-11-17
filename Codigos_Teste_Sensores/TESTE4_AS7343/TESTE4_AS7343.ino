#include <Wire.h>
#include <Adafruit_AS7343.h>
#include <math.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define PWM_PIN 25
#define PWM_FREQ 500
#define PWM_RESOLUTION 8

#define N_AMOSTRAS 50
#define INTERVALO_MS 200
#define TEMPO_ESTABILIZACAO_MS 5000

Adafruit_AS7343 as7343;

// ===============================
// CANAIS
// ===============================

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

// ===============================
// PONTOS DE PWM
// ===============================

const int NUM_PWM = 5;

int pwmPercent[NUM_PWM] = {
  0, 25, 50, 75, 100
};

// resultados
double mediaResultado[NUM_PWM][NUM_CANAIS];
double dpResultado[NUM_PWM][NUM_CANAIS];

// ===============================
// SETUP
// ===============================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" TESTE 4 - RESPOSTA A INTENSIDADE");
  Serial.println(" AS7343 + MALHA DE LEDS");
  Serial.println("======================================");
  Serial.println();

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!as7343.begin()) {
    Serial.println("[ERRO] AS7343 nao encontrado!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[OK] AS7343 detectado.");

  // Mesma configuração dos testes anteriores
  as7343.setGain(AS7343_GAIN_32X);
  as7343.setATIME(29);
  as7343.setASTEP(599);

  Serial.println("[OK] Sensor configurado:");
  Serial.println("GAIN  = 64X");
  Serial.println("ATIME = 29");
  Serial.println("ASTEP = 599");
  Serial.println();

  // PWM
  if (!ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("[ERRO] Falha ao configurar PWM.");
    while (true) {
      delay(1000);
    }
  }

  ledcWrite(PWM_PIN, 0);

  Serial.println("IMPORTANTE:");
  Serial.println("- Nao mova o sensor");
  Serial.println("- Nao altere a distancia");
  Serial.println("- Nao mexa na malha durante o ensaio");
  Serial.println();

  Serial.println("Teste iniciara em 5 segundos...");
  delay(5000);

  executarTesteCompleto();

  // desliga a malha ao final
  ledcWrite(PWM_PIN, 0);

  Serial.println();
  Serial.println("Teste finalizado.");
  Serial.println("PWM retornado para 0%.");

    Serial.println();
  Serial.println("Digite T para iniciar o Teste 4.");
}

void loop() {
  
  if (Serial.available()) {

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.equalsIgnoreCase("t")) {

      Serial.println();
      Serial.println("Iniciando Teste 4...");

      executarTesteCompleto();

      // desliga a malha ao final
      ledcWrite(PWM_PIN, 0);

      Serial.println();
      Serial.println("Teste concluido.");
      Serial.println("PWM retornado para 0%.");
      Serial.println();
      Serial.println("Digite T novamente para repetir.");
    }
  }
}

// ===============================
// TESTE COMPLETO
// ===============================

void executarTesteCompleto() {

  for (int p = 0; p < NUM_PWM; p++) {

    int dutyPercent = pwmPercent[p];

    int dutyValue =
      round((dutyPercent / 100.0) * 255.0);

    ledcWrite(PWM_PIN, dutyValue);

    Serial.println();
    Serial.println("======================================");

    Serial.print("PWM = ");
    Serial.print(dutyPercent);
    Serial.println("%");

    Serial.print("Valor digital = ");
    Serial.println(dutyValue);

    Serial.println("Aguardando estabilizacao...");

    delay(TEMPO_ESTABILIZACAO_MS);

    medirPWM(p);
  }

  imprimirTabelaFinal();
}

// ===============================
// MEDICAO DE CADA PWM
// ===============================

void medirPWM(int indicePWM) {

  double soma[NUM_CANAIS];
  double somaQuadrados[NUM_CANAIS];

  for (int i = 0; i < NUM_CANAIS; i++) {
    soma[i] = 0;
    somaQuadrados[i] = 0;
  }

  uint16_t leituras[18];

  int leiturasValidas = 0;

  for (int amostra = 0; amostra < N_AMOSTRAS; amostra++) {

    if (!as7343.readAllChannels(leituras)) {

      Serial.print("[ERRO] Leitura ");
      Serial.println(amostra + 1);

      delay(500);
      continue;
    }

    leiturasValidas++;

    Serial.print("Amostra ");
    Serial.print(amostra + 1);
    Serial.print("/");
    Serial.println(N_AMOSTRAS);

    for (int i = 0; i < NUM_CANAIS; i++) {

      uint16_t valor = leituras[canais[i]];

      soma[i] += valor;

      somaQuadrados[i] +=
        ((double)valor * (double)valor);
    }

    delay(INTERVALO_MS);
  }

  for (int i = 0; i < NUM_CANAIS; i++) {

    double media =
      soma[i] / leiturasValidas;

    double variancia =
      (somaQuadrados[i] / leiturasValidas)
      - (media * media);

    if (variancia < 0) {
      variancia = 0;
    }

    double dp =
      sqrt(variancia);

    mediaResultado[indicePWM][i] = media;
    dpResultado[indicePWM][i] = dp;
  }
}

// ===============================
// TABELA FINAL
// ===============================

void imprimirTabelaFinal() {

  Serial.println();
  Serial.println();
  Serial.println("======================================");
  Serial.println(" RESULTADOS FINAIS");
  Serial.println("======================================");
  Serial.println();

  Serial.println("PWM\tCanal\tMedia\tDP");

  for (int p = 0; p < NUM_PWM; p++) {

    for (int i = 0; i < NUM_CANAIS; i++) {

      Serial.print(pwmPercent[p]);
      Serial.print("%\t");

      Serial.print(nomes[i]);
      Serial.print("\t");

      Serial.print(mediaResultado[p][i], 2);
      Serial.print("\t");

      Serial.println(dpResultado[p][i], 2);
    }

    Serial.println();
  }

  // tabela resumida dos canais principais
  Serial.println();
  Serial.println("======================================");
  Serial.println(" RESUMO - REGIAO AZUL");
  Serial.println("======================================");

  Serial.println("PWM\tF2 425nm\tFZ 450nm\tF3 475nm");

  for (int p = 0; p < NUM_PWM; p++) {

    Serial.print(pwmPercent[p]);
    Serial.print("%\t");

    Serial.print(mediaResultado[p][1], 2);
    Serial.print("\t");

    Serial.print(mediaResultado[p][2], 2);
    Serial.print("\t");

    Serial.println(mediaResultado[p][3], 2);
  }
}