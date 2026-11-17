#include <Wire.h>
#include <Adafruit_AS7343.h>
#include <math.h>

// =====================================================
// I2C - AS7343
// =====================================================
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_AS7343 as7343;

// =====================================================
// PWM - DRIVER DE LED
// =====================================================
#define PWM_PIN 25

const int PWM_FREQ = 500;       // 500 Hz
const int PWM_RESOLUTION = 8;   // 8 bits -> 0 a 255

int dutyPercentAtual = 0;
int dutyValueAtual = 0;

// =====================================================
// TESTE DE REPETIBILIDADE
// =====================================================
#define N_AMOSTRAS 100
#define INTERVALO_MS 200

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

double soma[NUM_CANAIS];
double somaQuadrados[NUM_CANAIS];

uint16_t minimo[NUM_CANAIS];
uint16_t maximo[NUM_CANAIS];

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===========================================");
  Serial.println(" AS7343 + CONTROLE PWM - ESP32");
  Serial.println("===========================================");
  Serial.println();

  // ---------------------------------------------------
  // I2C
  // ---------------------------------------------------

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!as7343.begin()) {
    Serial.println("[ERRO] AS7343 nao encontrado!");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[OK] AS7343 detectado.");

  // Mesma configuração dos testes anteriores
  as7343.setGain(AS7343_GAIN_64X);
  as7343.setATIME(29);
  as7343.setASTEP(599);

  Serial.println("[OK] Configuracao do sensor:");
  Serial.println("GAIN  = 64X");
  Serial.println("ATIME = 29");
  Serial.println("ASTEP = 599");
  Serial.println();

  // ---------------------------------------------------
  // PWM
  // ---------------------------------------------------

  if (!ledcAttach(PWM_PIN, PWM_FREQ, PWM_RESOLUTION)) {
    Serial.println("[ERRO] Falha ao configurar PWM.");
    while (true) {
      delay(1000);
    }
  }

  // Começa desligado
  ledcWrite(PWM_PIN, 0);

  dutyPercentAtual = 0;
  dutyValueAtual = 0;

  Serial.println("[OK] PWM configurado:");
  Serial.println("GPIO = 25");
  Serial.println("Freq = 500 Hz");
  Serial.println("Resolucao = 8 bits");
  Serial.println();

  mostrarMenu();
}

// =====================================================
// LOOP
// =====================================================

void loop() {

  if (Serial.available()) {

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.length() == 0) {
      return;
    }

    // -------------------------------------------------
    // INICIAR TESTE
    // -------------------------------------------------

    if (comando.equalsIgnoreCase("t")) {

      Serial.println();
      Serial.println("Comando recebido: iniciar teste.");

      realizarTeste();

      mostrarMenu();
      return;
    }

    // -------------------------------------------------
    // MOSTRAR STATUS
    // -------------------------------------------------

    if (comando.equalsIgnoreCase("s")) {

      mostrarStatus();
      return;
    }

    // -------------------------------------------------
    // ALTERAR PWM
    // -------------------------------------------------

    bool numeroValido = true;

    for (unsigned int i = 0; i < comando.length(); i++) {

      if (!isDigit(comando[i])) {
        numeroValido = false;
        break;
      }
    }

    if (numeroValido) {

      int novoDuty = comando.toInt();

      if (novoDuty >= 0 && novoDuty <= 100) {

        configurarPWM(novoDuty);

      } else {

        Serial.println();
        Serial.println("[ERRO] Digite um duty entre 0 e 100%.");
      }

    } else {

      Serial.println();
      Serial.println("[ERRO] Comando invalido.");
      mostrarMenu();
    }
  }
}

// =====================================================
// CONFIGURAR PWM
// =====================================================

void configurarPWM(int dutyPercent) {

  int dutyValue =
    round((dutyPercent / 100.0) * 255.0);

  ledcWrite(PWM_PIN, dutyValue);

  dutyPercentAtual = dutyPercent;
  dutyValueAtual = dutyValue;

  Serial.println();
  Serial.println("-------------------------------------------");

  Serial.print("Duty configurado: ");
  Serial.print(dutyPercentAtual);
  Serial.println("%");

  Serial.print("Valor PWM: ");
  Serial.println(dutyValueAtual);

  Serial.println("-------------------------------------------");

  Serial.println();
  Serial.println("Aguarde a fonte estabilizar.");

  Serial.println(
    "Quando estiver pronto, digite T para iniciar o teste."
  );
}

// =====================================================
// TESTE DE REPETIBILIDADE
// =====================================================

void realizarTeste() {

  if (dutyPercentAtual == 0) {

    Serial.println();
    Serial.println(
      "[AVISO] O PWM esta em 0%. "
      "O teste sera realizado com a fonte desligada."
    );
  }

  Serial.println();
  Serial.println("===========================================");
  Serial.println(" TESTE 3 - REPETIBILIDADE");
  Serial.println("===========================================");

  Serial.print("Duty utilizado: ");
  Serial.print(dutyPercentAtual);
  Serial.println("%");

  Serial.print("PWM digital: ");
  Serial.println(dutyValueAtual);

  Serial.println();

  Serial.println("Nao mova:");
  Serial.println("- sensor");
  Serial.println("- malha de LEDs");
  Serial.println("- distancia");
  Serial.println("- suporte");
  Serial.println();

  Serial.println("Aguardando 5 segundos para estabilizacao...");
  delay(5000);

  // Inicialização
  for (int i = 0; i < NUM_CANAIS; i++) {

    soma[i] = 0;
    somaQuadrados[i] = 0;

    minimo[i] = 65535;
    maximo[i] = 0;
  }

  uint16_t leituras[18];

  int leiturasValidas = 0;

  // ---------------------------------------------------
  // AQUISIÇÃO
  // ---------------------------------------------------

  for (int amostra = 0; amostra < N_AMOSTRAS; amostra++) {

    if (!as7343.readAllChannels(leituras)) {

      Serial.print("[ERRO] Falha na leitura ");
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

      uint16_t valor =
        leituras[canais[i]];

      soma[i] += valor;

      somaQuadrados[i] +=
        ((double)valor * (double)valor);

      if (valor < minimo[i]) {
        minimo[i] = valor;
      }

      if (valor > maximo[i]) {
        maximo[i] = valor;
      }
    }

    delay(INTERVALO_MS);
  }

  // ---------------------------------------------------
  // RESULTADOS
  // ---------------------------------------------------

  Serial.println();
  Serial.println("===========================================");
  Serial.println(" RESULTADOS - REPETIBILIDADE");
  Serial.println("===========================================");

  Serial.print("PWM: ");
  Serial.print(dutyPercentAtual);
  Serial.println("%");

  Serial.print("Leituras validas: ");
  Serial.println(leiturasValidas);

  Serial.println();

  Serial.println(
    "Canal\tMedia\tDP\tCV(%)\tMin\tMax"
  );

  if (leiturasValidas == 0) {

    Serial.println("[ERRO] Nenhuma leitura valida.");
    return;
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

    double desvioPadrao =
      sqrt(variancia);

    double cv = 0;

    if (media > 0) {
      cv =
        (desvioPadrao / media) * 100.0;
    }

    Serial.print(nomes[i]);
    Serial.print("\t");

    Serial.print(media, 2);
    Serial.print("\t");

    Serial.print(desvioPadrao, 2);
    Serial.print("\t");

    Serial.print(cv, 3);
    Serial.print("\t");

    Serial.print(minimo[i]);
    Serial.print("\t");

    Serial.println(maximo[i]);
  }

  Serial.println();
  Serial.println("Teste concluido.");
}

// =====================================================
// STATUS
// =====================================================

void mostrarStatus() {

  Serial.println();
  Serial.println("========== STATUS ==========");

  Serial.print("PWM atual: ");
  Serial.print(dutyPercentAtual);
  Serial.println("%");

  Serial.print("Valor PWM digital: ");
  Serial.println(dutyValueAtual);

  Serial.println("============================");
}

// =====================================================
// MENU
// =====================================================

void mostrarMenu() {

  Serial.println();
  Serial.println("===========================================");
  Serial.println(" COMANDOS");
  Serial.println("===========================================");
  Serial.println();

  Serial.println("Digite um valor de 0 a 100");
  Serial.println("para configurar o PWM.");
  Serial.println();

  Serial.println("Exemplos:");
  Serial.println("0   -> 0%");
  Serial.println("25  -> 25%");
  Serial.println("50  -> 50%");
  Serial.println("75  -> 75%");
  Serial.println("100 -> 100%");
  Serial.println();

  Serial.println("T -> iniciar teste de repetibilidade");
  Serial.println("S -> mostrar status atual");

  Serial.println();
}