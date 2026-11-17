/*
  ============================================================
  TCC - Firmware preliminar integrado
  Monitoramento e controle de irradiancia espectral
  ESP32 + TCA9548A + 3x GY-AS7343-V2 + OLED SSD1306
  ============================================================

  Protoboard atual:
    Sensor 1 -> TCA canal 7 (SD7 / SC7)
    Sensor 2 -> TCA canal 5 (SD5 / SC5)
    Sensor 3 -> TCA canal 3 (SD3 / SC3)

  OLED:
    SDA -> GPIO 21
    SCL -> GPIO 22
    Endereco assumido: 0x3C

  PWM:
    GPIO 25
    500 Hz
    8 bits

  IMPORTANTE:
  A conversao RAW -> irradiancia abaixo e TEMPORARIA.
  Ela serve apenas para testar a logica do firmware.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AS7343.h>

// ============================================================
// I2C
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define TCA_ADDR  0x71
#define OLED_ADDR 0x3C

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

bool oledOK = false;

// ============================================================
// SENSORES
// ============================================================

const uint8_t NUM_SENSORS = 3;

// Ordem:
// S1 -> CH7
// S2 -> CH5
// S3 -> CH3
const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {7, 5, 3};

Adafruit_AS7343 sensors[NUM_SENSORS];

// Configuracao utilizada nos testes
#define SENSOR_GAIN AS7343_GAIN_32X
#define SENSOR_ATIME 29
#define SENSOR_ASTEP 599

// Full-scale observado
const uint16_t RAW_SATURATION = 17999;

// Pequena margem para indicar saturacao
const uint16_t RAW_SAT_WARNING = 17990;

// Irradiancia arbitrada para a saturacao
const float IRRADIANCE_MAX = 30.0f;

// ============================================================
// FAIXA TERAPEUTICA / REFERENCIA
// ============================================================

// Faixa utilizada no projeto
const float IRRADIANCE_MIN = 8.0f;
const float IRRADIANCE_MAX_THERAPEUTIC = 10.0f;

// Centro da faixa
const float IRRADIANCE_REFERENCE = 9.0f;

// ============================================================
// PWM
// ============================================================

#define PWM_PIN 25

const uint32_t PWM_FREQ = 500;
const uint8_t PWM_RESOLUTION = 8;
const uint16_t PWM_DUTY_MAX = 255;

// Inicio propositalmente baixo
const float PWM_INITIAL = 10.0f;

// Limites preliminares de bancada
const float PWM_MIN = 0.0f;
const float PWM_MAX = 75.0f;

// Controlador proporcional PRELIMINAR
// unidade aproximada:
// [%PWM / (uW/cm2/nm)]
const float KP = 1.5f;

// Evita mudancas abruptas
const float MAX_PWM_STEP = 5.0f;

float pwmPercent = PWM_INITIAL;

// ============================================================
// VALIDACAO PROVISORIA DOS SENSORES
// ============================================================

// Projeto final:
// minimo = 4 sensores de 5.
//
// Bancada atual:
// usa-se 2 de 3 para testar a logica de modo degradado.
const uint8_t MIN_VALID_SENSORS = 2;

// Regra provisoria de obstrucao
const float OBSTRUCTION_RATIO = 0.35f;
const uint8_t OBSTRUCTION_CYCLES = 3;
const float OBSTRUCTION_MIN_REFERENCE = 2.0f;

// ============================================================
// TEMPORIZACAO
// ============================================================

const uint32_t SAMPLE_INTERVAL = 1000;   // 1 leitura/s

uint32_t lastSampleTime = 0;

// ============================================================
// ESTRUTURAS
// ============================================================

struct SensorData {

  bool initialized;
  bool valid;
  bool saturated;
  bool obstructed;

  uint16_t channels[18];

  uint16_t blueRaw;
  float irradiance;

  uint8_t obstructionCounter;
};

SensorData dataSensor[NUM_SENSORS];

// ============================================================
// ESTADO DO SISTEMA
// ============================================================

enum SystemStatus {

  STATUS_INITIALIZING,
  STATUS_MONITORING,
  STATUS_OUT_OF_RANGE,
  STATUS_CONTROLLING,
  STATUS_CONTROL_DONE,
  STATUS_PWM_LIMIT,
  STATUS_SENSOR_FAULT,
  STATUS_OFF

};

SystemStatus systemStatus = STATUS_INITIALIZING;

bool systemOn = true;
bool controlActive = false;

float representativeIrradiance = 0.0f;
uint8_t validSensors = 0;

uint8_t displayPage = 0;

// ============================================================
// FUNCOES AUXILIARES
// ============================================================

float clampFloat(float value, float minimum, float maximum) {

  if (value < minimum) return minimum;
  if (value > maximum) return maximum;

  return value;
}

// ============================================================
// TCA9548A
// ============================================================

bool tcaSelect(uint8_t channel) {

  if (channel > 7) return false;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

void tcaDisableAll() {

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

// ============================================================
// PWM
// ============================================================

void setPWM(float percent) {

  percent = clampFloat(percent, 0.0f, 100.0f);

  pwmPercent = percent;

  uint16_t duty =
    (uint16_t)((percent / 100.0f) * PWM_DUTY_MAX);

  ledcWrite(PWM_PIN, duty);
}

// ============================================================
// CALCULO DA LEITURA AZUL
// ============================================================

uint16_t getBluePeak(const uint16_t *r) {

  /*
     Como ainda nao ha calibracao espectral,
     usamos temporariamente o MAIOR canal
     dentro da faixa de interesse 400-550 nm.

     Canais:
       F1 = 405 nm
       F2 = 425 nm
       FZ = 450 nm
       F3 = 475 nm
       F4 = 515 nm
       F5 = 550 nm

     Isso permite que 17999 corresponda
     aproximadamente a 30 uW/cm2/nm.
  */

  uint16_t peak = r[AS7343_CHANNEL_F1];

  if (r[AS7343_CHANNEL_F2] > peak)
    peak = r[AS7343_CHANNEL_F2];

  if (r[AS7343_CHANNEL_FZ] > peak)
    peak = r[AS7343_CHANNEL_FZ];

  if (r[AS7343_CHANNEL_F3] > peak)
    peak = r[AS7343_CHANNEL_F3];

  if (r[AS7343_CHANNEL_F4] > peak)
    peak = r[AS7343_CHANNEL_F4];

  if (r[AS7343_CHANNEL_F5] > peak)
    peak = r[AS7343_CHANNEL_F5];

  return peak;
}

// ============================================================
// CONVERSAO PROVISORIA RAW -> IRRADIANCIA
// ============================================================

float rawToIrradiance(uint16_t raw) {

  float E =
    IRRADIANCE_MAX *
    ((float)raw / (float)RAW_SATURATION);

  if (E > IRRADIANCE_MAX)
    E = IRRADIANCE_MAX;

  return E;
}

// ============================================================
// INICIALIZACAO DE UM SENSOR
// ============================================================

bool initializeSensor(uint8_t index) {

  if (!tcaSelect(SENSOR_CHANNELS[index])) {

    Serial.print("Erro ao selecionar TCA CH");
    Serial.println(SENSOR_CHANNELS[index]);

    return false;
  }

  if (!sensors[index].begin(0x39, &Wire)) {

    Serial.print("Sensor S");
    Serial.print(index + 1);
    Serial.println(" nao encontrado.");

    return false;
  }

  sensors[index].setGain(SENSOR_GAIN);
  sensors[index].setATIME(SENSOR_ATIME);
  sensors[index].setASTEP(SENSOR_ASTEP);
  sensors[index].setSMUXMode(AS7343_SMUX_18CH);

  Serial.print("Sensor S");
  Serial.print(index + 1);
  Serial.print(" OK - TCA CH");
  Serial.println(SENSOR_CHANNELS[index]);

  return true;
}

// ============================================================
// AQUISICAO DE UM SENSOR
// ============================================================

bool readSensor(uint8_t index) {

  dataSensor[index].valid = false;
  dataSensor[index].saturated = false;
  dataSensor[index].obstructed = false;

  Serial.println();
  Serial.print("[S");
  Serial.print(index + 1);
  Serial.print(" / CH");
  Serial.print(SENSOR_CHANNELS[index]);
  Serial.println("]");

  // 1. Sensor foi inicializado?
  if (!dataSensor[index].initialized) {
    Serial.println("ERRO: sensor nao inicializado.");
    return false;
  }

  Serial.println("Sensor inicializado: OK");

  // 2. Seleciona canal do multiplexador
  if (!tcaSelect(SENSOR_CHANNELS[index])) {
    Serial.println("ERRO: falha ao selecionar canal TCA.");
    return false;
  }

  Serial.println("Canal TCA: OK");

  // 3. Inicia medicao
  if (!sensors[index].startMeasurement()) {
    Serial.println("ERRO: startMeasurement() falhou.");
    return false;
  }

  Serial.println("Medicao iniciada: OK");

  // 4. Aguarda dado
  uint32_t start = millis();

  while (!sensors[index].dataReady()) {

    if (millis() - start > 1000) {
      Serial.println("ERRO: timeout esperando dataReady().");
      return false;
    }

    delay(1);
  }

  Serial.println("dataReady: OK");

  // 5. Le todos os canais
  if (!sensors[index].readAllChannels(
        dataSensor[index].channels)) {

    Serial.println("ERRO: readAllChannels() falhou.");
    return false;
  }

  Serial.println("readAllChannels: OK");

  // 6. Calcula RAW azul
  dataSensor[index].blueRaw =
    getBluePeak(dataSensor[index].channels);

  dataSensor[index].irradiance =
    rawToIrradiance(dataSensor[index].blueRaw);

  dataSensor[index].saturated =
    dataSensor[index].blueRaw >= RAW_SAT_WARNING;

  dataSensor[index].valid = true;

  Serial.print("RAW azul = ");
  Serial.println(dataSensor[index].blueRaw);

  Serial.println("LEITURA CONCLUIDA: OK");

  return true;
}

// ============================================================
// DETECCAO PROVISORIA DE OBSTRUCAO
// ============================================================

void detectObstructions() {

  /*
     Esta rotina ainda NAO e definitiva.

     Ideia:
     se UM sensor cair fortemente enquanto
     os outros dois permanecem altos por
     tres aquisicoes consecutivas,
     considera possivel obstrucao.
  */

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    if (!dataSensor[i].valid)
      continue;

    float otherSum = 0.0f;
    uint8_t otherCount = 0;

    for (uint8_t j = 0; j < NUM_SENSORS; j++) {

      if (j == i)
        continue;

      if (dataSensor[j].valid) {

        otherSum += dataSensor[j].irradiance;
        otherCount++;
      }
    }

    // Com 3 sensores precisamos dos outros 2
    if (otherCount < 2) {

      dataSensor[i].obstructionCounter = 0;
      continue;
    }

    float otherAverage =
      otherSum / otherCount;

    bool possibleObstruction =
      (otherAverage > OBSTRUCTION_MIN_REFERENCE) &&
      (dataSensor[i].irradiance <
       otherAverage * OBSTRUCTION_RATIO);

    if (possibleObstruction) {

      if (dataSensor[i].obstructionCounter <
          OBSTRUCTION_CYCLES) {

        dataSensor[i].obstructionCounter++;
      }

    } else {

      dataSensor[i].obstructionCounter = 0;
    }

    if (dataSensor[i].obstructionCounter >=
        OBSTRUCTION_CYCLES) {

      dataSensor[i].obstructed = true;
      dataSensor[i].valid = false;
    }
  }
}

// ============================================================
// IRRADIANCIA REPRESENTATIVA
// ============================================================

void calculateRepresentativeIrradiance() {

  float sum = 0.0f;
  validSensors = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    if (dataSensor[i].valid) {

      sum += dataSensor[i].irradiance;
      validSensors++;
    }
  }

  if (validSensors > 0)
    representativeIrradiance = sum / validSensors;
  else
    representativeIrradiance = 0.0f;
}

// ============================================================
// FAIXA TERAPEUTICA
// ============================================================

bool irradianceInRange() {

  return
    representativeIrradiance >= IRRADIANCE_MIN &&
    representativeIrradiance <=
      IRRADIANCE_MAX_THERAPEUTIC;
}

// ============================================================
// CONTROLE
// ============================================================

void executeControlStep() {

  if (!controlActive)
    return;

  if (irradianceInRange()) {

    controlActive = false;
    systemStatus = STATUS_CONTROL_DONE;

    Serial.println();
    Serial.println("================================");
    Serial.println(" CONTROLE CONCLUIDO");
    Serial.println(" Irradiancia dentro da faixa");
    Serial.println("================================");

    return;
  }

  float error =
    IRRADIANCE_REFERENCE -
    representativeIrradiance;

  float deltaPWM = KP * error;

  // Evita salto grande em protoboard
  deltaPWM =
    clampFloat(
      deltaPWM,
      -MAX_PWM_STEP,
      MAX_PWM_STEP
    );

  float newPWM =
    pwmPercent + deltaPWM;

  // Fluxograma:
  // verifica limite ANTES da nova atuacao
  if (newPWM < PWM_MIN ||
      newPWM > PWM_MAX) {

    controlActive = false;
    systemStatus = STATUS_PWM_LIMIT;

    Serial.println();
    Serial.println("***** LIMITE OPERACIONAL *****");
    Serial.println("Controle encerrado.");
    Serial.println("PWM solicitado fora do limite.");

    return;
  }

  setPWM(newPWM);

  systemStatus = STATUS_CONTROLLING;

  Serial.print("Controle -> erro = ");
  Serial.print(error, 2);

  Serial.print(" | novo PWM = ");
  Serial.print(pwmPercent, 1);

  Serial.println("%");
}

// ============================================================
// OLED - CABECALHO
// ============================================================

void beginDisplayFrame() {

  if (!oledOK)
    return;

  // OLED fica no barramento principal.
  // Desconectamos canais do TCA antes de escrever.
  tcaDisableAll();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
}

// ============================================================
// OLED - MONITORAMENTO
// ============================================================

void drawMonitoringPage() {

  beginDisplayFrame();

  if (!oledOK)
    return;

  if (displayPage == 0) {

    display.println("MONITORAMENTO");
    display.println("----------------");

    display.print("Erep: ");
    display.print(representativeIrradiance, 1);
    display.println(" uW/cm2/nm");

    display.print("PWM : ");
    display.print(pwmPercent, 1);
    display.println("%");

    display.print("Sens: ");
    display.print(validSensors);
    display.print("/");
    display.println(NUM_SENSORS);

    if (irradianceInRange())
      display.println("STATUS: FAIXA OK");
    else
      display.println("FORA FAIXA -> C");

  }

  else if (displayPage == 1) {

    display.println("IRRADIANCIA / SENSOR");
    display.println("--------------------");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {

      display.print("S");
      display.print(i + 1);
      display.print(": ");

      if (dataSensor[i].valid) {

        display.print(
          dataSensor[i].irradiance,
          1
        );

        if (dataSensor[i].saturated)
          display.print(" SAT");

      }

      else if (dataSensor[i].obstructed) {

        display.print("OBSTR");

      } else {

        display.print("ERRO");
      }

      display.println();
    }
  }

  else {

    display.println("RAW - REGIAO AZUL");
    display.println("----------------");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {

      display.print("S");
      display.print(i + 1);
      display.print(": ");

      display.println(
        dataSensor[i].blueRaw
      );
    }

    display.print("SAT=");
    display.println(RAW_SATURATION);
  }

  display.display();
}

// ============================================================
// OLED - MENSAGENS DE ESTADO
// ============================================================

void drawStatus() {

  if (!oledOK)
    return;

  // Monitoramento normal usa paginas
  if (systemStatus == STATUS_MONITORING) {

    drawMonitoringPage();
    return;
  }

  beginDisplayFrame();

  switch (systemStatus) {

    case STATUS_INITIALIZING:

      display.println("INICIALIZANDO...");
      display.println();
      display.println("ESP32");
      display.println("TCA9548A");
      display.println("AS7343");
      display.println("OLED");

      break;

    case STATUS_OUT_OF_RANGE:

      display.println("*** ALERTA ***");
      display.println();
      display.println("IRRADIANCIA");
      display.println("FORA DA FAIXA");
      display.println();

      display.print("E=");
      display.println(
        representativeIrradiance,
        1
      );

      display.println("C=calibrar N=nao");

      break;

    case STATUS_CONTROLLING:

      display.println("CONTROLE ATIVO");
      display.println("----------------");

      display.print("Erep: ");
      display.println(
        representativeIrradiance,
        1
      );

      display.print("Ref : ");
      display.println(
        IRRADIANCE_REFERENCE,
        1
      );

      display.print("PWM : ");
      display.print(pwmPercent, 1);
      display.println("%");

      display.println("Nova medicao...");

      break;

    case STATUS_CONTROL_DONE:

      display.println("CONTROLE");
      display.println("CONCLUIDO");
      display.println();
      display.println("IRRADIANCIA");
      display.println("DENTRO DA FAIXA");
      display.println();

      display.print("E=");
      display.println(
        representativeIrradiance,
        1
      );

      break;

    case STATUS_PWM_LIMIT:

      display.println("*** ALERTA ***");
      display.println();
      display.println("LIMITE OPERACIONAL");
      display.println("DO PWM ATINGIDO");
      display.println();
      display.println("CONTROLE ENCERRADO");

      break;

    case STATUS_SENSOR_FAULT:

      display.println("*** FALHA ***");
      display.println();
      display.println("SENSORES");
      display.println("INSUFICIENTES");
      display.println();

      display.print("Validos: ");
      display.print(validSensors);
      display.print("/");
      display.println(NUM_SENSORS);

      display.println("Controle bloqueado");

      break;

    case STATUS_OFF:

      display.println("SISTEMA DESLIGADO");
      display.println();
      display.println("PWM = 0%");
      display.println();
      display.println("Digite O");
      display.println("para religar");

      break;

    default:
      break;
  }

  display.display();
}

// ============================================================
// SERIAL - RESULTADOS
// ============================================================

void printMeasurements() {

  Serial.println();
  Serial.println(
    "================================================"
  );

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    Serial.print("S");
    Serial.print(i + 1);

    Serial.print(" CH");
    Serial.print(SENSOR_CHANNELS[i]);

    Serial.print(" | RAW=");
    Serial.print(dataSensor[i].blueRaw);

    Serial.print(" | E=");
    Serial.print(
      dataSensor[i].irradiance,
      2
    );

    Serial.print(" uW/cm2/nm");

    if (dataSensor[i].saturated)
      Serial.print(" | SATURADO");

    if (dataSensor[i].obstructed)
      Serial.print(" | OBSTRUIDO");

    if (!dataSensor[i].valid)
      Serial.print(" | INVALIDO");

    Serial.println();
  }

  Serial.println("------------------------------");

  Serial.print("E REPRESENTATIVA = ");
  Serial.print(
    representativeIrradiance,
    2
  );

  Serial.println(" uW/cm2/nm");

  Serial.print("PWM = ");
  Serial.print(pwmPercent, 1);
  Serial.println("%");

  Serial.print("Sensores validos = ");
  Serial.print(validSensors);
  Serial.print("/");
  Serial.println(NUM_SENSORS);

  Serial.println(
    "================================================"
  );
}

// ============================================================
// AQUISICAO COMPLETA
// ============================================================

void acquireSystem() {

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {

    readSensor(i);
  }

  detectObstructions();

  calculateRepresentativeIrradiance();

  printMeasurements();

  // Quantidade insuficiente
  if (validSensors < MIN_VALID_SENSORS) {

    controlActive = false;

    systemStatus =
      STATUS_SENSOR_FAULT;

    drawStatus();

    return;
  }

  // Se controle estiver ativo,
  // executa um passo da malha.
  if (controlActive) {

    executeControlStep();

    drawStatus();

    return;
  }

  // Monitoramento normal
  if (irradianceInRange()) {

    systemStatus =
      STATUS_MONITORING;

  } else {

    systemStatus =
      STATUS_OUT_OF_RANGE;
  }

  drawStatus();
}

// ============================================================
// COMANDOS SERIAIS
// ============================================================

void handleSerial() {

  while (Serial.available()) {

    char command =
      Serial.read();

    // Ignora quebra de linha
    if (command == '\n' ||
        command == '\r' ||
        command == ' ')
      continue;

    if (command >= 'A' &&
        command <= 'Z') {

      command =
        command - 'A' + 'a';
    }

    // --------------------------------------------------------
    // ON / OFF
    // --------------------------------------------------------

    if (command == 'o') {

      if (systemOn) {

        systemOn = false;
        controlActive = false;

        setPWM(0.0f);

        systemStatus =
          STATUS_OFF;

        Serial.println(
          "Sistema DESLIGADO."
        );

      } else {

        systemOn = true;

        setPWM(PWM_INITIAL);

        systemStatus =
          STATUS_MONITORING;

        lastSampleTime = 0;

        Serial.println(
          "Sistema LIGADO."
        );

        Serial.print(
          "PWM inicial = "
        );

        Serial.print(
          PWM_INITIAL
        );

        Serial.println("%");
      }

      drawStatus();
    }

    // --------------------------------------------------------
    // CALIBRAR / CONFIRMAR
    // --------------------------------------------------------

    else if (command == 'c') {

      if (!systemOn)
        continue;

      if (validSensors <
          MIN_VALID_SENSORS) {

        Serial.println(
          "Controle bloqueado: sensores insuficientes."
        );

        continue;
      }

      if (irradianceInRange()) {

        Serial.println(
          "Irradiancia ja esta dentro da faixa."
        );

        continue;
      }

      controlActive = true;

      systemStatus =
        STATUS_CONTROLLING;

      Serial.println();
      Serial.println(
        "ROTINA DE CONTROLE AUTORIZADA."
      );

      drawStatus();
    }

    // --------------------------------------------------------
    // CANCELAR
    // --------------------------------------------------------

    else if (command == 'n') {

      controlActive = false;

      Serial.println(
        "Controle nao autorizado."
      );

      Serial.println(
        "Mantendo monitoramento."
      );
    }

    // --------------------------------------------------------
    // AVANCAR DISPLAY
    // --------------------------------------------------------

    else if (command == 'a') {

      displayPage++;

      if (displayPage > 2)
        displayPage = 0;

      if (systemStatus ==
          STATUS_MONITORING)

        drawMonitoringPage();
    }

    // --------------------------------------------------------
    // VOLTAR DISPLAY
    // --------------------------------------------------------

    else if (command == 'v') {

      if (displayPage == 0)
        displayPage = 2;
      else
        displayPage--;

      if (systemStatus ==
          STATUS_MONITORING)

        drawMonitoringPage();
    }

    // --------------------------------------------------------
    // AJUDA
    // --------------------------------------------------------

    else if (command == 'h') {

      Serial.println();
      Serial.println("COMANDOS:");
      Serial.println("O = ON/OFF");
      Serial.println("C = confirmar controle");
      Serial.println("N = cancelar controle");
      Serial.println("A = avancar tela");
      Serial.println("V = voltar tela");
      Serial.println("H = ajuda");
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    " SISTEMA DE MONITORAMENTO - TCC"
  );
  Serial.println(
    " Firmware preliminar - 3 sensores"
  );
  Serial.println(
    "========================================"
  );

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  Wire.setClock(100000);

  tcaDisableAll();

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  oledOK =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    );

  if (oledOK) {

    systemStatus =
      STATUS_INITIALIZING;

    drawStatus();

  } else {

    Serial.println(
      "AVISO: OLED nao encontrado."
    );

    Serial.println(
      "Firmware continuara pelo Serial."
    );
  }

  // ----------------------------------------------------------
  // PWM
  // ----------------------------------------------------------

  if (!ledcAttach(
        PWM_PIN,
        PWM_FREQ,
        PWM_RESOLUTION)) {

    Serial.println(
      "ERRO ao configurar PWM."
    );
  }

  // Comeca desligado durante inicializacao
  setPWM(0.0f);

  // ----------------------------------------------------------
  // SENSORES
  // ----------------------------------------------------------

  for (uint8_t i = 0;
       i < NUM_SENSORS;
       i++) {

    dataSensor[i].initialized =
      initializeSensor(i);

    dataSensor[i].valid = false;
    dataSensor[i].saturated = false;
    dataSensor[i].obstructed = false;
    dataSensor[i].blueRaw = 0;
    dataSensor[i].irradiance = 0;
    dataSensor[i].obstructionCounter = 0;
  }

  tcaDisableAll();

  // ----------------------------------------------------------
  // PWM INICIAL
  // ----------------------------------------------------------

  setPWM(PWM_INITIAL);

  Serial.println();
  Serial.print(
    "PWM inicial aplicado: "
  );

  Serial.print(
    PWM_INITIAL
  );

  Serial.println("%");

  Serial.println();
  Serial.println(
    "Digite H para ver os comandos."
  );

  systemStatus =
    STATUS_MONITORING;

  delay(1000);
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  handleSerial();

  if (!systemOn) {

    delay(10);
    return;
  }

  if (millis() - lastSampleTime >=
      SAMPLE_INTERVAL) {

    lastSampleTime = millis();

    acquireSystem();
  }
}