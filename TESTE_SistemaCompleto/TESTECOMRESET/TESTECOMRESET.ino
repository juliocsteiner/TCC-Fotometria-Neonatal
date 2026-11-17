/*
  ============================================================
  TCC - Firmware preliminar integrado

  ESP32 + TCA9548A + 3x AS7343 + OLED SSD1306

  PINAGEM:

  GPIO 18 -> PWM LEDs
  GPIO 21 -> SDA
  GPIO 22 -> SCL
  GPIO 25 -> Botao ON/OFF -> GND
  GPIO 26 -> Botao CALIBRACAO -> GND
  GPIO 34 -> RESET TCA9548A

  TCA9548A -> 0x71

  S1 -> TCA CH7
  S2 -> TCA CH5
  S3 -> TCA CH3

  OLED -> 0x3C
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AS7343.h>


// ============================================================
// PINOS
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define PWM_PIN 18

#define BUTTON_ON_OFF 25
#define BUTTON_CALIBRATION 26

#define TCA_RESET_PIN 34


// ============================================================
// I2C
// ============================================================

#define TCA_ADDR  0x71
#define OLED_ADDR 0x3C
#define SENSOR_ADDR 0x39


// ============================================================
// TEMPOS DE INICIALIZACAO
// ============================================================

// Tempo para a alimentacao estabilizar no cold boot
const uint32_t POWER_STABILIZATION_MS = 1000;

// Pulso de reset do TCA
const uint32_t TCA_RESET_PULSE_MS = 10;

// Tempo apos liberar RESET
const uint32_t TCA_RESET_RECOVERY_MS = 50;

// Numero de tentativas
const uint8_t TCA_INIT_RETRIES = 5;
const uint8_t SENSOR_INIT_RETRIES = 5;


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

const uint8_t SENSOR_CHANNELS[NUM_SENSORS] = {
  7,
  5,
  3
};

Adafruit_AS7343 sensors[NUM_SENSORS];

#define SENSOR_GAIN AS7343_GAIN_32X
#define SENSOR_ATIME 29
#define SENSOR_ASTEP 599

const uint16_t RAW_SATURATION = 17999;
const uint16_t RAW_SAT_WARNING = 17990;

const float IRRADIANCE_MAX = 30.0f;


// ============================================================
// FAIXA TERAPEUTICA
// ============================================================

const float IRRADIANCE_MIN = 8.0f;
const float IRRADIANCE_MAX_THERAPEUTIC = 10.0f;
const float IRRADIANCE_REFERENCE = 9.0f;


// ============================================================
// PWM
// ============================================================

const uint32_t PWM_FREQ = 500;
const uint8_t PWM_RESOLUTION = 8;
const uint16_t PWM_DUTY_MAX = 255;

const float PWM_INITIAL = 10.0f;

const float PWM_MIN = 0.0f;
const float PWM_MAX = 75.0f;

const float KP = 1.5f;

const float MAX_PWM_STEP = 5.0f;

float pwmPercent = PWM_INITIAL;


// ============================================================
// BOTOES
// ============================================================

const uint32_t BUTTON_DEBOUNCE_MS = 150;

volatile bool onOffButtonEvent = false;
volatile bool calibrationButtonEvent = false;

uint32_t lastOnOffAccepted = 0;
uint32_t lastCalibrationAccepted = 0;


// ============================================================
// VALIDACAO
// ============================================================

const uint8_t MIN_VALID_SENSORS = 2;

const float OBSTRUCTION_RATIO = 0.35f;
const uint8_t OBSTRUCTION_CYCLES = 3;
const float OBSTRUCTION_MIN_REFERENCE = 2.0f;


// ============================================================
// TEMPORIZACAO
// ============================================================

const uint32_t SAMPLE_INTERVAL = 1000;
uint32_t lastSampleTime = 0;


// ============================================================
// ESTRUTURA DOS SENSORES
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
// AUXILIAR
// ============================================================

float clampFloat(
  float value,
  float minimum,
  float maximum
) {

  if (value < minimum) return minimum;
  if (value > maximum) return maximum;

  return value;
}


// ============================================================
// VERIFICA DISPOSITIVO I2C
// ============================================================

bool i2cDevicePresent(uint8_t address) {

  Wire.beginTransmission(address);

  return Wire.endTransmission() == 0;
}


// ============================================================
// AGUARDA DISPOSITIVO I2C
// ============================================================

bool waitForI2CDevice(
  uint8_t address,
  uint32_t timeoutMs
) {

  uint32_t start = millis();

  while (
    millis() - start < timeoutMs
  ) {

    if (
      i2cDevicePresent(address)
    ) {

      return true;
    }

    delay(20);
  }

  return false;
}


// ============================================================
// RESET FISICO DO TCA9548A
// ============================================================

void tcaHardwareReset() {

  Serial.println(
    "Reset fisico do TCA9548A..."
  );

  // RESET ativo em LOW
  digitalWrite(
    TCA_RESET_PIN,
    LOW
  );

  delay(
    TCA_RESET_PULSE_MS
  );

  // Libera o TCA
  digitalWrite(
    TCA_RESET_PIN,
    HIGH
  );

  delay(
    TCA_RESET_RECOVERY_MS
  );
}


// ============================================================
// TCA9548A
// ============================================================

bool tcaSelect(uint8_t channel) {

  if (channel > 7) {
    return false;
  }

  Wire.beginTransmission(TCA_ADDR);

  Wire.write(
    1 << channel
  );

  return Wire.endTransmission() == 0;
}


bool tcaDisableAll() {

  Wire.beginTransmission(TCA_ADDR);

  Wire.write(0x00);

  return Wire.endTransmission() == 0;
}


// ============================================================
// PREPARACAO ROBUSTA DO TCA
// ============================================================

bool initializeTCA() {

  Serial.println();

  Serial.println(
    "Inicializando TCA9548A..."
  );


  for (
    uint8_t attempt = 1;
    attempt <= TCA_INIT_RETRIES;
    attempt++
  ) {

    Serial.print(
      "Tentativa TCA "
    );

    Serial.print(attempt);

    Serial.print("/");

    Serial.println(
      TCA_INIT_RETRIES
    );


    // Faz reset real do multiplexador
    tcaHardwareReset();


    // Aguarda resposta em 0x71
    if (
      !waitForI2CDevice(
        TCA_ADDR,
        300
      )
    ) {

      Serial.println(
        "TCA nao respondeu."
      );

      delay(100);

      continue;
    }


    Serial.println(
      "TCA encontrado em 0x71."
    );


    // Garante todos os canais desligados
    if (
      !tcaDisableAll()
    ) {

      Serial.println(
        "Falha ao limpar canais do TCA."
      );

      delay(100);

      continue;
    }


    delay(20);


    Serial.println(
      "TCA inicializado corretamente."
    );


    return true;
  }


  Serial.println(
    "ERRO: TCA nao inicializado."
  );


  return false;
}


// ============================================================
// RECUPERACAO DO TCA
// ============================================================

bool recoverTCA() {

  Serial.println();

  Serial.println(
    "*** RECUPERANDO TCA ***"
  );


  tcaHardwareReset();


  if (
    !waitForI2CDevice(
      TCA_ADDR,
      500
    )
  ) {

    Serial.println(
      "Falha na recuperacao do TCA."
    );

    return false;
  }


  tcaDisableAll();

  delay(20);


  Serial.println(
    "TCA recuperado."
  );


  return true;
}


// ============================================================
// PWM
// ============================================================

void setPWM(float percent) {

  percent =
    clampFloat(
      percent,
      0.0f,
      100.0f
    );


  pwmPercent = percent;


  uint16_t duty =
    (uint16_t)(
      (percent / 100.0f)
      *
      PWM_DUTY_MAX
    );


  ledcWrite(
    PWM_PIN,
    duty
  );
}


// ============================================================
// REGIAO AZUL
// ============================================================

uint16_t getBluePeak(
  const uint16_t *r
) {

  uint16_t peak =
    r[AS7343_CHANNEL_F1];


  if (
    r[AS7343_CHANNEL_F2] > peak
  )
    peak = r[AS7343_CHANNEL_F2];


  if (
    r[AS7343_CHANNEL_FZ] > peak
  )
    peak = r[AS7343_CHANNEL_FZ];


  if (
    r[AS7343_CHANNEL_F3] > peak
  )
    peak = r[AS7343_CHANNEL_F3];


  if (
    r[AS7343_CHANNEL_F4] > peak
  )
    peak = r[AS7343_CHANNEL_F4];


  if (
    r[AS7343_CHANNEL_F5] > peak
  )
    peak = r[AS7343_CHANNEL_F5];


  return peak;
}


// ============================================================
// RAW -> IRRADIANCIA TEMPORARIA
// ============================================================

float rawToIrradiance(uint16_t raw) {

  float E =
    IRRADIANCE_MAX
    *
    (
      (float)raw
      /
      (float)RAW_SATURATION
    );


  if (
    E > IRRADIANCE_MAX
  )
    E = IRRADIANCE_MAX;


  return E;
}


// ============================================================
// INICIALIZACAO ROBUSTA DE UM SENSOR
// ============================================================

bool initializeSensor(
  uint8_t index
) {

  uint8_t channel =
    SENSOR_CHANNELS[index];


  Serial.println();

  Serial.print(
    "Inicializando Sensor S"
  );

  Serial.print(
    index + 1
  );

  Serial.print(
    " - TCA CH"
  );

  Serial.println(
    channel
  );


  for (
    uint8_t attempt = 1;
    attempt <= SENSOR_INIT_RETRIES;
    attempt++
  ) {

    Serial.print(
      "Tentativa "
    );

    Serial.print(attempt);

    Serial.print("/");

    Serial.println(
      SENSOR_INIT_RETRIES
    );


    // Garante nenhum canal anterior conectado
    if (
      !tcaDisableAll()
    ) {

      recoverTCA();
    }


    delay(10);


    // Seleciona canal
    if (
      !tcaSelect(channel)
    ) {

      Serial.println(
        "Falha ao selecionar TCA."
      );

      recoverTCA();

      continue;
    }


    delay(20);


    // Antes de chamar a biblioteca,
    // verifica se o sensor realmente apareceu
    if (
      !waitForI2CDevice(
        SENSOR_ADDR,
        300
      )
    ) {

      Serial.println(
        "Sensor 0x39 nao respondeu."
      );


      recoverTCA();

      delay(100);

      continue;
    }


    Serial.println(
      "Sensor 0x39 detectado."
    );


    if (
      sensors[index].begin(
        SENSOR_ADDR,
        &Wire
      )
    ) {

      sensors[index].setGain(
        SENSOR_GAIN
      );


      sensors[index].setATIME(
        SENSOR_ATIME
      );


      sensors[index].setASTEP(
        SENSOR_ASTEP
      );


      sensors[index].setSMUXMode(
        AS7343_SMUX_18CH
      );


      Serial.print(
        "Sensor S"
      );

      Serial.print(
        index + 1
      );

      Serial.print(
        " OK - TCA CH"
      );

      Serial.println(
        channel
      );


      return true;
    }


    Serial.println(
      "begin() do AS7343 falhou."
    );


    recoverTCA();

    delay(100);
  }


  Serial.print(
    "ERRO: Sensor S"
  );

  Serial.print(
    index + 1
  );

  Serial.println(
    " nao inicializado."
  );


  return false;
}


// ============================================================
// AQUISICAO
// ============================================================

bool readSensor(uint8_t index) {

  dataSensor[index].valid = false;
  dataSensor[index].saturated = false;
  dataSensor[index].obstructed = false;


  Serial.println();

  Serial.print("[S");

  Serial.print(index + 1);

  Serial.print(" / CH");

  Serial.print(
    SENSOR_CHANNELS[index]
  );

  Serial.println("]");


  if (
    !dataSensor[index].initialized
  ) {

    Serial.println(
      "ERRO: sensor nao inicializado."
    );

    return false;
  }


  // Seleciona canal
  if (
    !tcaSelect(
      SENSOR_CHANNELS[index]
    )
  ) {

    Serial.println(
      "Erro TCA. Tentando recuperar..."
    );


    if (
      !recoverTCA()
    ) {

      return false;
    }


    if (
      !tcaSelect(
        SENSOR_CHANNELS[index]
      )
    ) {

      return false;
    }
  }


  // Inicia medicao
  if (
    !sensors[index].startMeasurement()
  ) {

    Serial.println(
      "ERRO: startMeasurement()."
    );

    recoverTCA();

    return false;
  }


  uint32_t start =
    millis();


  while (
    !sensors[index].dataReady()
  ) {

    if (
      millis() - start > 1000
    ) {

      Serial.println(
        "ERRO: timeout dataReady()."
      );

      recoverTCA();

      return false;
    }


    delay(1);
  }


  if (
    !sensors[index].readAllChannels(
      dataSensor[index].channels
    )
  ) {

    Serial.println(
      "ERRO: readAllChannels()."
    );

    recoverTCA();

    return false;
  }


  dataSensor[index].blueRaw =
    getBluePeak(
      dataSensor[index].channels
    );


  dataSensor[index].irradiance =
    rawToIrradiance(
      dataSensor[index].blueRaw
    );


  dataSensor[index].saturated =
    dataSensor[index].blueRaw
    >=
    RAW_SAT_WARNING;


  dataSensor[index].valid =
    true;


  Serial.print(
    "RAW azul = "
  );

  Serial.println(
    dataSensor[index].blueRaw
  );


  return true;
}


// ============================================================
// OBSTRUCAO
// ============================================================

void detectObstructions() {

  for (
    uint8_t i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    if (!dataSensor[i].valid)
      continue;


    float otherSum = 0.0f;
    uint8_t otherCount = 0;


    for (
      uint8_t j = 0;
      j < NUM_SENSORS;
      j++
    ) {

      if (j == i)
        continue;


      if (dataSensor[j].valid) {

        otherSum +=
          dataSensor[j].irradiance;

        otherCount++;
      }
    }


    if (otherCount < 2) {

      dataSensor[i]
        .obstructionCounter = 0;

      continue;
    }


    float otherAverage =
      otherSum / otherCount;


    bool possibleObstruction =

      (
        otherAverage >
        OBSTRUCTION_MIN_REFERENCE
      )

      &&

      (
        dataSensor[i].irradiance
        <
        otherAverage
        *
        OBSTRUCTION_RATIO
      );


    if (possibleObstruction) {

      if (
        dataSensor[i].obstructionCounter
        <
        OBSTRUCTION_CYCLES
      ) {

        dataSensor[i]
          .obstructionCounter++;
      }

    }

    else {

      dataSensor[i]
        .obstructionCounter = 0;
    }


    if (
      dataSensor[i].obstructionCounter
      >=
      OBSTRUCTION_CYCLES
    ) {

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


  for (
    uint8_t i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    if (
      dataSensor[i].valid
    ) {

      sum +=
        dataSensor[i].irradiance;

      validSensors++;
    }
  }


  if (
    validSensors > 0
  ) {

    representativeIrradiance =
      sum / validSensors;
  }

  else {

    representativeIrradiance =
      0.0f;
  }
}


// ============================================================
// FAIXA
// ============================================================

bool irradianceInRange() {

  return

    representativeIrradiance
      >= IRRADIANCE_MIN

    &&

    representativeIrradiance
      <= IRRADIANCE_MAX_THERAPEUTIC;
}


// ============================================================
// CONTROLE
// ============================================================

void executeControlStep() {

  if (!controlActive)
    return;


  if (
    irradianceInRange()
  ) {

    controlActive = false;

    systemStatus =
      STATUS_CONTROL_DONE;


    Serial.println(
      "CONTROLE CONCLUIDO."
    );

    return;
  }


  float error =
    IRRADIANCE_REFERENCE
    -
    representativeIrradiance;


  float deltaPWM =
    KP * error;


  deltaPWM =
    clampFloat(
      deltaPWM,
      -MAX_PWM_STEP,
      MAX_PWM_STEP
    );


  float newPWM =
    pwmPercent
    +
    deltaPWM;


  if (
    newPWM < PWM_MIN
    ||
    newPWM > PWM_MAX
  ) {

    controlActive = false;

    systemStatus =
      STATUS_PWM_LIMIT;


    Serial.println(
      "LIMITE OPERACIONAL DO PWM."
    );

    return;
  }


  setPWM(
    newPWM
  );


  systemStatus =
    STATUS_CONTROLLING;


  Serial.print(
    "Controle -> erro="
  );

  Serial.print(
    error,
    2
  );

  Serial.print(
    " | PWM="
  );

  Serial.print(
    pwmPercent,
    1
  );

  Serial.println("%");
}


// ============================================================
// OLED
// ============================================================

void beginDisplayFrame() {

  if (!oledOK)
    return;


  tcaDisableAll();


  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    0,
    0
  );
}


void drawMonitoringPage() {

  beginDisplayFrame();

  if (!oledOK)
    return;


  if (
    displayPage == 0
  ) {

    display.println(
      "MONITORAMENTO"
    );

    display.println(
      "----------------"
    );


    display.print(
      "Erep: "
    );

    display.print(
      representativeIrradiance,
      1
    );

    display.println(
      " uW/cm2/nm"
    );


    display.print(
      "PWM : "
    );

    display.print(
      pwmPercent,
      1
    );

    display.println("%");


    display.print(
      "Sens: "
    );

    display.print(
      validSensors
    );

    display.print("/");

    display.println(
      NUM_SENSORS
    );


    if (
      irradianceInRange()
    ) {

      display.println(
        "STATUS: FAIXA OK"
      );

    }

    else {

      display.println(
        "FORA FAIXA"
      );

      display.println(
        "BTN CAL ou C"
      );
    }

  }

  else if (
    displayPage == 1
  ) {

    display.println(
      "IRRADIANCIA/SENSOR"
    );

    display.println(
      "------------------"
    );


    for (
      uint8_t i = 0;
      i < NUM_SENSORS;
      i++
    ) {

      display.print("S");

      display.print(i + 1);

      display.print(": ");


      if (
        dataSensor[i].valid
      ) {

        display.print(
          dataSensor[i].irradiance,
          1
        );


        if (
          dataSensor[i].saturated
        )
          display.print(" SAT");

      }

      else if (
        dataSensor[i].obstructed
      ) {

        display.print("OBSTR");

      }

      else {

        display.print("ERRO");
      }


      display.println();
    }

  }

  else {

    display.println(
      "RAW - REGIAO AZUL"
    );

    display.println(
      "----------------"
    );


    for (
      uint8_t i = 0;
      i < NUM_SENSORS;
      i++
    ) {

      display.print("S");

      display.print(i + 1);

      display.print(": ");

      display.println(
        dataSensor[i].blueRaw
      );
    }


    display.print("SAT=");

    display.println(
      RAW_SATURATION
    );
  }


  display.display();
}


void drawStatus() {

  if (!oledOK)
    return;


  if (
    systemStatus ==
    STATUS_MONITORING
  ) {

    drawMonitoringPage();

    return;
  }


  beginDisplayFrame();


  switch (
    systemStatus
  ) {

    case STATUS_INITIALIZING:

      display.println(
        "INICIALIZANDO..."
      );

      display.println(
        "TCA / SENSORES"
      );

      break;


    case STATUS_OUT_OF_RANGE:

      display.println(
        "*** ALERTA ***"
      );

      display.println(
        "IRRADIANCIA"
      );

      display.println(
        "FORA DA FAIXA"
      );

      display.print("E=");

      display.println(
        representativeIrradiance,
        1
      );

      display.println(
        "BTN CAL ou C"
      );

      break;


    case STATUS_CONTROLLING:

      display.println(
        "CONTROLE ATIVO"
      );

      display.print(
        "Erep: "
      );

      display.println(
        representativeIrradiance,
        1
      );

      display.print(
        "PWM: "
      );

      display.print(
        pwmPercent,
        1
      );

      display.println("%");

      break;


    case STATUS_CONTROL_DONE:

      display.println(
        "CONTROLE"
      );

      display.println(
        "CONCLUIDO"
      );

      display.println(
        "FAIXA OK"
      );

      break;


    case STATUS_PWM_LIMIT:

      display.println(
        "*** ALERTA ***"
      );

      display.println(
        "LIMITE PWM"
      );

      display.println(
        "CONTROLE ENCERRADO"
      );

      break;


    case STATUS_SENSOR_FAULT:

      display.println(
        "*** FALHA ***"
      );

      display.println(
        "SENSORES"
      );

      display.println(
        "INSUFICIENTES"
      );

      display.print(
        "Validos: "
      );

      display.print(
        validSensors
      );

      display.print("/");

      display.println(
        NUM_SENSORS
      );

      break;


    case STATUS_OFF:

      display.println(
        "SISTEMA DESLIGADO"
      );

      display.println(
        "PWM = 0%"
      );

      display.println(
        "BTN ON/OFF"
      );

      break;


    default:

      break;
  }


  display.display();
}


// ============================================================
// SERIAL
// ============================================================

void printMeasurements() {

  Serial.println();

  Serial.println(
    "=============================================="
  );


  for (
    uint8_t i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    Serial.print("S");

    Serial.print(i + 1);

    Serial.print(" CH");

    Serial.print(
      SENSOR_CHANNELS[i]
    );

    Serial.print(
      " | RAW="
    );

    Serial.print(
      dataSensor[i].blueRaw
    );

    Serial.print(
      " | E="
    );

    Serial.print(
      dataSensor[i].irradiance,
      2
    );


    if (
      dataSensor[i].saturated
    )
      Serial.print(
        " | SATURADO"
      );


    if (
      dataSensor[i].obstructed
    )
      Serial.print(
        " | OBSTRUIDO"
      );


    if (
      !dataSensor[i].valid
    )
      Serial.print(
        " | INVALIDO"
      );


    Serial.println();
  }


  Serial.print(
    "E REPRESENTATIVA = "
  );

  Serial.println(
    representativeIrradiance,
    2
  );


  Serial.print(
    "PWM = "
  );

  Serial.print(
    pwmPercent,
    1
  );

  Serial.println("%");


  Serial.print(
    "Sensores validos = "
  );

  Serial.print(
    validSensors
  );

  Serial.print("/");

  Serial.println(
    NUM_SENSORS
  );
}


// ============================================================
// AQUISICAO COMPLETA
// ============================================================

void acquireSystem() {

  for (
    uint8_t i = 0;
    i < NUM_SENSORS;
    i++
  ) {

    readSensor(i);
  }


  detectObstructions();

  calculateRepresentativeIrradiance();

  printMeasurements();


  if (
    validSensors
    <
    MIN_VALID_SENSORS
  ) {

    controlActive = false;

    systemStatus =
      STATUS_SENSOR_FAULT;


    drawStatus();

    return;
  }


  if (
    controlActive
  ) {

    executeControlStep();

    drawStatus();

    return;
  }


  if (
    irradianceInRange()
  ) {

    systemStatus =
      STATUS_MONITORING;

  }

  else {

    systemStatus =
      STATUS_OUT_OF_RANGE;
  }


  drawStatus();
}


// ============================================================
// ON / OFF
// ============================================================

void toggleSystem() {

  if (
    systemOn
  ) {

    systemOn = false;

    controlActive = false;

    setPWM(0.0f);

    systemStatus =
      STATUS_OFF;


    Serial.println(
      "SISTEMA DESLIGADO - PWM 0%"
    );

  }

  else {

    systemOn = true;

    controlActive = false;

    setPWM(
      PWM_INITIAL
    );

    systemStatus =
      STATUS_MONITORING;

    lastSampleTime = 0;


    Serial.println(
      "SISTEMA LIGADO"
    );
  }


  drawStatus();
}


// ============================================================
// CALIBRACAO
// ============================================================

void requestCalibration() {

  Serial.println(
    "SOLICITACAO DE CALIBRACAO"
  );


  if (
    !systemOn
  ) {

    Serial.println(
      "Sistema desligado."
    );

    return;
  }


  if (
    validSensors
    <
    MIN_VALID_SENSORS
  ) {

    Serial.println(
      "Sensores insuficientes."
    );

    return;
  }


  if (
    irradianceInRange()
  ) {

    Serial.println(
      "Irradiancia ja esta na faixa."
    );

    return;
  }


  controlActive = true;

  systemStatus =
    STATUS_CONTROLLING;


  Serial.println(
    "CONTROLE AUTORIZADO."
  );


  drawStatus();
}


// ============================================================
// INTERRUPCOES
// ============================================================

void IRAM_ATTR onOffButtonISR() {

  onOffButtonEvent = true;
}


void IRAM_ATTR calibrationButtonISR() {

  calibrationButtonEvent = true;
}


// ============================================================
// BOTOES
// ============================================================

void handleButtons() {

  uint32_t now =
    millis();


  if (
    onOffButtonEvent
  ) {

    onOffButtonEvent =
      false;


    if (
      now -
      lastOnOffAccepted
      >=
      BUTTON_DEBOUNCE_MS
    ) {

      lastOnOffAccepted =
        now;


      Serial.println(
        "GPIO25 -> ON/OFF"
      );


      toggleSystem();
    }
  }


  if (
    calibrationButtonEvent
  ) {

    calibrationButtonEvent =
      false;


    if (
      now -
      lastCalibrationAccepted
      >=
      BUTTON_DEBOUNCE_MS
    ) {

      lastCalibrationAccepted =
        now;


      Serial.println(
        "GPIO26 -> CALIBRACAO"
      );


      requestCalibration();
    }
  }
}


// ============================================================
// COMANDOS SERIAL
// ============================================================

void handleSerial() {

  while (
    Serial.available()
  ) {

    char command =
      Serial.read();


    if (
      command == '\n'
      ||
      command == '\r'
      ||
      command == ' '
    )
      continue;


    if (
      command >= 'A'
      &&
      command <= 'Z'
    ) {

      command =
        command - 'A' + 'a';
    }


    if (
      command == 'o'
    ) {

      toggleSystem();

    }

    else if (
      command == 'c'
    ) {

      requestCalibration();

    }

    else if (
      command == 'n'
    ) {

      controlActive = false;

      Serial.println(
        "Controle cancelado."
      );

    }

    else if (
      command == 'a'
    ) {

      displayPage++;

      if (
        displayPage > 2
      )
        displayPage = 0;


      drawMonitoringPage();

    }

    else if (
      command == 'v'
    ) {

      if (
        displayPage == 0
      )
        displayPage = 2;

      else
        displayPage--;


      drawMonitoringPage();

    }

    else if (
      command == 'h'
    ) {

      Serial.println(
        "O = ON/OFF"
      );

      Serial.println(
        "C = calibracao"
      );

      Serial.println(
        "N = cancelar"
      );

      Serial.println(
        "A = avancar tela"
      );

      Serial.println(
        "V = voltar tela"
      );

      Serial.println(
        "H = ajuda"
      );
    }
  }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  /*
    ----------------------------------------------------------
    PASSO 1

    Mantem LEDs desligados e TCA em RESET
    IMEDIATAMENTE ao iniciar o ESP32.
    ----------------------------------------------------------
  */

  pinMode(
    PWM_PIN,
    OUTPUT
  );

  digitalWrite(
    PWM_PIN,
    LOW
  );


  pinMode(
    TCA_RESET_PIN,
    OUTPUT
  );

  // Mantem TCA fisicamente resetado
  // enquanto as fontes estabilizam

  digitalWrite(
    TCA_RESET_PIN,
    LOW
  );


  // ----------------------------------------------------------
  // BOTOES
  // ----------------------------------------------------------

  pinMode(
    BUTTON_ON_OFF,
    INPUT_PULLUP
  );

  pinMode(
    BUTTON_CALIBRATION,
    INPUT_PULLUP
  );


  // ----------------------------------------------------------
  // SERIAL
  // ----------------------------------------------------------

  Serial.begin(
    115200
  );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    " SISTEMA TCC - COLD START"
  );

  Serial.println(
    "========================================"
  );


  Serial.println(
    "Aguardando estabilizacao da alimentacao..."
  );


  /*
    ----------------------------------------------------------
    PASSO 2

    Espera a alimentacao do MP1584,
    ESP32, TCA e sensores estabilizar.
    ----------------------------------------------------------
  */

  delay(
    POWER_STABILIZATION_MS
  );


  // ----------------------------------------------------------
  // I2C DO ESP32
  // ----------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );


  Wire.setClock(
    100000
  );


  /*
    ----------------------------------------------------------
    PASSO 3

    Libera o RESET somente depois que
    alimentacao e I2C ja estao estabilizados.
    ----------------------------------------------------------
  */

  digitalWrite(
    TCA_RESET_PIN,
    HIGH
  );


  delay(
    TCA_RESET_RECOVERY_MS
  );


  /*
    ----------------------------------------------------------
    PASSO 4

    Reinicializa novamente o TCA e verifica
    se ele realmente responde.
    ----------------------------------------------------------
  */

  bool tcaOK =
    initializeTCA();


  if (
    !tcaOK
  ) {

    Serial.println(
      "ATENCAO: falha critica no TCA."
    );
  }


  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  oledOK =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    );


  if (
    oledOK
  ) {

    systemStatus =
      STATUS_INITIALIZING;

    drawStatus();


    Serial.println(
      "OLED OK."
    );

  }

  else {

    Serial.println(
      "OLED nao encontrado."
    );
  }


  // ----------------------------------------------------------
  // PWM
  // ----------------------------------------------------------

  if (
    !ledcAttach(
      PWM_PIN,
      PWM_FREQ,
      PWM_RESOLUTION
    )
  ) {

    Serial.println(
      "ERRO PWM."
    );
  }


  // Mantem LEDs desligados
  // durante inicializacao dos sensores

  setPWM(
    0.0f
  );


  /*
    ----------------------------------------------------------
    PASSO 5

    Inicializa cada sensor com varias tentativas.
    ----------------------------------------------------------
  */

  for (
    uint8_t i = 0;
    i < NUM_SENSORS;
    i++
  ) {

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
  // INTERRUPCOES DOS BOTOES
  // ----------------------------------------------------------

  attachInterrupt(
    digitalPinToInterrupt(
      BUTTON_ON_OFF
    ),
    onOffButtonISR,
    FALLING
  );


  attachInterrupt(
    digitalPinToInterrupt(
      BUTTON_CALIBRATION
    ),
    calibrationButtonISR,
    FALLING
  );


  /*
    ----------------------------------------------------------
    PASSO 6

    So liga a malha de LEDs DEPOIS
    de toda inicializacao.
    ----------------------------------------------------------
  */

  setPWM(
    PWM_INITIAL
  );


  Serial.println();

  Serial.println(
    "========================================"
  );

  Serial.println(
    " INICIALIZACAO CONCLUIDA"
  );

  Serial.println(
    "========================================"
  );


  Serial.print(
    "S1: "
  );

  Serial.println(
    dataSensor[0].initialized
    ? "OK"
    : "ERRO"
  );


  Serial.print(
    "S2: "
  );

  Serial.println(
    dataSensor[1].initialized
    ? "OK"
    : "ERRO"
  );


  Serial.print(
    "S3: "
  );

  Serial.println(
    dataSensor[2].initialized
    ? "OK"
    : "ERRO"
  );


  Serial.print(
    "PWM inicial = "
  );

  Serial.print(
    PWM_INITIAL
  );

  Serial.println("%");


  Serial.println(
    "GPIO27 = RESET TCA9548A"
  );


  systemStatus =
    STATUS_MONITORING;


  lastSampleTime =
    millis();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  handleButtons();

  handleSerial();


  if (
    !systemOn
  ) {

    delay(10);

    return;
  }


  if (
    millis() -
    lastSampleTime
    >=
    SAMPLE_INTERVAL
  ) {

    lastSampleTime =
      millis();


    acquireSystem();
  }
}