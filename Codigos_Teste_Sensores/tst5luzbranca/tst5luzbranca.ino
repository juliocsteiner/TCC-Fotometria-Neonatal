
/*
  TCC - ENSAIO SEM COMPUTADOR

  ESP32 + ISO1540 + TCA9548A
  5 sensores AS7341 / AS7343

  BOTAO SELECIONAR:
  1o toque: 30 leituras BASE
  2o toque: PWM 25% e 100 leituras
  Toque durante coleta: interromper

  SALVAMENTO:
  - Memoria flash NVS / Preferences
  - Checkpoint a cada 10 ciclos
  - Resultados preservados sem energia

  RECUPERACAO POSTERIOR:
  Conectar computador, abrir Monitor
  Serial em 115200 e enviar P.

  Sao salvas estatisticas do canal
  azul, nao todas as amostras RAW.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_AS7343.h>
#include <math.h>

// =====================================
// PINAGEM
// =====================================

constexpr uint8_t PIN_SDA = 17;
constexpr uint8_t PIN_SCL = 16;
constexpr uint8_t PIN_RESET_MUX = 19;

constexpr uint8_t PIN_PWM = 32;
constexpr uint8_t PIN_SELECIONAR = 18;

constexpr uint8_t END_MUX = 0x70;
constexpr uint8_t END_SENSOR = 0x39;

constexpr uint8_t N = 5;

// Mapeamento do ultimo teste funcional
const uint8_t CH[N] = {2, 3, 5, 6, 7};

// =====================================
// PARAMETROS
// =====================================

constexpr uint16_t LEITURAS_BASE = 30;
constexpr uint16_t LEITURAS_LED = 100;

constexpr uint8_t PWM_LED = 25;
constexpr uint16_t LIMIAR_SAT = 17990;

constexpr uint8_t ATIME_VAL = 29;
constexpr uint16_t ASTEP_VAL = 599;

// Identificacao dos dados armazenados
constexpr uint32_t MARCA = 0x54434332;
constexpr uint16_t VERSAO = 1;

// =====================================
// ESTRUTURAS
// =====================================

struct Estatistica {
  uint32_t ok;
  uint32_t nAzul;
  uint32_t zeros;
  uint32_t satAzul;
  uint32_t satAlgum;

  uint16_t minimo;
  uint16_t maximo;

  double soma;
  double soma2;
};

struct Registro {
  uint32_t marca;
  uint16_t versao;

  uint8_t etapa;
  uint8_t pwm;

  uint8_t canal[N];
  uint16_t modelo[N];

  uint16_t ciclosBase;
  uint16_t ciclosLed;

  Estatistica base[N];
  Estatistica led[N];
};

// Estado gravado na memoria
constexpr uint8_t ET_BASE = 1;
constexpr uint8_t ET_BASE_OK = 2;
constexpr uint8_t ET_LED = 3;
constexpr uint8_t ET_CONCLUIDO = 4;
constexpr uint8_t ET_ABORTADO = 5;

// =====================================
// SENSORES
// =====================================

Adafruit_AS7341 sensor41[N];
Adafruit_AS7343 sensor43[N];

uint16_t modeloAtual[N] = {0};

bool valido[N] = {
  false, false, false, false, false
};

// =====================================
// MEMORIA E CONTROLE
// =====================================

Preferences memoria;
Registro reg = {};

bool memoriaOK = false;
bool pwmOK = false;

// 0 = pronto
// 1 = aguardando segundo toque
// 2 = coletando
uint8_t estado = 0;

// =====================================
// BOTAO
// =====================================

volatile bool flagToque = false;

uint32_t ultimoToque = 0;

void IRAM_ATTR isrSelecionar() {
  flagToque = true;
}

bool toqueSelecionar() {

  noInterrupts();
  bool houve = flagToque;
  flagToque = false;
  interrupts();

  if (!houve) return false;

  uint32_t agora = millis();

  if (agora - ultimoToque < 250) {
    return false;
  }

  ultimoToque = agora;

  return true;
}

void limparToque() {
  noInterrupts();
  flagToque = false;
  interrupts();
}

// =====================================
// PWM
// =====================================

void pwmDesligado() {

  if (pwmOK) {
    ledcWrite(PIN_PWM, 0);
  } else {
    digitalWrite(PIN_PWM, LOW);
  }
}

bool pwmAtivar() {

  if (!pwmOK) return false;

  uint32_t duty =
    ((uint32_t)PWM_LED * 255UL + 50UL)
    / 100UL;

  return ledcWrite(PIN_PWM, duty);
}

// =====================================
// MULTIPLEXADOR
// =====================================

bool mux(int canal) {

  if (canal < -1 || canal > 7) {
    return false;
  }

  Wire.beginTransmission(END_MUX);

  if (canal == -1) {
    Wire.write((uint8_t)0);
  } else {
    Wire.write((uint8_t)(1U << canal));
  }

  return Wire.endTransmission() == 0;
}

bool temACK() {

  Wire.beginTransmission(END_SENSOR);

  return Wire.endTransmission() == 0;
}

const char* nome(uint16_t m) {

  if (m == 7341) return "AS7341";
  if (m == 7343) return "AS7343";

  return "NA";
}

// =====================================
// IDENTIFICACAO AUTOMATICA
// =====================================

bool inicializarUm(uint8_t i) {

  valido[i] = false;
  modeloAtual[i] = 0;

  if (!mux(CH[i])) return false;

  delay(20);

  if (!temACK()) return false;

  // Testar AS7341
  if (sensor41[i].begin(
        END_SENSOR, &Wire)) {

    if (!sensor41[i].setATIME(ATIME_VAL) ||
        !sensor41[i].setASTEP(ASTEP_VAL) ||
        !sensor41[i].setGain(
          AS7341_GAIN_32X)) {

      return false;
    }

    sensor41[i].enableLED(false);
    modeloAtual[i] = 7341;

  // Testar AS7343
  } else if (sensor43[i].begin(
               END_SENSOR, &Wire)) {

    if (!sensor43[i].setATIME(ATIME_VAL) ||
        !sensor43[i].setASTEP(ASTEP_VAL) ||
        !sensor43[i].setGain(
          AS7343_GAIN_32X) ||
        !sensor43[i].setSMUXMode(
          AS7343_SMUX_18CH)) {

      return false;
    }

    sensor43[i].enableLED(false);
    modeloAtual[i] = 7343;

  } else {
    return false;
  }

  valido[i] = true;
  return true;
}

void inicializarTodos() {

  pwmDesligado();

  for (uint8_t i = 0; i < N; i++) {

    bool ok = false;

    for (uint8_t t = 0;
         t < 3 && !ok;
         t++) {

      ok = inicializarUm(i);

      if (!ok) delay(100);
    }

    Serial.printf(
      "# S%u;CH%u;%s;%s\n",
      i + 1,
      CH[i],
      nome(modeloAtual[i]),
      ok ? "OK" : "FALHA"
    );
  }

  mux(-1);
}

bool todosValidos() {

  for (uint8_t i = 0; i < N; i++) {

    if (!valido[i]) return false;
  }

  return true;
}

// =====================================
// LEITURA DO CANAL AZUL
// =====================================

bool lerAzul(
  uint8_t i,
  uint16_t &azul,
  bool &satAlgum
) {

  uint16_t bruto[18] = {0};

  if (modeloAtual[i] == 7341) {

    uint16_t b[12] = {0};

    if (!sensor41[i].readAllChannels(b)) {
      return false;
    }

    // AS7341: F2 = 445 nm
    azul = b[1];

    for (uint8_t j = 0; j < 12; j++) {

      if (b[j] >= LIMIAR_SAT) {
        satAlgum = true;
      }
    }

  } else if (modeloAtual[i] == 7343) {

    if (!sensor43[i].readAllChannels(bruto)) {
      return false;
    }

    // AS7343: FZ = 450 nm
    azul = bruto[0];

    for (uint8_t j = 0; j < 18; j++) {

      if (bruto[j] >= LIMIAR_SAT) {
        satAlgum = true;
      }
    }

  } else {
    return false;
  }

  return true;
}

// =====================================
// SALVAR NA FLASH
// =====================================

bool gravar() {

  if (!memoriaOK) return false;

  reg.marca = MARCA;
  reg.versao = VERSAO;

  size_t escritos = memoria.putBytes(
    "ultimo",
    &reg,
    sizeof(reg)
  );

  if (escritos != sizeof(reg)) {

    pwmDesligado();

    Serial.println(
      "# ERRO_GRAVACAO_FLASH"
    );

    return false;
  }

  return true;
}

// =====================================
// RECUPERAR DA FLASH
// =====================================

bool carregar() {

  if (!memoriaOK) return false;

  if (memoria.getBytesLength("ultimo")
      != sizeof(reg)) {

    return false;
  }

  Registro lido = {};

  size_t bytes = memoria.getBytes(
    "ultimo",
    &lido,
    sizeof(lido)
  );

  if (bytes != sizeof(lido)) {
    return false;
  }

  if (lido.marca != MARCA ||
      lido.versao != VERSAO) {

    return false;
  }

  reg = lido;

  return true;
}

// =====================================
// ESTATISTICAS
// =====================================

void acumular(
  uint8_t fase,
  uint8_t i,
  uint16_t azul,
  bool saturouAlgum
) {

  Estatistica &e =
    (fase == 0)
    ? reg.base[i]
    : reg.led[i];

  e.ok++;

  if (azul == 0) e.zeros++;

  if (saturouAlgum) {
    e.satAlgum++;
  }

  if (azul >= LIMIAR_SAT) {
    e.satAzul++;
    return;
  }

  if (e.nAzul == 0 ||
      azul < e.minimo) {

    e.minimo = azul;
  }

  if (e.nAzul == 0 ||
      azul > e.maximo) {

    e.maximo = azul;
  }

  e.nAzul++;

  e.soma += azul;
  e.soma2 += (double)azul * azul;
}

double calcularMedia(
  uint8_t fase,
  uint8_t i
) {

  const Estatistica &e =
    (fase == 0)
    ? reg.base[i]
    : reg.led[i];

  if (!e.nAzul) return NAN;

  return e.soma / e.nAzul;
}

double calcularDP(
  uint8_t fase,
  uint8_t i
) {

  const Estatistica &e =
    (fase == 0)
    ? reg.base[i]
    : reg.led[i];

  if (e.nAzul < 2) return NAN;

  double n = e.nAzul;

  double variancia =
    (e.soma2 - e.soma * e.soma / n)
    / (n - 1.0);

  if (variancia < 0) {
    variancia = 0;
  }

  return sqrt(variancia);
}

// =====================================
// IMPRIMIR RESULTADO ARMAZENADO
// =====================================

void imprimirResumo() {

  if (reg.marca != MARCA ||
      reg.versao != VERSAO) {

    Serial.println(
      "# NAO_HA_ENSAIO_SALVO"
    );

    return;
  }

  Serial.println();
  Serial.println(
    "# RESULTADO_SALVO_NA_FLASH"
  );

  Serial.printf(
    "# estado=%u;BASE=%u/%u;"
    "LED25=%u/%u;PWM=%u%%\n",
    reg.etapa,
    reg.ciclosBase,
    LEITURAS_BASE,
    reg.ciclosLed,
    LEITURAS_LED,
    reg.pwm
  );

  Serial.println(
    "etapa;sensor;CH;modelo;"
    "leituras_OK;n_azul;zeros_azul;"
    "sat_azul;sat_qualquer;"
    "min;max;media;DP;CV_percentual"
  );

  for (uint8_t fase = 0;
       fase < 2;
       fase++) {

    for (uint8_t i = 0; i < N; i++) {

      const Estatistica &e =
        fase == 0
        ? reg.base[i]
        : reg.led[i];

      Serial.printf(
        "%s;S%u;%u;%s;"
        "%lu;%lu;%lu;%lu;%lu;",
        fase == 0 ? "BASE" : "LED25",
        i + 1,
        reg.canal[i],
        nome(reg.modelo[i]),
        (unsigned long)e.ok,
        (unsigned long)e.nAzul,
        (unsigned long)e.zeros,
        (unsigned long)e.satAzul,
        (unsigned long)e.satAlgum
      );

      if (!e.nAzul) {
        Serial.println(
          "NA;NA;NA;NA;NA"
        );
        continue;
      }

      double m = calcularMedia(fase, i);
      double dp = calcularDP(fase, i);

      Serial.printf(
        "%u;%u;%.3f;",
        e.minimo,
        e.maximo,
        m
      );

      if (isnan(dp)) {

        Serial.println("NA;NA");

      } else if (m <= 0) {

        Serial.printf(
          "%.3f;NA\n", dp
        );

      } else {

        Serial.printf(
          "%.3f;%.3f\n",
          dp,
          100.0 * dp / m
        );
      }
    }
  }

  Serial.println(
    "# sensor;delta_MEDIA_LED_MENOS_BASE"
  );

  for (uint8_t i = 0; i < N; i++) {

    Serial.printf("# S%u;", i + 1);

    if (reg.etapa != ET_CONCLUIDO ||
        !reg.base[i].nAzul ||
        !reg.led[i].nAzul ||
        reg.led[i].satAzul) {

      Serial.println("NA");

    } else {

      double delta =
        calcularMedia(1, i) -
        calcularMedia(0, i);

      Serial.printf("%.3f\n", delta);
    }
  }

  Serial.println(
    "# FIM_RESULTADO_SALVO"
  );
}

// =====================================
// CANCELAMENTO
// =====================================

bool cancelarSolicitado() {

  if (toqueSelecionar()) return true;

  while (Serial.available()) {

    char c = Serial.read();

    if (c == 'X' ||
        c == 'x' ||
        c == '0') {

      return true;
    }
  }

  return false;
}

void interromper() {

  pwmDesligado();
  mux(-1);

  reg.etapa = ET_ABORTADO;
  gravar();

  estado = 0;
  limparToque();

  Serial.println(
    "# ENSAIO_INTERROMPIDO;PWM=0"
  );
}

// =====================================
// COLETA DE UMA ETAPA
// =====================================

bool amostrarEtapa(
  uint8_t fase,
  uint16_t quantidade
) {

  for (uint16_t ciclo = 1;
       ciclo <= quantidade;
       ciclo++) {

    for (uint8_t i = 0; i < N; i++) {

      if (cancelarSolicitado()) {
        interromper();
        return false;
      }

      if (!mux(CH[i]) || !temACK()) {

        Serial.printf(
          "# SEM_ACK_OU_MUX;S%u;CH%u\n",
          i + 1,
          CH[i]
        );

        interromper();
        return false;
      }

      uint16_t azul = 0;
      bool saturouQualquer = false;

      if (!lerAzul(
            i, azul, saturouQualquer)) {

        Serial.printf(
          "# ERRO_LEITURA;S%u\n",
          i + 1
        );

        interromper();
        return false;
      }

      acumular(
        fase,
        i,
        azul,
        saturouQualquer
      );
    }

    mux(-1);

    if (fase == 0) {
      reg.ciclosBase = ciclo;
    } else {
      reg.ciclosLed = ciclo;
    }

    // Grava a cada dez ciclos
    if (ciclo % 10 == 0) {

      if (!gravar()) {
        interromper();
        return false;
      }
    }

    delay(250);
  }

  return true;
}

// =====================================
// PRIMEIRO TOQUE: BASE
// =====================================

void primeiraPressao() {

  if (!memoriaOK ||
      !pwmOK ||
      !todosValidos()) {

    Serial.println(
      "# NAO_PRONTO:"
      " memoria/PWM/sensores"
    );

    return;
  }

  pwmDesligado();

  // Novo ensaio substitui o anterior
  reg = {};

  reg.etapa = ET_BASE;
  reg.pwm = PWM_LED;

  for (uint8_t i = 0; i < N; i++) {
    reg.canal[i] = CH[i];
    reg.modelo[i] = modeloAtual[i];
  }

  if (!gravar()) return;

  Serial.println(
    "# COLETANDO_BASE;LED=0%;30_CICLOS"
  );

  estado = 2;

  if (!amostrarEtapa(
        0, LEITURAS_BASE)) {

    return;
  }

  reg.etapa = ET_BASE_OK;

  if (!gravar()) {
    interromper();
    return;
  }

  pwmDesligado();

  estado = 1;
  limparToque();

  Serial.println(
    "# BASE_SALVA;"
    "PRESSIONE_SELECIONAR_PARA_LED25"
  );
}

// =====================================
// SEGUNDO TOQUE: LEDs
// =====================================

void segundaPressao() {

  if (!todosValidos() || !pwmOK) {
    interromper();
    return;
  }

  reg.etapa = ET_LED;

  if (!gravar()) {
    interromper();
    return;
  }

  estado = 2;
  limparToque();

  if (!pwmAtivar()) {
    interromper();
    return;
  }

  Serial.println(
    "# PWM=25%;"
    "AGUARDANDO_ESTABILIZACAO"
  );

  // Estabilizacao
  for (uint8_t i = 0; i < 15; i++) {

    delay(100);

    if (cancelarSolicitado()) {
      interromper();
      return;
    }
  }

  Serial.println(
    "# COLETANDO_LED;100_CICLOS"
  );

  if (!amostrarEtapa(
        1, LEITURAS_LED)) {

    return;
  }

  pwmDesligado();
  mux(-1);

  reg.etapa = ET_CONCLUIDO;

  if (!gravar()) {
    interromper();
    return;
  }

  estado = 0;
  limparToque();

  Serial.println(
    "# ENSAIO_CONCLUIDO;"
    "PWM=0;DADOS_SALVOS"
  );

  imprimirResumo();
}

// =====================================
// SETUP
// =====================================

void setup() {

  Serial.begin(115200);

  // PWM inicialmente desligado
  digitalWrite(PIN_PWM, LOW);
  pinMode(PIN_PWM, OUTPUT);

  pwmOK = ledcAttach(
    PIN_PWM, 500, 8
  );

  pwmDesligado();

  // RESET do multiplexador liberado
  digitalWrite(PIN_RESET_MUX, LOW);
  pinMode(PIN_RESET_MUX, OUTPUT);

  // Botao SELECIONAR
  pinMode(
    PIN_SELECIONAR,
    INPUT_PULLUP
  );

  attachInterrupt(
    digitalPinToInterrupt(PIN_SELECIONAR),
    isrSelecionar,
    FALLING
  );

  // I2C
  Wire.begin(
    PIN_SDA,
    PIN_SCL,
    100000
  );

  Wire.setTimeOut(100);

  // Inicializar memoria nao volatil
  memoriaOK = memoria.begin(
    "tcc_led",
    false
  );

  delay(1200);

  bool existe = carregar();

  Serial.println(
    "# TCC: MODO SEM COMPUTADOR"
  );

  if (existe) {
    imprimirResumo();
  } else {
    Serial.println(
      "# SEM_RESULTADO_ANTERIOR"
    );
  }

  // Inicializar sensores
  if (!mux(-1)) {

    Serial.println("# ERRO_MUX");

  } else {

    inicializarTodos();
  }

  limparToque();

  Serial.println(
    "# BOTAO: 1o BASE; 2o LED25"
  );

  Serial.println(
    "# TOQUE DURANTE COLETA CANCELA"
  );

  Serial.println(
    "# SERIAL: P=imprimir; "
    "R=reiniciar sensores"
  );
}

// =====================================
// LOOP
// =====================================

void loop() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == 'P' || c == 'p') {

      imprimirResumo();
    }

    if (c == 'R' || c == 'r') {

      if (estado == 0) {
        inicializarTodos();
      }
    }
  }

  if (toqueSelecionar()) {

    if (estado == 0) {

      primeiraPressao();

    } else if (estado == 1) {

      segundaPressao();
    }
  }

  delay(10);
}
