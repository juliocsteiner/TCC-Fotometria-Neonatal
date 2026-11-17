#include <Preferences.h>
#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ctype.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_AS7341.h>
#include <SparkFun_AS7343.h>

struct BlocoLeituras;
// ---------------- PINAGEM DA PCB (PRESERVADA DO SKETCH ANTERIOR) ----------------
constexpr uint8_t I2C_SDA = 17;          // TX2 como SDA
constexpr uint8_t I2C_SCL = 16;          // RX2 como SCL
constexpr uint8_t MUX_RESET = 19;        // rede RST: GPIO19 -> PC817 -> RST do MUX
constexpr uint8_t SENSOR_ADDR = 0x39;    // Ambos usam endereco I2C 0x39

constexpr uint8_t TFT_CS   = 26;
constexpr uint8_t TFT_DC   = 33;
constexpr uint8_t TFT_RST  = 13;
constexpr uint8_t TFT_MOSI = 14;
constexpr uint8_t TFT_SCK  = 25;
constexpr uint8_t TFT_MISO = 36;
constexpr uint8_t TFT_LED  = 27;
// Pinos dos botoes conforme ESQUEMATICO fornecido (J2 em comum no GND).
constexpr uint8_t BOTAO_SELECIONAR = 18;
constexpr uint8_t BOTAO_VOLTAR     = 21; 
constexpr uint8_t BOTAO_AVANCAR    = 22; 
int eventoSelecionar() {
  static bool anterior = false, estavel = false, longoEnviado = false;
  static uint32_t mudou = 0, inicio = 0;
  const uint32_t agora = millis();
  const bool pressionado = digitalRead(BOTAO_SELECIONAR) == LOW;
  if (pressionado != anterior) {
    anterior = pressionado;
    mudou = agora;
  }
  if (agora - mudou >= 40 && estavel != pressionado) {
    estavel = pressionado;
    if (estavel) {
      inicio = mudou;
      longoEnviado = false;
    } else if (!longoEnviado) {
      return mudou - inicio >= 1000 ? 3 : 0;
    }
  }
  if (estavel && pressionado && !longoEnviado && agora - inicio >= 1000) {
    longoEnviado = true;
    return 3;
  }
  return -1;
}


constexpr uint8_t PWM_PIN = 32;
constexpr uint16_t PWM_FREQ = 500;
constexpr uint8_t PWM_BITS = 8;
constexpr uint8_t PWM_MAX_TESTE = 100;
static_assert(BOTAO_SELECIONAR != TFT_LED && BOTAO_SELECIONAR != PWM_PIN &&
              BOTAO_SELECIONAR != MUX_RESET && BOTAO_SELECIONAR != I2C_SDA &&
              BOTAO_SELECIONAR != I2C_SCL,
              "Conflito de GPIO no botao SELECIONAR!");

// Altere SOMENTE estes canais se a nova PCB foi roteada de outra forma.
constexpr uint8_t NUM_SENSORES = 5;
const uint8_t canaisSensor[NUM_SENSORES] = {2, 3, 5, 6, 7};

// Para PC817 configurado como INVERSOR (conforme codigo anterior).
constexpr uint8_t RESET_ATIVAR = HIGH;
constexpr uint8_t RESET_LIBERAR = LOW;

Adafruit_ILI9341 tft(&SPI, TFT_DC, TFT_CS, TFT_RST);
// Um objeto por sensor AS7341: cada um preserva seu buffer de leitura.
Adafruit_AS7341 sensor7341[NUM_SENSORES];
// Biblioteca SparkFun AS7343 usa o barramento apontado para o canal selecionado.
SfeAS7343ArdI2C sensor7343;

enum class TipoSensor : uint8_t {
  SEM_ACK,
  DESCONHECIDO,
  AS7341,
  AS7343
};

const char *nomeTipo(TipoSensor tipo);
TipoSensor identificarSensor(uint8_t indice);
TipoSensor tipoSensor[NUM_SENSORES];
bool sensorInicializado[NUM_SENSORES] = {false};
bool leituraOK[NUM_SENSORES] = {false};
uint8_t id7341Raw[NUM_SENSORES] = {0};
uint8_t id7343Raw[NUM_SENSORES] = {0};
uint16_t leituraA[NUM_SENSORES] = {0};
uint16_t leituraB[NUM_SENSORES] = {0};
uint8_t muxEndereco = 0;                // 0 = nao encontrado
const char *nomeTipo(TipoSensor tipo) {
  switch (tipo) {
    case TipoSensor::AS7341: return "AS7341";
    case TipoSensor::AS7343: return "AS7343";
    case TipoSensor::DESCONHECIDO: return "ID?";
    default: return "AUSENTE";
  }
}

void limparEstadoSensores() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    tipoSensor[i] = TipoSensor::SEM_ACK;
    sensorInicializado[i] = false;
    leituraOK[i] = false;
    id7341Raw[i] = 0;
    id7343Raw[i] = 0;
    leituraA[i] = 0;
    leituraB[i] = 0;
  }
}

bool enderecoResponde(uint8_t endereco) {
  Wire.beginTransmission(endereco);
  return Wire.endTransmission() == 0;
}

bool lerRegistrador(uint8_t endereco, uint8_t registrador, uint8_t &valor) {
  Wire.beginTransmission(endereco);
  Wire.write(registrador);
  if (Wire.endTransmission(false) != 0) return false; // repeated START
  if (Wire.requestFrom(endereco, (uint8_t)1) != 1) return false;
  valor = Wire.read();
  return true;
}

void executarReset(bool resetInicial) {

  Serial.println("\n[MUX] RESET GPIO19 HIGH -> LOW (pressupoe PC817 inversor)");

  pinMode(MUX_RESET, OUTPUT);
  digitalWrite(MUX_RESET, RESET_ATIVAR);
  delay(resetInicial ? 5000 : 150);
  digitalWrite(MUX_RESET, RESET_LIBERAR);
  delay(50);

  Wire.end();
  delay(10);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // 100 kHz: teste conservador com isolador I2C
  Wire.setTimeOut(50);

  muxEndereco = 0;
  limparEstadoSensores();
}

bool procurarMUX() {
  muxEndereco = 0;
  for (uint8_t endereco = 0x70; endereco <= 0x77; endereco++) {
    if (enderecoResponde(endereco)) {
      muxEndereco = endereco;
      break;
    }
  }
  if (muxEndereco)
    Serial.printf("[MUX] DETECTADO no endereco 0x%02X\n", muxEndereco);
  else
    Serial.println("[MUX] NAO DETECTADO em 0x70..0x77");
  return muxEndereco != 0;
}

bool selecionarCanal(uint8_t canal) {
  if (!muxEndereco || canal > 7) return false;
  Wire.beginTransmission(muxEndereco);
  Wire.write(uint8_t(1U << canal));
  return Wire.endTransmission() == 0;
}

void desligarCanais() {
  if (!muxEndereco) return;
  Wire.beginTransmission(muxEndereco);
  Wire.write(uint8_t(0));
  Wire.endTransmission();
}

// ---------------- DETECCAO AS7341 / AS7343 ----------------
// AS7341: registro WHOAMI 0x92, ID nos bits 7:2, esperado 0x09.
// Checagem: (registro & 0xFC) == (0x09 << 2) == 0x24.
// AS7343: registro ID 0x5A no banco 1, ID esperado 0x81.
// Para o AS7343, a biblioteca SparkFun gerencia a selecao do banco.
TipoSensor identificarSensor(uint8_t indice) {
  id7341Raw[indice] = 0;
  id7343Raw[indice] = 0;
  if (!selecionarCanal(canaisSensor[indice])) return TipoSensor::SEM_ACK;
  if (!enderecoResponde(SENSOR_ADDR)) return TipoSensor::SEM_ACK;

  uint8_t id41 = 0;
  if (lerRegistrador(SENSOR_ADDR, 0x92, id41)) {
    id7341Raw[indice] = id41;
    if ((id41 & 0xFC) == 0x24) return TipoSensor::AS7341;
  }

  // Nao identificado como AS7341: verifica o ID real do AS7343.
  // getDeviceID() le 0x5A usando o banco correto do proprio AS7343.
  if (sensor7343.begin()) {
    uint8_t id43 = sensor7343.getDeviceID();
    id7343Raw[indice] = id43;
    if (id43 == 0x81) return TipoSensor::AS7343;
  }
  return TipoSensor::DESCONHECIDO;
}

bool gravarRegistro(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(reg); Wire.write(valor);
  if (Wire.endTransmission()!=0) return false;
  uint8_t lido=0;
  return lerRegistrador(SENSOR_ADDR,reg,lido) && lido==valor;
}

void iniciarSensores() {
  if (!muxEndereco) return;
  Serial.println("\n[SENSORES] Identificacao de chips nos canais do MUX:");
  limparEstadoSensores();

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    uint8_t canal = canaisSensor[i];
    tipoSensor[i] = identificarSensor(i);

    if (tipoSensor[i] == TipoSensor::SEM_ACK) {
      Serial.printf("[S%u CH%u] AUSENTE: sem ACK em 0x39\n", i + 1, canal);
      continue;
    }
    if (tipoSensor[i] == TipoSensor::DESCONHECIDO) {
      Serial.printf("[S%u CH%u] ID DESCONHECIDO (0x92=0x%02X, AS7343_ID=0x%02X)\n",
                    i + 1, canal, id7341Raw[i], id7343Raw[i]);
      continue;
    }

    const TipoSensor esperado[] = {TipoSensor::AS7343, TipoSensor::AS7341,
                                   TipoSensor::AS7343, TipoSensor::AS7341,
                                   TipoSensor::AS7343};
    if (tipoSensor[i] != esperado[i]) {
      Serial.printf("[S%u CH%u] Modelo diverge do Quadro 13. Confira a etiqueta.\n", i+1, canal);
      continue;
    }

    // O MUX ainda deve estar no canal selecionado na etapa de identificacao.
    bool ok = false;
    if (tipoSensor[i] == TipoSensor::AS7341) {
      ok = sensor7341[i].begin(SENSOR_ADDR, &Wire);
      if (ok) ok = sensor7341[i].setATIME(29);
      if (ok) ok = sensor7341[i].setASTEP(599);
      if (ok) ok = sensor7341[i].setGain(AS7341_GAIN_32X);
    } else if (tipoSensor[i] == TipoSensor::AS7343) {
      ok = sensor7343.begin();
      if (ok) ok = sensor7343.powerOn();
      if (ok) ok = sensor7343.setAutoSmux(AUTOSMUX_18_CHANNELS);
      if (ok) ok = sensor7343.setAgain(AGAIN_32);
      if (ok) ok = gravarRegistro(0x81,29);
      if (ok) ok = gravarRegistro(0xD4,0x57); // 599 = 0x0257
      if (ok) ok = gravarRegistro(0xD5,0x02);
      if (ok) ok = sensor7343.enableSpectralMeasurement();
    }

    sensorInicializado[i] = ok;
    Serial.printf("[S%u CH%u] %-6s ID(41)=0x%02X ID(43)=0x%02X -> %s\n",
                  i + 1, canal, nomeTipo(tipoSensor[i]), id7341Raw[i],
                  id7343Raw[i], ok ? "INICIALIZADO" : "ERRO DE CONFIGURACAO");
  }
  desligarCanais();
  delay(150);
}

// ---------------- LEITURA BRUTA ADEQUADA A CADA CHIP ----------------
bool paradaSolicitada();  // prototipo para interrupcao durante leitura AS7341

bool lerUmAS7341(uint8_t indice) {
  // Leitura em duas etapas do SMUX, com limite de tempo em vez de
  // readAllChannels() bloqueante (que esperaria para sempre em falha).
  Adafruit_AS7341 &sensor = sensor7341[indice];
  if (!sensor.startReading()) return false;
  uint32_t inicio = millis();
  bool concluido = false;
  while (millis() - inicio < 1200) {
    // Interrompe a espera quando o operador solicita parada.
    if (paradaSolicitada()) return false;
    if (!enderecoResponde(SENSOR_ADDR)) return false;
    if (sensor.checkReadingProgress()) {
      concluido = true;
      break;
    }
    delay(5);
  }
  if (!concluido) return false;
  leituraA[indice] = sensor.getChannel(AS7341_CHANNEL_415nm_F1);
  leituraB[indice] = sensor.getChannel(AS7341_CHANNEL_445nm_F2);
  return true;
}

bool lerUmAS7343(uint8_t indice) {
  if (!sensor7343.readSpectraDataFromSensor()) return false;
  leituraA[indice] = sensor7343.getChannelData(CH_DARK_BLUE_F2_425NM);
  leituraB[indice] = sensor7343.getBlue();
  return true;
}



// ================================================================
// TESTE AUTOMATICO: 5 SENSORES X 7 PWM X 10 RODADAS
// Leitura dos cinco sensores sequencialmente via TCA9548A em cada
// rodada. Intervalo entre INICIOS das rodadas: 1000 ms (se possivel).
// PWM de 0,10,30,50,60,75,68%, nessa ordem.
// Cada novo PWM estabiliza por 2000 ms antes da primeira rodada.
// Impressao CSV em tempo real + gravacao por bloco na NVS.
// Dados antigos da calibracao nao sao alterados (namespace distinto).
// ================================================================

Preferences memoria;
constexpr uint8_t TOTAL_PWM = 7;
constexpr uint8_t TOTAL_AMOSTRAS = 10;
const uint8_t LISTA_PWM[TOTAL_PWM] = {0, 10, 30, 50, 60, 75, 68};
const char *NOME_BLOCO[TOTAL_PWM] = {"Fundo", "A1", "A2", "A3", "V1", "V2", "V3"};
// Referencia do THOR obtida no S1 da campanha anterior. Somente metadado;
// NAO corresponde a uma medicao simultanea do THOR nesta coleta.
const uint16_t THOR_S1_DECIMOS[TOTAL_PWM] = {0, 17, 46, 74, 83, 103, 92};

constexpr uint32_t ESPERA_ESTABILIZACAO_MS = 2000;
constexpr uint32_t INTERVALO_AMOSTRAGEM_MS = 1000;
constexpr uint16_t LIMIAR_RAW_SAT_AS7343 = 17990;
constexpr uint16_t LIMIAR_RAW_SAT_AS7341 = 65500;
constexpr bool EXIGIR_CINCO_SENSORES = true;

// Status de cada medicao: 0=FALHA, 1=OK, 2=SATURADO, 3=AUSENTE.
enum StatusLeitura : uint8_t {
  FALHA_LEITURA=0, LEITURA_OK=1, LEITURA_SATURADA=2, SENSOR_AUSENTE=3
};

// Uma entrada por nivel PWM, com 10 amostras independentes para cada sensor.
// Guardada em Preferences (NVS), sem interferir em "thorQ13v1".
struct BlocoLeituras {
  uint32_t assinatura;
  uint8_t versao;
  uint8_t pwm;
  uint16_t thorReferenciaS1;
  uint8_t tipo[NUM_SENSORES];
  uint8_t status[NUM_SENSORES][TOTAL_AMOSTRAS];
  uint16_t principal[NUM_SENSORES][TOTAL_AMOSTRAS];
  uint16_t auxiliar[NUM_SENSORES][TOTAL_AMOSTRAS];
  uint32_t tempoMs[TOTAL_AMOSTRAS];  // inicio de cada rodada
  uint32_t intervaloMs[TOTAL_AMOSTRAS];
  uint32_t tempoSensorMs[NUM_SENSORES][TOTAL_AMOSTRAS]; // fim da leitura de cada sensor
};

// Prototipos definidos aqui para evitar erro na geracao automatica
// de prototipos da Arduino IDE (tipo usado antes da declaracao).
void ajustarPWM(uint8_t percentual);
void telaPronto(const char *mensagem);
void telaProgresso(uint8_t p, uint8_t rodada, const BlocoLeituras &r);
bool paradaSolicitada();
bool esperarComCancelamento(uint32_t duracaoMs);
uint8_t contarSensores();
bool lerSensor(uint8_t indice);
bool gravarBloco(uint8_t p, const BlocoLeituras &r);
bool recuperarBloco(uint8_t p, BlocoLeituras &r);
void imprimirLinhaCSV(uint8_t p, uint8_t rodada, uint8_t sensor, const BlocoLeituras &r);
void exportarMemoria();
void executarColeta();

constexpr uint32_t ASSINATURA = 0x53505735UL; // "SPW5"
bool pwmAtivo = false;
bool memoriaOK = false;
bool emColeta = false;
bool solicitouParar = false;
uint8_t pwmAtual = 0;

void ajustarPWM(uint8_t percentual) {
  pwmAtual = percentual > 100 ? 100 : percentual;
  if (!pwmAtivo) return;
  // Core ESP32 Arduino 3.x, como no codigo de calibracao original.
  ledcWrite(PWM_PIN, (uint32_t(pwmAtual) * 255U + 50U) / 100U);
}

uint8_t contarSensores() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NUM_SENSORES; ++i) {
    if (sensorInicializado[i]) ++n;
  }
  return n;
}

void telaPronto(const char *mensagem) {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(6, 8);
  tft.println("LEITURA 5 SENSORES");
  tft.setCursor(6, 40);
  tft.print("Ativos: "); tft.print(contarSensores()); tft.println(" / 5");
  tft.setCursor(6, 68);
  tft.println("PWM 0/10/30/50");
  tft.println("60/75/68 %");
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(6, 128);
  tft.println(mensagem);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(6, 182);
  tft.println("SELECT ou Serial R");
  tft.println("Serial E: exportar");
}

void telaProgresso(uint8_t p, uint8_t rodada, const BlocoLeituras &r) {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(6, 5);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.print(NOME_BLOCO[p]); tft.print("  PWM "); tft.print(LISTA_PWM[p]); tft.println("%");
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Rodada "); tft.print(rodada+1); tft.println("/10");
  tft.setTextSize(2);
  for (uint8_t s=0; s<NUM_SENSORES; ++s) {
    tft.setTextColor(r.status[s][rodada] == LEITURA_OK ? ILI9341_GREEN : ILI9341_YELLOW);
    tft.print("S"); tft.print(s+1); tft.print(" ");
    if (r.status[s][rodada] == SENSOR_AUSENTE) {
      tft.println("AUSENTE");
    } else if (r.status[s][rodada] == FALHA_LEITURA) {
      tft.println("FALHA");
    } else {
      tft.print(r.principal[s][rodada]);
      if (r.status[s][rodada] == LEITURA_SATURADA) tft.print(" SAT");
      tft.println();
    }
  }
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Intervalo medido: "); tft.print(r.intervaloMs[rodada]); tft.println(" ms");
  tft.print("SELECT 1s ou Serial S: parar");
}

// Lê comandos mesmo durante as pausas e o processo de amostragem.
bool paradaSolicitada() {
  if (solicitouParar) return true;
  if (eventoSelecionar() == 3) solicitouParar = true;
  while (Serial.available()) {
    char c = toupper((unsigned char)Serial.read());
    if (c == 'S') solicitouParar = true;
  }
  return solicitouParar;
}

bool esperarComCancelamento(uint32_t duracaoMs) {
  uint32_t inicio = millis();
  while ((uint32_t)(millis()-inicio) < duracaoMs) {
    if (paradaSolicitada()) return false;
    delay(5);
  }
  return !paradaSolicitada();
}

bool lerSensor(uint8_t indice) {
  leituraA[indice] = 0;
  leituraB[indice] = 0;
  if (!sensorInicializado[indice]) return false;
  if (!selecionarCanal(canaisSensor[indice])) return false;
  if (!enderecoResponde(SENSOR_ADDR)) return false;

  bool ok = false;
  if (tipoSensor[indice] == TipoSensor::AS7341) {
    ok = lerUmAS7341(indice);
  } else if (tipoSensor[indice] == TipoSensor::AS7343) {
    ok = lerUmAS7343(indice);
  }
  desligarCanais();
  return ok;
}

String chaveBloco(uint8_t p) { return String("p") + String(p); }

bool gravarBloco(uint8_t p, const BlocoLeituras &r) {
  if (!memoriaOK) return false;
  String chave = chaveBloco(p);
  if (memoria.putBytes(chave.c_str(), &r, sizeof(r)) != sizeof(r)) return false;
  BlocoLeituras conferido = {};
  if (memoria.getBytesLength(chave.c_str()) != sizeof(r)) return false;
  if (memoria.getBytes(chave.c_str(), &conferido, sizeof(conferido)) != sizeof(r)) return false;
  return memcmp(&conferido, &r, sizeof(r)) == 0;
}

bool recuperarBloco(uint8_t p, BlocoLeituras &r) {
  if (!memoriaOK || p >= TOTAL_PWM) return false;
  String chave = chaveBloco(p);
  if (memoria.getBytesLength(chave.c_str()) != sizeof(r)) return false;
  if (memoria.getBytes(chave.c_str(), &r, sizeof(r)) != sizeof(r)) return false;
  return r.assinatura == ASSINATURA && r.versao == 1 && r.pwm == LISTA_PWM[p];
}

const char *textoStatus(uint8_t status) {
  switch (status) {
    case LEITURA_OK:       return "OK";
    case LEITURA_SATURADA: return "SATURADO";
    case SENSOR_AUSENTE:   return "AUSENTE";
    default:               return "FALHA";
  }
}

void imprimirLinhaCSV(uint8_t p, uint8_t rodada, uint8_t s, const BlocoLeituras &r) {
  // raw zerado significa dado AUSENTE/FALHA quando status != OK/SATURADO.
  Serial.printf("S%u;%u;%s;%s;%u;%.1f;%u;%lu;%lu;%lu;%u;%u;%s\n",
                s+1, canaisSensor[s], nomeTipo(static_cast<TipoSensor>(r.tipo[s])),
                NOME_BLOCO[p], LISTA_PWM[p], r.thorReferenciaS1/10.0f,
                rodada+1, (unsigned long)r.tempoMs[rodada],
                (unsigned long)r.intervaloMs[rodada],
                (unsigned long)r.tempoSensorMs[s][rodada],
                r.principal[s][rodada], r.auxiliar[s][rodada],
                textoStatus(r.status[s][rodada]));
}

void exportarMemoria() {
  if (!memoriaOK) { Serial.println("[ERRO] NVS indisponivel"); return; }
  Serial.println("\nCSV_INICIO");
  Serial.println("sensor;ch;modelo;bloco;pwm;thor_ref_S1;n;inicio_rodada_ms;intervalo_ms;fim_leitura_sensor_ms;raw_principal;raw_aux;status");
  BlocoLeituras r;
  uint8_t blocosEncontrados = 0;
  for (uint8_t p=0; p<TOTAL_PWM; ++p) {
    if (!recuperarBloco(p, r)) continue;
    ++blocosEncontrados;
    for (uint8_t n=0; n<TOTAL_AMOSTRAS; ++n) {
      for (uint8_t s=0; s<NUM_SENSORES; ++s) imprimirLinhaCSV(p, n, s, r);
    }
  }
  Serial.println("CSV_FIM");
  Serial.printf("[NVS] %u/%u blocos exportados\n", blocosEncontrados, TOTAL_PWM);
}

void executarColeta() {
  if (emColeta) return;
  if (!pwmAtivo) {
    Serial.println("[ERRO] PWM indisponivel");
    telaPronto("ERRO PWM");
    return;
  }
  uint8_t ativos = contarSensores();
  if (EXIGIR_CINCO_SENSORES && ativos != NUM_SENSORES) {
    Serial.printf("[ERRO] Sao necessarios 5 sensores. Detectados %u/5.\n", ativos);
    telaPronto("FALTAM SENSORES");
    return;
  }
  if (ativos == 0) return;

  emColeta = true;
  solicitouParar = false;
  Serial.println("\n[INICIO] 5 sensores / 7 PWM / 10 rodadas. Ordem: 0,10,30,50,60,75,68%.");
  Serial.println("[INFO] Cinco sensores lidos em SEQUENCIA no mesmo ciclo do multiplexador I2C.");
  Serial.println("[INFO] THOR referencia S1 anterior; NAO medido durante esta rodada.");
  Serial.println("[INFO] Cada bloco completo substitui somente o mesmo bloco neste namespace NVS.");
  Serial.println("CSV_TEMPO_REAL_INICIO");
  Serial.println("sensor;ch;modelo;bloco;pwm;thor_ref_S1;n;inicio_rodada_ms;intervalo_ms;fim_leitura_sensor_ms;raw_principal;raw_aux;status");
  const uint32_t instanteInicio = millis();
  bool interrompido = false;

  for (uint8_t p=0; p<TOTAL_PWM && !interrompido; ++p) {
    BlocoLeituras bloco = {};
    bloco.assinatura = ASSINATURA;
    bloco.versao = 1;
    bloco.pwm = LISTA_PWM[p];
    bloco.thorReferenciaS1 = THOR_S1_DECIMOS[p];
    for (uint8_t s=0; s<NUM_SENSORES; ++s) bloco.tipo[s] = (uint8_t)tipoSensor[s];

    ajustarPWM(LISTA_PWM[p]);
    Serial.printf("[PWM] Bloco %s = %u%%; estabilizando %lu ms...\n",
                  NOME_BLOCO[p], LISTA_PWM[p],
                  (unsigned long)ESPERA_ESTABILIZACAO_MS);
    telaPronto("ESTABILIZANDO...");
    if (!esperarComCancelamento(ESPERA_ESTABILIZACAO_MS)) {
      interrompido = true;
      break;
    }

    uint32_t inicioAnterior = 0;
    for (uint8_t n=0; n<TOTAL_AMOSTRAS; ++n) {
      if (paradaSolicitada()) { interrompido = true; break; }
      if (n>0) {
        uint32_t tempoDecorrido = millis()-inicioAnterior;
        if (tempoDecorrido < INTERVALO_AMOSTRAGEM_MS) {
          if (!esperarComCancelamento(INTERVALO_AMOSTRAGEM_MS-tempoDecorrido)) {
            interrompido = true;
            break;
          }
        }
      }
      uint32_t inicioRodada = millis();
      bloco.tempoMs[n] = inicioRodada - instanteInicio;
      bloco.intervaloMs[n] = n==0 ? 0 : inicioRodada-inicioAnterior;
      inicioAnterior = inicioRodada;
      if (n>0 && bloco.intervaloMs[n]>INTERVALO_AMOSTRAGEM_MS+50) {
        Serial.printf("[AVISO] Rodada %u %s: intervalo real %lu ms (> 1000 ms).\n",
                      n+1,NOME_BLOCO[p],(unsigned long)bloco.intervaloMs[n]);
      }

      // O TCA9548A permite um unico canal ativo por vez; os sensores
      // nao sao amostrados exatamente no mesmo instante.
      for (uint8_t s=0; s<NUM_SENSORES; ++s) {
        if (paradaSolicitada()) { interrompido=true; break; }
        if (!sensorInicializado[s]) {
          bloco.status[s][n] = SENSOR_AUSENTE;
          continue;
        }
        const bool ok = lerSensor(s);
        bloco.tempoSensorMs[s][n] = millis() - instanteInicio;
        if (paradaSolicitada()) { interrompido=true; break; }
        if (!ok) {
          bloco.status[s][n] = FALHA_LEITURA;
        } else {
          // Mantemos a convencao do sketch original: principal = leituraB.
          bloco.principal[s][n] = leituraB[s];
          bloco.auxiliar[s][n] = leituraA[s];
          const uint16_t limiar = tipoSensor[s] == TipoSensor::AS7343 ?
                                      LIMIAR_RAW_SAT_AS7343 : LIMIAR_RAW_SAT_AS7341;
          bloco.status[s][n] = (leituraB[s] >= limiar || leituraA[s] >= limiar)
                               ? LEITURA_SATURADA : LEITURA_OK;
        }
      }
      desligarCanais();
      if (interrompido) break;
      for (uint8_t s=0; s<NUM_SENSORES; ++s) imprimirLinhaCSV(p, n, s, bloco);
      telaProgresso(p,n,bloco);
    }

    // So grava blocos completos; interromper nunca substitui bloco salvo.
    if (!interrompido) {
      if (gravarBloco(p,bloco))
        Serial.printf("[NVS OK] %s (PWM %u%%): 50 registros gravados\n", NOME_BLOCO[p], LISTA_PWM[p]);
      else
        Serial.printf("[NVS FALHA] %s: confira a saida CSV em tempo real.\n", NOME_BLOCO[p]);
    }
  }
  ajustarPWM(0); // Nunca deixar LEDs acesos ao encerrar ou cancelar.
  desligarCanais();
  emColeta=false;
  Serial.println("CSV_TEMPO_REAL_FIM");
  if (interrompido) {
    Serial.println("[CANCELADO] Dados de blocos completos permanecem na NVS.");
    telaPronto("COLETA CANCELADA");
  } else {
    Serial.println("[CONCLUIDO] Foram executadas 10 rodadas para cada PWM.");
    Serial.println("[SERIAL] E para exportar os resultados gravados; R para refazer.");
    telaPronto("COLETA CONCLUIDA");
  }
  solicitouParar = false;
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  pwmAtivo = ledcAttach(PWM_PIN, PWM_FREQ, PWM_BITS);
  ajustarPWM(0);
  pinMode(BOTAO_SELECIONAR, INPUT_PULLUP);
  pinMode(BOTAO_VOLTAR, INPUT_PULLUP);
  pinMode(BOTAO_AVANCAR, INPUT_PULLUP);

  memoriaOK = memoria.begin("leitura5pwm", false);
  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin(4000000);
  tft.setRotation(1);
  executarReset(true);
  if (procurarMUX()) iniciarSensores();
  Serial.printf("[BOOT] %u de 5 sensores prontos. NVS=%s PWM=%s\n",
                contarSensores(),memoriaOK?"OK":"ERRO",pwmAtivo?"OK":"ERRO");
  Serial.println("[CONTROLES] SELECT (toque) ou R = iniciar; SELECT 1s ou S = parar; E = exportar NVS.");
  Serial.println("[NOTA] Dados de calibracao anteriores permanecem em outra area da NVS.");
  telaPronto("PRONTO");
}

void loop() {
  int evento = eventoSelecionar();
  if (evento == 0) {
    executarColeta();
    return;
  }
  if (Serial.available()) {
    char comando = toupper((unsigned char)Serial.read());
    if (comando == 'R') executarColeta();
    else if (comando == 'E') exportarMemoria();
    else if (comando == '?') {
      Serial.println("R: executar | E: exportar | S: parar durante coleta | SELECT: executar");
    }
  }
  delay(10);
}
