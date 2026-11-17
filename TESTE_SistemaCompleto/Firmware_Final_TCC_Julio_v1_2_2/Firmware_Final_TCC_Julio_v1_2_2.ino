/*
  TCC - Monitoramento e Controle de Irradiancia Espectral - v1.2.2 experimental - zeros validos e botoes com interrupcao
  ESP32-WROOM-32, TFT ILI9341 320x240, TCA9548A, 3xAS7343, 2xAS7341.
  Projeto academico de bancada. NAO e equipamento para uso clinico.

  Calibracao: Analise_calibracao_THOR_17-11-2026.xlsx, aba CAL_30:
  modelo E_i = a_i * max(RAWprincipal_i - Fundo_i, 0), coeficiente b=0.
  Ajustes de calibracao: PWM 10, 30, 50, 60, 75%; 68% para verificacao.
  Novo limite de atuacao: 100%. Entre 75 e 100%, as estimativas
  extrapolam os dados de calibracao: sao apenas indicativas; confirmar THOR.
  Controle busca 8,0 a 9,0, com alvo interno de 8,5 uW/cm2/nm.
  A banda 8,0 a 10,0 continua como faixa geral informativa.
  Estes parametros SOMENTE sao defensaveis com geometria fixa de 30 cm,
  mesma fonte/montagem, tampa, ganho, integracao, canais e orientacao.
  Para a montagem a 40 cm: REFAZER validacao/calibracao; NAO transferir.

  Os LEDs ficam em PWM=0 na inicializacao, durante OFF e em falha grave.
  A malha de controle NUNCA atua sem confirmacao explicita no TFT.
  Alertas sao indicadores VISUAIS OPERACIONAIS, nao alarmes IEC certificados.

  Bibliotecas: Adafruit GFX, Adafruit ILI9341, Adafruit AS7341,
               SparkFun AS7343, Arduino-ESP32 3.x.
*/
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_AS7341.h>
#include <SparkFun_AS7343.h>

// ==== PINAGEM FINAL, ver Quadro 5 da monografia e sketch ensaiado ====
constexpr uint8_t PIN_SDA = 17, PIN_SCL = 16, PIN_MUX_RST = 19;
constexpr uint8_t PIN_PWM = 32;
constexpr uint8_t PIN_TFT_CS = 26, PIN_TFT_DC = 33, PIN_TFT_RST = 13;
constexpr uint8_t PIN_TFT_MOSI = 14, PIN_TFT_SCK = 25;
constexpr uint8_t PIN_TFT_MISO = 36, PIN_TFT_LED = 27;
constexpr uint8_t PIN_SELECT = 18, PIN_BACK = 21, PIN_NEXT = 22;
constexpr uint8_t PIN_POWER = 2; // ATENCAO: GPIO2 e pino de boot/strap do ESP32
constexpr uint8_t I2C_SENSOR = 0x39;
constexpr uint8_t N = 5;
constexpr uint8_t CH[N] = {2, 3, 5, 6, 7};
constexpr uint16_t PWM_FREQ = 500;
constexpr uint8_t PWM_BITS = 8;

// ==== PARAMETROS DE OPERAÇÃO: ENSAIO DE BANCADA (NAO PRESCRICAO CLINICA) ====
constexpr float E_MIN = 8.0f, E_MAX = 10.0f; // faixa geral informativa
constexpr float E_OBJ_INF = 8.0f, E_OBJ_SUP = 9.0f;
constexpr float E_ALVO = 8.5f; // ponto medio da nova faixa objetivo
constexpr float PWM_MAX_CALIBRADO = 75.0f; // limite dos ENSAIOS, nao do atuador
constexpr float KP = 1.5f;                 // %PWM / (uW/cm2/nm), inicial, validar
constexpr float PASSO_MAX = 5.0f;          // maximo 5 pontos percentuais por ciclo
constexpr float PWM_MAX = 100.0f; // autoriza investigar 75-100% NA BANCADA
constexpr float PWM_MIN_CONTROLE = 10.0f; // nao extrapolar abaixo do menor patamar calibrado
constexpr float PWM_INICIO = 50.0f;
constexpr uint8_t SENSORES_MIN = 4;
constexpr uint8_t CICLOS_OK = 3;
constexpr uint8_t MAX_PASSOS = 25;
constexpr uint32_t PERIODO_MEDIDA_MS = 1300;
constexpr uint32_t ESTABILIZACAO_PWM_MS = 1800;
constexpr uint32_t ATUALIZACAO_TELA_MS = 700;
constexpr uint32_t LIMITE_AS7341_MS = 1300;
constexpr uint32_t RETENTATIVA_SENSOR_MS = 5000;
constexpr uint8_t FALHAS_PARA_REINICIAR = 3;
constexpr bool DESLIGAR_EM_FALHA_DE_MEDICAO = true; // escolha de protecao de bancada

// ==== CALIBRACAO INDIVIDUAL NO PLANO / 30 CM (planilha 17/11/2026) ====
// S1/S3/S5: AS7343 canal principal = getBlue(); auxiliar = F2 425nm
// S2/S4: AS7341 canal principal = 445nm F2; auxiliar = 415nm F1
// Os valores NAO correspondem aos coeficientes da aba CENTRO.
struct Calib {
  float a;
  float fundo;
  float maxRAW;
};
const Calib CAL[N] = {
  {0.007380975596f,  70.025f, 17990.0f}, // S1 AS7343, canal MUX 2
  {0.002344222903f, 214.000f, 65500.0f}, // S2 AS7341, canal MUX 3
  {0.001207907889f,  63.350f, 17990.0f}, // S3 AS7343, canal MUX 5
  {0.005849934637f, 199.750f, 65500.0f}, // S4 AS7341, canal MUX 6
  {0.016314020884f,  80.875f, 17990.0f}  // S5 AS7343, canal MUX 7
};

// Perfil esperado de cada posicao no plano de 30 cm (THOR local, aba CAL_30)
// PWM = 10,30,50,60,75; usado APENAS para heuristica de obstrucao.
const float PERF_PWM[5] = {10, 30, 50, 60, 75};
const float PERF_E[N][5] = {
  {0.9, 1.2, 1.6, 1.9, 2.1},
  {0.9, 1.2, 1.5, 1.8, 1.9},
  {2.5, 6.7,11.2,13.5,16.8},
  {0.0, 1.0, 1.5, 1.8, 2.1},
  {0.0, 1.2, 1.6, 1.8, 2.1}
};
constexpr float LIMIAR_OBSTRUCAO_REL = 0.35f;
constexpr float LIMIAR_RECUPERACAO_REL = 0.65f;
constexpr uint8_t CICLOS_OBSTRUCAO = 3;
constexpr uint8_t CICLOS_RECUPERACAO = 3;

// ==== CORES (RGB565) ==== 
constexpr uint16_t FUNDO = 0x0862;   // azul escuro / grafite
constexpr uint16_t CARTAO = 0x18E5;
constexpr uint16_t CARTAO2 = 0x2146;
constexpr uint16_t BRANCO = 0xFFFF;
constexpr uint16_t CINZA = 0xAD55;
constexpr uint16_t CIANO = 0x4D9F;
constexpr uint16_t VERDE = 0x47E9;
constexpr uint16_t AMARELO = 0xFE20;
constexpr uint16_t VERMELHO = 0xF800; // RESERVADO, nao usado em alerta rotineiro
constexpr uint16_t AZUL = 0x4D1F;

Adafruit_ILI9341 tft(&SPI, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST);
Adafruit_AS7341 as7341[N];
SfeAS7343ArdI2C as7343; // objeto compartilhado; MUX permite um canal por vez

enum class Modelo : uint8_t { AUSENTE, AS7341, AS7343, DESCONHECIDO };
enum class Estado : uint8_t { OFF, INICIANDO, MONITORANDO, FORA_FAIXA,
                             CONFIRMAR, CONTROLANDO, CONCLUIDO, LIMITE_PWM,
                             SATURACAO, FALHA_SENSORES, SEM_CALIBRACAO };
enum class Motivo : uint8_t { OK, AUSENTE, FALHA, SATURADO, OBSTRUIDO };
struct Ponto {
  Modelo tipo = Modelo::AUSENTE;
  bool inicializado = false;
  bool leituraFisica = false;
  Motivo motivo = Motivo::AUSENTE;
  uint16_t rawPrincipal = 0, rawAux = 0;
  float E = NAN;
  uint8_t contadorObstrucao = 0, contadorRecuperacao = 0;
  bool obstruido = false;
  uint8_t falhasConsecutivas = 0;
};
// Os sensores AS734x podem ocupar o loop por >100ms. Interrupcoes
// registram o pressionamento COMPLETO (pressionar/soltar), mesmo durante I2C.
// Os eventos sao processados DEPOIS da aquisicao, fora da interrupcao.
struct BotaoIRQ {
  volatile bool pressionado = false;
  volatile uint32_t iniciouUs = 0;
  volatile uint32_t ultimaSolturaUs = 0;
  volatile uint8_t curtos = 0;
  volatile uint8_t longos = 0;
};
constexpr uint8_t PIN_BOTOES[4] = {PIN_SELECT, PIN_BACK, PIN_NEXT, PIN_POWER};
BotaoIRQ botoesIRQ[4];

void IRAM_ATTR registrarBorda(uint8_t i) {
  const uint32_t agora = micros();
  const bool pressionou = (digitalRead(PIN_BOTOES[i]) == LOW);
  BotaoIRQ &b = botoesIRQ[i];
  if (pressionou) {
    if (!b.pressionado && uint32_t(agora - b.ultimaSolturaUs) >= 50000UL) {
      b.pressionado = true;
      b.iniciouUs = agora;
    }
  } else if (b.pressionado) {
    const uint32_t duracao = uint32_t(agora - b.iniciouUs);
    // Ignora os primeiros bounces de soltura, mas mantem o inicio.
    if (duracao >= 35000UL) {
      b.pressionado = false;
      b.ultimaSolturaUs = agora;
      if (duracao >= 900000UL) {
        if (b.longos < 255) ++b.longos;
      } else {
        if (b.curtos < 255) ++b.curtos;
      }
    }
  }
}
void IRAM_ATTR irqSelecionar() { registrarBorda(0); }
void IRAM_ATTR irqVoltar()    { registrarBorda(1); }
void IRAM_ATTR irqAvancar()  { registrarBorda(2); }
void IRAM_ATTR irqPower()    { registrarBorda(3); }

Ponto pontos[N];
uint8_t muxAddr = 0;
Estado estado = Estado::OFF;
uint8_t pagina = 0;
float pwm = 0.0f, eMedia = NAN;
uint8_t nValidos = 0, passosControle = 0, consecutivosOK = 0;
uint8_t ciclosFora = 0;
uint32_t ultimaLeitura = 0, ultimaTela = 0, alterouPWM = 0;
uint32_t avisoConclusao = 0;
bool pwmPronto = false, telaSuja = true;
uint8_t totalSaturados();
uint32_t ultimaTentativaSensor = 0;
// Memoriza a ultima causa de encerramento por limite, para a tela.
bool limitePorTentativas = false;

bool objetivoAtingido() {
  return isfinite(eMedia) && nValidos >= SENSORES_MIN &&
         totalSaturados() == 0 &&
         eMedia >= E_OBJ_INF && eMedia <= E_OBJ_SUP;
}
bool extrapolando() { return pwm > PWM_MAX_CALIBRADO + 0.1f; }
uint8_t totalSaturados() {
  uint8_t n = 0;
  for (uint8_t i=0; i<N; ++i)
    if (pontos[i].motivo == Motivo::SATURADO) ++n;
  return n;
}

// Prototipos explicitos para o preprocessador da Arduino IDE.
void pwmSet(float valor);
void desligarTudo();
void iniciarOperacao();
void eventoBotao(uint8_t botao, bool longo);
void processarBotoes();
void tentarRecuperarSensores();
bool lerSensor(uint8_t i);
void adquirirTodos();
void atualizarControle();
void mostrarTela();

bool ack(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
bool lerReg(uint8_t addr, uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, uint8_t(1)) != 1) return false;
  out = Wire.read();
  return true;
}
bool gravarRegConferido(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(I2C_SENSOR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) return false;
  uint8_t rb = 0;
  return lerReg(I2C_SENSOR, reg, rb) && rb == val;
}
void deselecionarMux() {
  if (!muxAddr) return;
  Wire.beginTransmission(muxAddr);
  Wire.write(uint8_t(0));
  Wire.endTransmission();
}
bool selecionarMux(uint8_t canal) {
  if (!muxAddr || canal > 7) return false;
  Wire.beginTransmission(muxAddr);
  Wire.write(uint8_t(1U << canal));
  return Wire.endTransmission() == 0;
}
void resetMuxIsolado() {
  // PC817 INVERSOR ensaiado: GPIO19 HIGH -> RESET ativo (LOW no dominio ISO)
  digitalWrite(PIN_MUX_RST, HIGH);
  delay(90);
  digitalWrite(PIN_MUX_RST, LOW);
  delay(80);
}
bool encontrarMux() {
  muxAddr = 0;
  for (uint8_t a = 0x70; a <= 0x77; ++a) {
    if (ack(a)) { muxAddr = a; break; }
  }
  if (muxAddr) Serial.printf("[I2C] TCA9548A 0x%02X\n", muxAddr);
  else Serial.println("[I2C] MUX ausente");
  return muxAddr != 0;
}
Modelo detectarModelo(uint8_t i) {
  if (!selecionarMux(CH[i]) || !ack(I2C_SENSOR)) return Modelo::AUSENTE;
  uint8_t id41 = 0;
  if (lerReg(I2C_SENSOR, 0x92, id41) && (id41 & 0xFC) == 0x24)
    return Modelo::AS7341;
  if (as7343.begin() && as7343.getDeviceID() == 0x81)
    return Modelo::AS7343;
  return Modelo::DESCONHECIDO;
}
const char* tipoStr(Modelo t) {
  if (t == Modelo::AS7343) return "7343";
  if (t == Modelo::AS7341) return "7341";
  if (t == Modelo::DESCONHECIDO) return "ID?";
  return "----";
}
const char* motivoStr(Motivo m) {
  switch (m) {
    case Motivo::OK: return "OK";
    case Motivo::FALHA: return "FALHA";
    case Motivo::SATURADO: return "SAT";
    case Motivo::OBSTRUIDO: return "OBSTR";
    default: return "AUSENTE";
  }
}
bool inicializarSensor(uint8_t i) {
  // Reconexao independente: o cabo USB-C pode ter sido encaixado apos o boot.
  Ponto &p = pontos[i];
  p = Ponto();
  const Modelo esperado[N] = {Modelo::AS7343, Modelo::AS7341,
                             Modelo::AS7343, Modelo::AS7341, Modelo::AS7343};
  p.tipo = detectarModelo(i);
  if (p.tipo != esperado[i]) {
    Serial.printf("[S%u CH%u] Ausente ou modelo incorreto: %s\n",
                  i+1, CH[i], tipoStr(p.tipo));
    p.motivo = p.tipo == Modelo::AUSENTE ? Motivo::AUSENTE : Motivo::FALHA;
    deselecionarMux();
    return false;
  }
  bool ok = false;
  if (p.tipo == Modelo::AS7341) {
    ok = as7341[i].begin(I2C_SENSOR, &Wire);
    if (ok) ok = as7341[i].setATIME(29);
    if (ok) ok = as7341[i].setASTEP(599);
    if (ok) ok = as7341[i].setGain(AS7341_GAIN_32X);
  } else {
    ok = as7343.begin();
    if (ok) ok = as7343.powerOn();
    if (ok) ok = as7343.setAutoSmux(AUTOSMUX_18_CHANNELS);
    if (ok) ok = as7343.setAgain(AGAIN_32);
    if (ok) ok = gravarRegConferido(0x81, 29);
    if (ok) ok = gravarRegConferido(0xD4, 0x57);
    if (ok) ok = gravarRegConferido(0xD5, 0x02);
    if (ok) ok = as7343.enableSpectralMeasurement();
  }
  p.inicializado = ok;
  p.motivo = ok ? Motivo::FALHA : Motivo::AUSENTE;
  Serial.printf("[S%u CH%u] %s: %s\n", i+1, CH[i],
                tipoStr(p.tipo), ok ? "INICIALIZADO" : "FALHOU");
  deselecionarMux();
  return ok;
}
void inicializarSensores() {
  for (uint8_t i=0; i<N; ++i) inicializarSensor(i);
}

// Revarredura ao religar/encaixar sensores. Nunca mexer no MUX no meio
// de um ajuste autorizado nem alterar automaticamente o PWM por reconexao.
void tentarRecuperarSensores() {
  if (estado == Estado::CONTROLANDO || estado == Estado::CONFIRMAR) return;
  const uint32_t agora = millis();
  if (agora-ultimaTentativaSensor < RETENTATIVA_SENSOR_MS) return;
  ultimaTentativaSensor = agora;
  if (!muxAddr) {
    resetMuxIsolado();
    if (!encontrarMux()) return;
  }
  bool mudou = false;
  for (uint8_t i = 0; i<N; ++i) {
    if (pontos[i].inicializado) continue;
    if (inicializarSensor(i)) mudou = true;
  }
  if (mudou) {
    Serial.println("[RECUPERACAO] Sensor(es) reinicializado(s); atualizando leituras");
    telaSuja = true;
  }
}

uint8_t ativosIniciais() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < N; ++i) if (pontos[i].inicializado) ++n;
  return n;
}

void pwmSet(float valor) {
  if (!pwmPronto) { pwm = 0; return; }
  // Guardar e mostrar o PWM REAL de 8 bits, nao apenas o valor solicitado.
  const float pedido = constrain(valor, 0.0f, PWM_MAX);
  uint32_t duty = uint32_t(lroundf(pedido * 255.0f / 100.0f));
  pwm = 100.0f * float(duty) / 255.0f;
  ledcWrite(PIN_PWM, duty);
  alterouPWM = millis();
  telaSuja = true;
  Serial.printf("[PWM] %.1f %% (duty %lu/255)\n", pwm, (unsigned long)duty);
}
void desligarTudo() {
  pwmSet(0.0f);
  estado = Estado::OFF;
  // OFF deixa PWM=0 mas NAO significa que os sensores estao ausentes.
  // Uma nova leitura com LEDs apagados atualiza status e valores a ~0.
  nValidos = 0;
  eMedia = NAN;
  for (auto &p : pontos) {
    p.obstruido = false;
    p.contadorObstrucao = p.contadorRecuperacao = 0;
    p.leituraFisica = false;
    p.E = NAN;
  }
  ultimaLeitura = 0;
  consecutivosOK = 0;
  ciclosFora = 0;
  telaSuja = true;
  pagina = 0;
  Serial.println("[SISTEMA] OFF / PWM=0");
}
void iniciarOperacao() {
  if (!pwmPronto || !muxAddr || ativosIniciais() < SENSORES_MIN ||
      nValidos < SENSORES_MIN) {
    estado = Estado::FALHA_SENSORES;
    pwmSet(0.0f);
    pagina = 0;
    telaSuja = true;
    Serial.println("[BLOQUEIO] Necessarias >=4 leituras validas; execute D no Serial");
    return;
  }
  for (auto &p : pontos) {
    p.obstruido = false;
    p.contadorObstrucao = p.contadorRecuperacao = 0;
  }
  consecutivosOK = 0;
  ciclosFora = 0;
  passosControle = 0;
  limitePorTentativas = false;
  estado = Estado::INICIANDO;
  pwmSet(PWM_INICIO);
  ultimaLeitura = 0;
  pagina = 0;
  telaSuja = true;
  Serial.println("[SISTEMA] Partida com PWM=50%; aguardando estabilizacao");
}

// Evento CURTO chega mesmo se o operador pressionou/soltou durante
// uma leitura longa. Eventos nunca sao despachados dentro da ISR.
void eventoBotao(uint8_t b, bool longo) {
  Serial.printf("[BOTAO] %s %s, pagina=%u, estado=%u\n",
                b==0?"SELECT":b==1?"VOLTAR":b==2?"AVANCAR":"POWER",
                longo?"LONGO":"CURTO", pagina, uint8_t(estado));
  if (b == 3) {
    if (estado == Estado::OFF || estado == Estado::FALHA_SENSORES) iniciarOperacao();
    else desligarTudo();
    telaSuja = true;
    return;
  }
  // Navegacao SEMPRE habilitada, inclusive em OFF e com falha de sensores.
  if (b == 1) {
    if (estado == Estado::CONFIRMAR) estado = Estado::FORA_FAIXA;
    else pagina = (pagina + 2) % 3;
  } else if (b == 2) {
    pagina = (pagina + 1) % 3;
  } else if (b == 0) {
    if (estado == Estado::OFF || estado == Estado::FALHA_SENSORES ||
        estado == Estado::SEM_CALIBRACAO) return;
    if (longo) {
      if (estado == Estado::CONTROLANDO) {
        Serial.println("[CONTROLE] Cancelado pelo operador; PWM mantido");
        estado = Estado::MONITORANDO;
      } else if (estado == Estado::CONFIRMAR) estado = Estado::FORA_FAIXA;
    } else if (estado == Estado::CONFIRMAR) {
      if (nValidos >= SENSORES_MIN && isfinite(eMedia) &&
          totalSaturados() == 0 && !objetivoAtingido()) {
        estado = Estado::CONTROLANDO;
        consecutivosOK = 0;
        passosControle = 0;
        limitePorTentativas = false;
        Serial.println("[CONTROLE] AUTORIZADO PELO OPERADOR");
      } else estado = Estado::MONITORANDO;
    } else if (estado == Estado::FORA_FAIXA || estado == Estado::LIMITE_PWM) {
      if (nValidos >= SENSORES_MIN && !objetivoAtingido() &&
          totalSaturados() == 0) estado = Estado::CONFIRMAR;
    }
  }
  telaSuja = true;
}
void processarBotoes() {
  for (uint8_t i = 0; i < 4; ++i) {
    uint8_t curtos, longos;
    noInterrupts();
    curtos = botoesIRQ[i].curtos;
    longos = botoesIRQ[i].longos;
    botoesIRQ[i].curtos = botoesIRQ[i].longos = 0;
    interrupts();
    // Evitar excesso caso um botao esteja com mau contato.
    if (curtos > 3) curtos = 3;
    if (longos > 3) longos = 3;
    while (curtos--) eventoBotao(i, false);
    while (longos--) eventoBotao(i, true);
  }
}
bool lerSensor(uint8_t i) {
  Ponto &p = pontos[i];
  p.leituraFisica = false;
  p.rawPrincipal = p.rawAux = 0;
  p.E = NAN;
  if (!p.inicializado || !selecionarMux(CH[i]) || !ack(I2C_SENSOR)) {
    p.motivo = Motivo::AUSENTE;
    if (p.inicializado && ++p.falhasConsecutivas >= FALHAS_PARA_REINICIAR) {
      p.inicializado = false; // sera redetectado em uma futura revarredura
      Serial.printf("[S%u] Sem ACK repetidamente; programada redeteccao\n", i+1);
    }
    deselecionarMux();
    return false;
  }
  bool ok = false;
  uint16_t principal = 0, aux = 0;
  if (p.tipo == Modelo::AS7341) {
    Adafruit_AS7341 &s = as7341[i];
    if (s.startReading()) {
      uint32_t t0 = millis();
      while (millis()-t0 < LIMITE_AS7341_MS) {
        if (!ack(I2C_SENSOR)) break;
        if (s.checkReadingProgress()) { ok = true; break; }
        delay(5);
      }
      if (ok) {
        aux = s.getChannel(AS7341_CHANNEL_415nm_F1);
        principal = s.getChannel(AS7341_CHANNEL_445nm_F2);
      }
    }
  } else if (p.tipo == Modelo::AS7343) {
    ok = as7343.readSpectraDataFromSensor();
    if (ok) {
      aux = as7343.getChannelData(CH_DARK_BLUE_F2_425NM);
      principal = as7343.getBlue();
    }
  }
  deselecionarMux();
  if (!ok) {
    p.motivo = Motivo::FALHA;
    if (++p.falhasConsecutivas >= FALHAS_PARA_REINICIAR) {
      p.inicializado = false;
      Serial.printf("[S%u] Falhas consecutivas; programada reinicializacao\n", i+1);
    }
    return false;
  }
  p.falhasConsecutivas = 0;
  p.rawPrincipal = principal;
  p.rawAux = aux;
  if (principal >= CAL[i].maxRAW || aux >= CAL[i].maxRAW) {
    p.motivo = Motivo::SATURADO;
    return false;
  }
  // Contagem abaixo do fundo ou mesmo RAW=0 e leitura fisica VALIDA.
  // Uma falha I2C nunca e substituida artificialmente por zero.
  float e = CAL[i].a * fmaxf(0.0f, float(principal)-CAL[i].fundo);
  if (!isfinite(e) || e < 0.0f || e > 150.0f) {
    p.motivo = Motivo::FALHA;
    return false;
  }
  p.E = e;
  p.leituraFisica = true;
  p.motivo = p.obstruido ? Motivo::OBSTRUIDO : Motivo::OK;
  return true;
}
float esperado(uint8_t i, float pwmAtual) {
  // Nao extrapolar o perfil local usado na DETECCAO de obstrucao.
  if (pwmAtual < 10.0f || pwmAtual > PWM_MAX_CALIBRADO) return NAN;
  for (uint8_t k = 0; k < 4; ++k) {
    if (pwmAtual >= PERF_PWM[k] && pwmAtual <= PERF_PWM[k+1]) {
      const float u = (pwmAtual - PERF_PWM[k])/(PERF_PWM[k+1]-PERF_PWM[k]);
      return PERF_E[i][k] + u*(PERF_E[i][k+1]-PERF_E[i][k]);
    }
  }
  return PERF_E[i][4];
}
void analisarObstrucoes() {
  // Heuristica relativa e conservadora; nao substitui ensaio de validacao.
  // Perfil de obstrucao so foi caracterizado ate 75%. Acima disso, nao
  // manter bloqueios antigos baseados na interpolacao de uma curva invalida.
  if (extrapolando()) {
    for (uint8_t i=0; i<N; ++i) {
      pontos[i].obstruido = false;
      pontos[i].contadorObstrucao = pontos[i].contadorRecuperacao = 0;
      if (pontos[i].leituraFisica && pontos[i].motivo == Motivo::OBSTRUIDO)
        pontos[i].motivo = Motivo::OK;
    }
    return;
  }
  // Somente com pelo menos 4 leituras fisicas, PWM estavel e >=30%.
  if (pwm < 30.0f || millis()-alterouPWM < ESTABILIZACAO_PWM_MS) return;
  float normalizado[N];
  uint8_t nComparaveis = 0;
  for (uint8_t i=0; i<N; ++i) {
    float v=esperado(i,pwm);
    normalizado[i] = (pontos[i].leituraFisica && isfinite(v) && v>=0.8f)
                    ? pontos[i].E/v : NAN;
    if (isfinite(normalizado[i])) ++nComparaveis;
  }
  if (nComparaveis < SENSORES_MIN) return;
  for (uint8_t i=0; i<N; ++i) {
    if (!isfinite(normalizado[i])) continue;
    float soma=0; uint8_t qtd=0;
    for (uint8_t j=0; j<N; ++j) {
      if (i==j || !isfinite(normalizado[j]) || pontos[j].obstruido) continue;
      soma += normalizado[j]; ++qtd;
    }
    if (qtd < 3) continue;
    const float mediaOutros = soma/qtd;
    if (mediaOutros < 0.5f) continue; // reducao GLOBAL: nao rotular como obstaculo
    bool baixa = normalizado[i] < LIMIAR_OBSTRUCAO_REL * mediaOutros;
    bool recuperada = normalizado[i] > LIMIAR_RECUPERACAO_REL * mediaOutros;
    if (!pontos[i].obstruido) {
      pontos[i].contadorObstrucao = baixa ? min(255, int(pontos[i].contadorObstrucao)+1) : 0;
      if (pontos[i].contadorObstrucao >= CICLOS_OBSTRUCAO) {
        pontos[i].obstruido = true;
        pontos[i].contadorRecuperacao = 0;
        Serial.printf("[AVISO] Suspeita persistente de obstrucao S%u\n", i+1);
      }
    } else {
      pontos[i].contadorRecuperacao = recuperada ? min(255,int(pontos[i].contadorRecuperacao)+1) : 0;
      if (pontos[i].contadorRecuperacao >= CICLOS_RECUPERACAO) {
        pontos[i].obstruido = false;
        pontos[i].contadorObstrucao = 0;
        Serial.printf("[INFO] S%u reintegrado apos recuperacao\n",i+1);
      }
    }
    if (pontos[i].leituraFisica)
      pontos[i].motivo = pontos[i].obstruido ? Motivo::OBSTRUIDO : Motivo::OK;
  }
}
void adquirirTodos() {
  // Com PWM=0 (inclusive OFF) tambem fazemos leituras! Um valor 0.00
  // e um dado valido; status AUSENTE/FALHA fica reservado a erro real.
  for (uint8_t i=0; i<N; ++i) lerSensor(i);
  deselecionarMux();
  analisarObstrucoes();
  nValidos = 0;
  float soma = 0;
  for (uint8_t i=0; i<N; ++i) {
    Ponto &p = pontos[i];
    if (p.leituraFisica && !p.obstruido && p.motivo == Motivo::OK) {
      soma += p.E;
      nValidos++;
    }
  }
  eMedia = nValidos >= SENSORES_MIN ? soma/float(nValidos) : NAN;
  Serial.printf("DADOS;%lu;%.1f;%u;",(unsigned long)millis(),pwm,nValidos);
  if (isfinite(eMedia)) Serial.printf("%.3f;",eMedia);
  else Serial.print("NA;");
  for (uint8_t i=0;i<N;++i) {
    if (isfinite(pontos[i].E)) Serial.printf("%.3f",pontos[i].E);
    else Serial.print("NA");
    Serial.print(i==N-1?'\n':';');
  }
  Serial.printf("ALVO_8_9;%lu;%.1f;%s;EXTRAPOLADO=%u;SATURADOS=%u\n",
                (unsigned long)millis(),pwm,
                objetivoAtingido() ? "SIM_ESTIMADO" : "NAO",
                extrapolando() ? 1U : 0U, totalSaturados());
  telaSuja = true;
}

void atualizarControle() {
  if (estado == Estado::OFF || estado == Estado::FALHA_SENSORES) return;
  if (!isfinite(eMedia) || nValidos < SENSORES_MIN) {
    // Nao confundir ausencia de luz com indisponibilidade dos sensores.
    // Interrompe comando dos LEDs, MAS continua fazendo leituras de diagnostico
    // no loop, inclusive com PWM=0, permitindo recuperacao posterior.
    Serial.printf("[AVISO] Insuficientes: %u/5. PWM desativado; leitura continuara.\n",nValidos);
    estado = Estado::FALHA_SENSORES;
    if (DESLIGAR_EM_FALHA_DE_MEDICAO) pwmSet(0.0f);
    telaSuja = true;
    return;
  }
  if (estado == Estado::INICIANDO) estado = Estado::MONITORANDO;
  if (estado == Estado::SATURACAO && totalSaturados() == 0) {
    estado = Estado::MONITORANDO;
    ciclosFora = 0;
    Serial.println("[INFO] Sensores sem saturacao; monitoramento retomado");
  }
  if (estado != Estado::CONTROLANDO && totalSaturados() > 0) {
    estado = Estado::SATURACAO;
    return;
  }
  if (estado == Estado::CONFIRMAR) {
    if (totalSaturados()) estado = Estado::SATURACAO;
    return;
  }
  if (estado == Estado::CONTROLANDO) {
    // Qualquer canal saturado compromete a verificacao multiponto.
    // Nao aumentar o PWM utilizando uma media com o ponto saturado ausente.
    if (totalSaturados() > 0) {
      estado = Estado::SATURACAO;
      Serial.println("[CONTROLE] SUSPENSO: sensor saturado; confirmar no THOR");
      return;
    }
    // O controle so termina com 3 aquisicoes na META 8,0 a 9,0.
    if (objetivoAtingido()) {
      if (++consecutivosOK >= CICLOS_OK) {
        estado = Estado::CONCLUIDO;
        avisoConclusao = millis();
        Serial.printf("[CONTROLE] META 8-9 ATINGIDA (estimativa); E=%.3f PWM=%.1f%s\n",
                      eMedia,pwm,extrapolando()?" [EXTRAPOLADO: conferir THOR]":"");
      }
      return; // tres ciclos mantendo o mesmo PWM
    }
    consecutivosOK = 0;
    if (passosControle >= MAX_PASSOS) {
      estado = Estado::LIMITE_PWM;
      limitePorTentativas = true;
      Serial.println("[CONTROLE] Parado: maximo de tentativas");
      return;
    }
    const float erro = E_ALVO - eMedia;
    const float delta = constrain(KP*erro,-PASSO_MAX,PASSO_MAX);
    const float novo = constrain(pwm+delta,PWM_MIN_CONTROLE,PWM_MAX);
    const uint8_t dutyAtual = uint8_t(lroundf(pwm*255.0f/100.0f));
    int dutyNovo = int(lroundf(novo*255.0f/100.0f));
    if (dutyNovo == dutyAtual) {
      // Evitar falso "limite" por quantizacao de 8 bits.
      dutyNovo += (erro > 0 ? 1 : -1);
      dutyNovo = constrain(dutyNovo,int(lroundf(PWM_MIN_CONTROLE*255.0f/100.0f)),255);
    }
    if (dutyNovo == dutyAtual) {
      estado = Estado::LIMITE_PWM;
      limitePorTentativas = false;
      Serial.printf("[CONTROLE] PWM no limite fisico %.1f%%; E=%.3f (meta 8-9 nao atingida)\n",pwm,eMedia);
      return;
    }
    pwmSet(100.0f*float(dutyNovo)/255.0f);
    passosControle++;
    Serial.printf("[CONTROLE] passo=%u E=%.3f erro=%.3f PWM=%.1f%s\n",
                  passosControle,eMedia,erro,pwm,extrapolando()?" [EXTRAPOLADO]":"");
    return;
  }
  if (estado == Estado::CONCLUIDO && millis()-avisoConclusao < 3000) return;
  if (estado == Estado::LIMITE_PWM || estado == Estado::SATURACAO) return;
  // 9,0 a 10,0 permanece dentro da faixa GERAL, mas fora da META 8-9.
  // Ainda permite solicitar ajuste, em vez de ignorar a meta especifica.
  if (!objetivoAtingido()) {
    if (ciclosFora < 255) ++ciclosFora;
    if (ciclosFora >= 2) estado = Estado::FORA_FAIXA;
  } else {
    ciclosFora = 0;
    estado = Estado::MONITORANDO;
  }
}

// ============================================================================
// INTERFACE v1.2.1: ELEMENTOS FIXOS + SOBRESCRITA DOS CAMPOS VARIAVEIS
// - Tela e cartoes desenhados apenas ao entrar/mudar de pagina.
// - Cada campo tem largura FIXA; setTextColor(fg,bg) sobrescreve somente
//   os pixels dos caracteres, inclusive os espacos finais.
// - O cache evita comandos SPI quando texto e cor nao se alteraram.
// - A barra recebe apenas o trecho que aumentou/diminuiu.
// Nao ha fillScreen, fillRect de pagina, nem cartao dentro da atualizacao.
// ============================================================================
void escrever(int x, int y, const String &s, uint16_t cor, uint8_t escala=1) {
  tft.setTextSize(escala);
  tft.setTextColor(cor); // usado SOMENTE na camada estatica
  tft.setCursor(x,y);
  tft.print(s);
}
void cartao(int x,int y,int w,int h,uint16_t bg=CARTAO) {
  tft.fillRoundRect(x,y,w,h,10,bg);
  tft.drawRoundRect(x,y,w,h,10,CARTAO2);
}
uint16_t corEstado() {
  if (estado==Estado::CONTROLANDO) return CIANO;
  if (estado==Estado::CONFIRMAR) return AZUL;
  if (estado==Estado::FORA_FAIXA || estado==Estado::LIMITE_PWM ||
      estado==Estado::SATURACAO || estado==Estado::FALHA_SENSORES ||
      estado==Estado::SEM_CALIBRACAO) return AMARELO;
  if (extrapolando()) return AMARELO;
  if (estado==Estado::MONITORANDO || estado==Estado::CONCLUIDO)
    return nValidos == N && objetivoAtingido() ? VERDE : AMARELO;
  return CINZA;
}
const char *nomeEstado() {
  switch (estado) {
    case Estado::OFF: return "DESLIGADO";
    case Estado::INICIANDO: return "INICIANDO";
    case Estado::MONITORANDO: return objetivoAtingido() ? "META 8-9 OK" : "MONITORANDO";
    case Estado::FORA_FAIXA: return (isfinite(eMedia) && eMedia >= E_MIN &&
                         eMedia <= E_MAX) ? "FORA DO ALVO" : "FORA DA FAIXA";
    case Estado::CONFIRMAR: return "CONFIRMAR AJUSTE";
    case Estado::CONTROLANDO: return "AJUSTANDO";
    case Estado::CONCLUIDO: return "AJUSTE CONCLUIDO";
    case Estado::LIMITE_PWM: return limitePorTentativas ? "MAX DE PASSOS" : "LIMITE DO PWM";
    case Estado::SATURACAO: return "SENSOR SATURADO";
    case Estado::FALHA_SENSORES: return "FALHA NA MEDICAO";
    default: return "SEM CALIBRACAO";
  }
}

// Cada indice corresponde a UMA regiao de texto; nao e um buffer de tela.
enum CampoTela : uint8_t {
  CAM_SELO=0, CAM_RODAPE1, CAM_RODAPE2, CAM_META, CAM_IRRAD,
  CAM_PWM, CAM_VALIDOS,
  CAM_SENSOR_E_0, CAM_SENSOR_E_1, CAM_SENSOR_E_2, CAM_SENSOR_E_3, CAM_SENSOR_E_4,
  CAM_SENSOR_STATUS_0, CAM_SENSOR_STATUS_1, CAM_SENSOR_STATUS_2,
  CAM_SENSOR_STATUS_3, CAM_SENSOR_STATUS_4,
  CAM_ESTADO, CAM_PASSOS, TOTAL_CAMPOS
};
struct CacheCampo {
  String texto;
  uint16_t tinta=0, fundo=0;
  bool valido=false;
};
CacheCampo cacheTela[TOTAL_CAMPOS];
uint8_t paginaDesenhada = 255; // 255 = nenhuma pagina desenhada
uint16_t corSeloAnterior = 0xFFFF;
int larguraBarraAnterior = -1;
uint16_t corBarraAnterior = 0xFFFF;

void invalidarCampos() {
  for(uint8_t i=0;i<TOTAL_CAMPOS;++i) cacheTela[i].valido=false;
  corSeloAnterior=0xFFFF;
  larguraBarraAnterior=-1;
  corBarraAnterior=0xFFFF;
}

// Fonte padrao Adafruit_GFX tem largura 6*escala px por caracter.
// IMPORTANTE: as posicoes abaixo foram reservadas para NAO sobrepor rotulos.
void sobrescrever(uint8_t id, int x, int y, const String &texto,
                 uint16_t tinta, uint16_t fundoCampo,
                 uint8_t escala, uint8_t caracteres) {
  CacheCampo &c=cacheTela[id];
  if(c.valido && c.texto==texto && c.tinta==tinta && c.fundo==fundoCampo) return;
  String fixo=texto;
  if(fixo.length()>caracteres) fixo.remove(caracteres);
  while(fixo.length()<caracteres) fixo+=' ';
  tft.setTextSize(escala);
  tft.setTextColor(tinta,fundoCampo); // escrita OPACA, sobrescreve texto anterior
  tft.setCursor(x,y);
  tft.print(fixo);
  c.texto=texto;
  c.tinta=tinta;
  c.fundo=fundoCampo;
  c.valido=true;
}

void desenharCabecalhoEstatico() {
  escrever(10,7,"FOTOMETRIA",BRANCO,2);
  escrever(145,14,"TCC LAB",CINZA,1);
}
void desenharRodapeEstatico() {
  tft.drawFastHLine(10,204,300,CARTAO2);
}
void desenharPainelEstatico() {
  cartao(10,38,300,95);
  escrever(21,47,"IRRADIANCIA ESTIMADA",CINZA);
  escrever(168,83,"uW/cm2/nm",CINZA);
  // A barra fica no fundo. Duas marcas verticais delimitam a meta 8-9.
  tft.fillRoundRect(22,117,276,5,2,CARTAO2);
  tft.drawFastVLine(22+int(276*8.0f/12.0f),112,15,BRANCO);
  tft.drawFastVLine(22+int(276*9.0f/12.0f),112,15,BRANCO);
  cartao(10,139,145,55);
  escrever(21,148,"CONTROLE PWM",CINZA);
  cartao(165,139,145,55);
  escrever(176,148,"SENSORES VALIDOS",CINZA);
}
void desenharSensoresEstatico() {
  escrever(13,40,"LEITURAS POR PONTO",BRANCO,2);
  for(uint8_t i=0;i<N;++i) {
    const int y=67+i*27;
    cartao(10,y,300,24);
    escrever(18,y+8,"S"+String(i+1),CIANO);
    escrever(46,y+8,String(tipoStr(pontos[i].tipo)),CINZA);
  }
}
void desenharControleEstatico() {
  escrever(13,40,"ESTADO DO SISTEMA",BRANCO,2);
  cartao(10,66,300,52);
  escrever(20,76,"MODO",CINZA);
  cartao(10,123,300,75);
  escrever(20,132,"OBJETIVO: 8.0 - 9.0  |  ALVO: 8.5",BRANCO);
  escrever(20,147,"FAIXA GERAL: 8.0 - 10.0",CINZA);
  escrever(20,162,"PWM: 0 - 100%  |  BASE CAL: <=75%",CIANO);
  escrever(20,177,"PASSOS:",CINZA);
}
void desenharPaginaEstatico() {
  // UNICO ponto com limpeza de tela: entrada/troca de pagina.
  tft.fillScreen(FUNDO);
  invalidarCampos();
  desenharCabecalhoEstatico();
  desenharRodapeEstatico();
  if(pagina==0) desenharPainelEstatico();
  else if(pagina==1) desenharSensoresEstatico();
  else desenharControleEstatico();
  paginaDesenhada=pagina;
}

const char *textoSelo() {
  switch(estado) {
    case Estado::OFF: return "OFF";
    case Estado::MONITORANDO: return nValidos==N ? "MONITOR":"ATENCAO";
    case Estado::CONTROLANDO: return "CONTROLE";
    case Estado::LIMITE_PWM: return "LIMITE";
    case Estado::SATURACAO: return "SATURADO";
    case Estado::CONCLUIDO: return "CONCLUIDO";
    default: return "ATENCAO";
  }
}
void atualizarSelo() {
  const uint16_t cor=corEstado();
  if(cor != corSeloAnterior) {
    // Somente o pequeno selo e atualizado quando a COR do estado muda.
    tft.fillRoundRect(220,6,91,21,7,cor);
    corSeloAnterior=cor;
    cacheTela[CAM_SELO].valido=false;
  }
  sobrescrever(CAM_SELO,227,13,String(textoSelo()),FUNDO,cor,1,12);
}
void atualizarRodape(const String &linha1) {
  sobrescrever(CAM_RODAPE1,10,212,linha1,CINZA,FUNDO,1,50);
  sobrescrever(CAM_RODAPE2,10,226,
    extrapolando()?String("PWM>75: EXTRAPOLADO | CONFIRA THOR"):
                   String("BANCADA 30cm | SEM USO CLINICO"),CINZA,FUNDO,1,50);
}

// Sem apagar ou reconstruir a barra: pinta APENAS os pixels modificados.
void atualizarBarra() {
  const float frac=isfinite(eMedia)?constrain(eMedia/12.0f,0.0f,1.0f):0.0f;
  const int novo=int(276.0f*frac);
  const uint16_t cor=corEstado();
  if(larguraBarraAnterior==novo && corBarraAnterior==cor) return;
  if(larguraBarraAnterior<0 || corBarraAnterior!=cor) {
    // Inicializacao ou troca de cor. Apenas faixa de 5 px, nunca o cartao.
    tft.fillRect(22,117,276,5,CARTAO2);
    if(novo>0) tft.fillRect(22,117,novo,5,cor);
  } else if(novo>larguraBarraAnterior) {
    tft.fillRect(22+larguraBarraAnterior,117,novo-larguraBarraAnterior,5,cor);
  } else if(novo<larguraBarraAnterior) {
    tft.fillRect(22+novo,117,larguraBarraAnterior-novo,5,CARTAO2);
  }
  // Restaura as marcas nos dois x fixos caso a faixa tenha atravessado esses x.
  tft.drawFastVLine(22+int(276*8.0f/12.0f),112,15,BRANCO);
  tft.drawFastVLine(22+int(276*9.0f/12.0f),112,15,BRANCO);
  larguraBarraAnterior=novo;
  corBarraAnterior=cor;
}
void atualizarPainel() {
  sobrescrever(CAM_META,201,47,
    objetivoAtingido()?String(extrapolando()?"8-9: SIM*":"8-9: SIM"):
                       String("8-9: NAO"),
    objetivoAtingido()?(extrapolando()?AMARELO:VERDE):AMARELO,CARTAO,1,16);

  String valor=isfinite(eMedia)?String(eMedia,2):String("--.--");
  if(isfinite(eMedia) && eMedia>99.99f) valor=">99.9";
  sobrescrever(CAM_IRRAD,21,68,valor,isfinite(eMedia)?BRANCO:AMARELO,CARTAO,4,6);
  atualizarBarra();

  sobrescrever(CAM_PWM,21,163,String(pwm,1)+"%",CIANO,CARTAO,3,7);
  sobrescrever(CAM_VALIDOS,176,163,String(nValidos)+" / 5",
             nValidos>=SENSORES_MIN?VERDE:AMARELO,CARTAO,3,6);
  if(estado==Estado::OFF) atualizarRodape("POWER: ligar | LEDs desativados");
  else if(millis()-alterouPWM < ESTABILIZACAO_PWM_MS)
    atualizarRodape("Estabilizando fonte | leitura anterior");
  else if(estado==Estado::CONFIRMAR) atualizarRodape("SELEC: confirmar  VOLTAR: cancelar");
  else if(estado==Estado::CONTROLANDO) atualizarRodape("Ajustando... SELEC 1s: cancelar");
  else if(estado==Estado::FORA_FAIXA) atualizarRodape("SELEC: solicitar ajuste | setas: paginas");
  else if(estado==Estado::LIMITE_PWM)
    atualizarRodape(limitePorTentativas ? "Max passos | SELEC: tentar novamente":
                                         "Limite 100% ou 10% | meta nao alcancada");
  else if(estado==Estado::SATURACAO) atualizarRodape("SATURACAO: pare e confira THOR");
  else if(estado==Estado::FALHA_SENSORES) atualizarRodape("Falha: desligue/verifique sensores");
  else atualizarRodape("Setas: paginas | POWER: desligar");
}
void atualizarSensores() {
  for(uint8_t i=0;i<N;++i) {
    const int y=67+i*27;
    String valor=isfinite(pontos[i].E)?String(pontos[i].E,2):String("--.--");
    if(isfinite(pontos[i].E) && pontos[i].E>99.99f) valor=">99.9";
    sobrescrever(static_cast<CampoTela>(CAM_SENSOR_E_0+i),100,y+7,
      valor,isfinite(pontos[i].E)?BRANCO:AMARELO,CARTAO,2,7);
    sobrescrever(static_cast<CampoTela>(CAM_SENSOR_STATUS_0+i),237,y+8,
      String(motivoStr(pontos[i].motivo)),
      pontos[i].motivo==Motivo::OK?VERDE:AMARELO,CARTAO,1,11);
  }
  atualizarRodape("S1..S5 | E estimada em uW/cm2/nm");
}
void atualizarControleTela() {
  sobrescrever(CAM_ESTADO,20,95,String(nomeEstado()),corEstado(),CARTAO,2,22);
  sobrescrever(CAM_PASSOS,75,177,
               String(passosControle)+" / "+String(MAX_PASSOS),CINZA,CARTAO,1,12);
  if(estado==Estado::CONFIRMAR) atualizarRodape("CONFIRME: SELEC | CANCELE: VOLTAR");
  else if(extrapolando()) atualizarRodape("ACIMA 75: ESTIMATIVA EXTRAPOLADA");
  else atualizarRodape("Controle requer autorizacao do operador");
}
void mostrarTela() {
  const uint32_t agora=millis();
  const bool trocarPagina=(pagina!=paginaDesenhada);
  if(!trocarPagina && !telaSuja && agora-ultimaTela<ATUALIZACAO_TELA_MS) return;
  ultimaTela=agora;
  if(trocarPagina) desenharPaginaEstatico();
  // Nao redesenha nenhum cartao, titulo, borda ou fundo durante a leitura.
  atualizarSelo();
  if(pagina==0) atualizarPainel();
  else if(pagina==1) atualizarSensores();
  else atualizarControleTela();
  telaSuja=false;
}
void imprimirDiagnostico() {
  Serial.printf("[DIAG] Estado=%u PWM=%.1f MUX=0x%02X iniciados=%u/5 validos=%u/5 media=",
                uint8_t(estado),pwm,muxAddr,ativosIniciais(),nValidos);
  if (isfinite(eMedia)) Serial.println(String(eMedia,3));
  else Serial.println("NA (menos de 4 validos)");
  for(uint8_t i=0;i<N;++i) {
    const Ponto &p=pontos[i];
    Serial.printf("[DIAG] S%u CH%u modelo=%s init=%u motivo=%s RAW=%u AUX=%u E=",
                  i+1,CH[i],tipoStr(p.tipo),p.inicializado?1U:0U,
                  motivoStr(p.motivo),p.rawPrincipal,p.rawAux);
    if(isfinite(p.E)) Serial.println(String(p.E,3));
    else Serial.println("NA");
  }
}
void comandoSerial() {
  while (Serial.available()) {
    char c = toupper((unsigned char)Serial.read());
    if(c=='?') Serial.println("X=OFF I=iniciar P=proxima V=voltar D=diagnostico B=GPIO botoes R=reiniciar sensores (OFF) ?=ajuda");
    else if(c=='X') desligarTudo();
    else if(c=='I' && (estado==Estado::OFF || estado==Estado::FALHA_SENSORES)) iniciarOperacao();
    else if(c=='P') { pagina=(pagina+1)%3; telaSuja=true; }
    else if(c=='V') { pagina=(pagina+2)%3; telaSuja=true; }
    else if(c=='D') imprimirDiagnostico();
    else if(c=='B') {
      Serial.printf("[BOTOES] SEL GPIO18=%d VOL GPIO21=%d AVA GPIO22=%d POWER GPIO2=%d (0=pressionado)\n",
        digitalRead(PIN_SELECT),digitalRead(PIN_BACK),digitalRead(PIN_NEXT),digitalRead(PIN_POWER));
    } else if(c=='R') {
      if(estado==Estado::OFF || estado==Estado::FALHA_SENSORES) {
        pwmSet(0);
        resetMuxIsolado();
        if(encontrarMux()) inicializarSensores();
        ultimaLeitura=0;
        telaSuja=true;
      } else Serial.println("[R] Desligue com POWER/X antes de reiniciar sensores");
    }
  }
}
void setup() {
  Serial.begin(115200);
  delay(120);
  pinMode(PIN_PWM,OUTPUT);
  digitalWrite(PIN_PWM,LOW);
  pwmPronto = ledcAttach(PIN_PWM,PWM_FREQ,PWM_BITS); // ESP32 Arduino core 3.x
  pwmSet(0);
  pinMode(PIN_SELECT,INPUT_PULLUP);
  pinMode(PIN_BACK,INPUT_PULLUP);
  pinMode(PIN_NEXT,INPUT_PULLUP);
  pinMode(PIN_POWER,INPUT_PULLUP);
  pinMode(PIN_MUX_RST,OUTPUT);
  digitalWrite(PIN_MUX_RST,LOW);
  pinMode(PIN_TFT_LED,OUTPUT);
  digitalWrite(PIN_TFT_LED,HIGH);
  SPI.begin(PIN_TFT_SCK,PIN_TFT_MISO,PIN_TFT_MOSI,PIN_TFT_CS);
  tft.begin(4000000);
  tft.setRotation(1); // 320x240 landscape
  // A primeira apresentacao desenha a camada estatica.
  estado=Estado::INICIANDO;
  telaSuja=true;
  mostrarTela();
  Wire.begin(PIN_SDA,PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  resetMuxIsolado();
  if(encontrarMux()) inicializarSensores();
  desligarTudo();
  // Botao fisico ativo em LOW; todos os quatro podem ser pressionados
  // durante uma leitura lenta. Instalar interrupcoes apos o boot.
  for(uint8_t i=0;i<4;++i) {
    botoesIRQ[i].pressionado = (digitalRead(PIN_BOTOES[i])==LOW);
    botoesIRQ[i].iniciouUs = micros();
    botoesIRQ[i].ultimaSolturaUs = micros()-60000UL;
  }
  attachInterrupt(digitalPinToInterrupt(PIN_SELECT),irqSelecionar,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BACK),irqVoltar,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_NEXT),irqAvancar,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_POWER),irqPower,CHANGE);
  // Primeira leitura com LEDs apagados: 0.00 e um valor valido.
  if(muxAddr) adquirirTodos();
  ultimaLeitura = millis();
  Serial.println("[BOOT] Firmware TCC v1.2.2 / CAL_30 / PWM max 100 / 5 sensores");
  Serial.printf("[BOOT] MUX=%02X sensores iniciais=%u/5 PWM=%s\n",muxAddr,ativosIniciais(),pwmPronto?"OK":"FALHA");
  Serial.println("CSV: DADOS;tempo_ms;PWM;validos;Erep;E1;E2;E3;E4;E5");
  mostrarTela();
}
void loop() {
  processarBotoes();
  comandoSerial();
  tentarRecuperarSensores();
  const uint32_t agora=millis();
  const bool estabilizou=(agora-alterouPWM >= ESTABILIZACAO_PWM_MS);
  // Ler em TODOS os estados, inclusive OFF e FALHA_SENSORES.
  // Isso mantem o status atualizado e permite verificar os 5 sensores
  // com irradiancia nula, sem ligar os LEDs.
  if ((estado==Estado::OFF || estado==Estado::FALHA_SENSORES ||
       (estabilizou && estado!=Estado::SEM_CALIBRACAO)) &&
      agora-ultimaLeitura >= PERIODO_MEDIDA_MS) {
    ultimaLeitura=agora;
    adquirirTodos();
    if(estado!=Estado::OFF && estado!=Estado::FALHA_SENSORES)
      atualizarControle();
  }
  // Se a captura I2C demorou, trata os cliques que a interrupcao guardou.
  processarBotoes();
  mostrarTela();
  delay(5);
}
