/*
  TCC - Monitoramento e Controle de Irradiancia Espectral - v1.2.7 experimental / registro persistente LittleFS - obstrucao com tolerancia temporal de 10 s; OFF sem comparacao percentual
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
#include <LittleFS.h>
#include <Preferences.h>

// ==== PINAGEM FINAL, ver Quadro 5 da monografia e sketch ensaiado ====
constexpr uint8_t PIN_SDA = 17, PIN_SCL = 16, PIN_MUX_RST = 19;
constexpr uint8_t PIN_PWM = 32;
constexpr uint8_t PIN_TFT_CS = 26, PIN_TFT_DC = 33, PIN_TFT_RST = 13;
constexpr uint8_t PIN_TFT_MOSI = 14, PIN_TFT_SCK = 25;
constexpr uint8_t PIN_TFT_MISO = 36, PIN_TFT_LED = 27;
constexpr uint8_t PIN_SELECT = 18, PIN_BACK = 21, PIN_NEXT = 22;
// Sem botao POWER no GPIO2: SELECT por >=2s liga/desliga a OPERACAO.
// O ESP32 permanece acordado e continua lendo os sensores no modo OFF.
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

// Heuristica solicitada para a BANCADA: NAO usar perfis esperados por posicao.
// Um sensor e sinalizado como "obstruido" somente se:
//   (a) nao ha comunicacao/identificacao I2C (motivo AUSENTE); ou
//   (b) sua irradiancia diverge >=90% da irradiancia do OUTRO sensor
//       fisicamente valido com valor mais proximo.
// Divergencia simetrica: abs(Ei-Ej)/max(Ei,Ej). Assim 15 vs 2 => 86.7%
// (ACEITO, apesar da assimetria da malha) e 20 vs 1 => 95% (OBSTR).
// Se a media PRELIMINAR dos 5 pontos esta <=0.5 uW/cm2/nm, nao comparar
// percentual; no estado OFF, somente perda de I2C conta como obstrucao.
// Duas obstrucoes simultaneas devem persistir >=10s para parar a operacao.
// "Obstrucao" e apenas indicador operacional, nao diagnostico optico.
constexpr float LIMIAR_DIVERGENCIA_REL = 0.90f;
constexpr float PISO_COMPARACAO_UW = 0.50f;
// Se a MEDIA PRELIMINAR dos sensores fisicamente lidos estiver muito
// proxima de zero, NAO e adequado usar divergencia percentual de 90%.
constexpr float PISO_MEDIA_ESCURA_UW = 0.50f;
// Duas ou mais obstrucoes simultaneas devem persistir por pelo menos 10s
// para causar o desligamento protetivo da operacao.
constexpr uint32_t TEMPO_OBSTRUCAO_DUPLA_MS = 10000UL;

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
  bool obstruido = false;
  uint8_t referenciaMaisProxima = 0; // S1..S5, ou 0 se nao comparavel
  float divergenciaRel = NAN;         // 0.0..1.0; nao confundir com RAW
  uint8_t falhasConsecutivas = 0;
};
// Os sensores AS734x podem ocupar o loop por >100ms. As interrupcoes
// capturam pressionar/soltar mesmo durante uma leitura I2C demorada.
// SELECT: 2 s = POWER, toque curto = confirmar. A acao de 2 s pode ocorrer
// enquanto o botao permanece pressionado, e nao se repete ao soltar.
// BACK: 1 s cancela controle automatico, sem desligar os LEDs.
constexpr uint32_t SELECT_POWER_US = 2000000UL;
constexpr uint32_t OUTRO_LONGO_US = 1000000UL;
constexpr uint8_t TOTAL_BOTOES = 3;
struct BotaoIRQ {
  volatile bool pressionado = false;
  volatile bool longoConsumido = false;
  volatile uint32_t iniciouUs = 0;
  volatile uint32_t ultimaSolturaUs = 0;
  volatile uint8_t curtos = 0;
  volatile uint8_t longos = 0;
};
constexpr uint8_t PIN_BOTOES[TOTAL_BOTOES] = {PIN_SELECT, PIN_BACK, PIN_NEXT};
BotaoIRQ botoesIRQ[TOTAL_BOTOES];

void IRAM_ATTR registrarBorda(uint8_t i) {
  const uint32_t agora = micros();
  const bool pressionou = (digitalRead(PIN_BOTOES[i]) == LOW);
  BotaoIRQ &b = botoesIRQ[i];
  if (pressionou) {
    if (!b.pressionado && uint32_t(agora - b.ultimaSolturaUs) >= 50000UL) {
      b.pressionado = true;
      b.longoConsumido = false;
      b.iniciouUs = agora;
    }
  } else if (b.pressionado) {
    const uint32_t duracao = uint32_t(agora - b.iniciouUs);
    // Descarta pulsos de bounce inferiores a 35 ms.
    if (duracao >= 35000UL) {
      b.pressionado = false;
      b.ultimaSolturaUs = agora;
      if (!b.longoConsumido) {
        const uint32_t limiar = (i == 0) ? SELECT_POWER_US : OUTRO_LONGO_US;
        if (duracao >= limiar) {
          if (b.longos < 255) ++b.longos;
        } else {
          if (b.curtos < 255) ++b.curtos;
        }
      }
      b.longoConsumido = false;
    }
  }
}
void IRAM_ATTR irqSelecionar() { registrarBorda(0); }
void IRAM_ATTR irqVoltar()    { registrarBorda(1); }
void IRAM_ATTR irqAvancar()  { registrarBorda(2); }

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
// ==== REGISTRO PERSISTENTE PARA RECUPERAR APOS DESLIGAR A FONTE ====
// Duas janelas de arquivo: as amostras mais antigas sao descartadas ao girar.
// Nao gravar em NVS a cada leitura: dados sequenciais ficam no LittleFS.
// Cada linha e fechada imediatamente (maior tolerancia a desligamento brusco).
// Formato CSV ';' para Excel BR. Arquivos permanecem ao tirar alimentacao.
constexpr size_t TAMANHO_LOG_JANELA = 220UL * 1024UL;
constexpr const char* ARQUIVO_LOG_ATUAL = "/tcc_atual.csv";
constexpr const char* ARQUIVO_LOG_ANTERIOR = "/tcc_anterior.csv";
constexpr const char* CABECALHO_LOG =
  "sessao;ms;tipo;estado;pwm_pct;validos;Erep;E1;E2;E3;E4;E5;"
  "S1;S2;S3;S4;S5;RAW1;RAW2;RAW3;RAW4;RAW5;"
  "obstruidos;dupla_ms;passos;pagina;meta8a9;extrapolado;evento";
bool logDisponivel = false;
bool avisoLogFalha = false;
bool limparLogArmado = false;
uint32_t instanteArmadoLog = 0;
uint32_t sessaoLog = 0;
uint8_t amostrasFalhaSalvas = 0;
uint32_t amostrasPersistidas = 0;
uint32_t eventosPersistidos = 0;

void iniciarLogPersistente();
void registrarEvento(const char* evento);
void registrarAmostra();
void exportarLog();
void resumirLog();
void apagarLogConfirmado();

// Temporizador nao bloqueante: duas ou mais obstrucoes ao mesmo tempo.
// Conta SOMENTE quando a operacao esta ligada; OFF ignora OBSTR/90%.
uint8_t quantidadeObstruidos = 0;
bool obstrucaoDuplaAtiva = false;
bool obstrucaoDuplaConfirmada = false;
uint32_t inicioObstrucaoDuplaMs = 0;
uint32_t duracaoObstrucaoDuplaMs = 0;

void limparTemporizadorObstrucao() {
  quantidadeObstruidos = 0;
  obstrucaoDuplaAtiva = false;
  obstrucaoDuplaConfirmada = false;
  inicioObstrucaoDuplaMs = 0;
  duracaoObstrucaoDuplaMs = 0;
}

// Nenhum delay(): o sistema continua lendo os sensores e atualizando o TFT.
void atualizarTemporizadorObstrucao() {
  uint8_t quantos = 0;
  for (uint8_t i=0; i<N; ++i)
    if (pontos[i].obstruido) ++quantos;
  quantidadeObstruidos = quantos;
  if (estado == Estado::OFF || quantos < 2) {
    if (obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada)
      Serial.println("[OBSTRUCAO] Dupla deixou de ser simultanea; temporizador reiniciado");
    if (obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada) registrarEvento("OBSTRUCAO_DUPLA_RECUPEROU");
    obstrucaoDuplaAtiva = false;
    obstrucaoDuplaConfirmada = false;
    duracaoObstrucaoDuplaMs = 0;
    return;
  }
  const uint32_t agora = millis();
  if (!obstrucaoDuplaAtiva) {
    inicioObstrucaoDuplaMs = agora;
    duracaoObstrucaoDuplaMs = 0;
    obstrucaoDuplaAtiva = true;
    obstrucaoDuplaConfirmada = false;
    Serial.printf("[OBSTRUCAO] %u sensores simultaneos: iniciando espera de 10s\n", quantos);
    registrarEvento("OBSTRUCAO_DUPLA_INICIO");
  } else {
    duracaoObstrucaoDuplaMs = uint32_t(agora - inicioObstrucaoDuplaMs);
  }
  if (!obstrucaoDuplaConfirmada && duracaoObstrucaoDuplaMs >= TEMPO_OBSTRUCAO_DUPLA_MS) {
    obstrucaoDuplaConfirmada = true;
    Serial.printf("[OBSTRUCAO] %u sensores obstruidos por >=10s: protecao habilitada\n", quantos);
    registrarEvento("OBSTRUCAO_DUPLA_10S");
  }
}

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
uint8_t ativosIniciais();
void diagnosticarI2C();
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
void reiniciarBarramentoI2C() {
  // Na campanha de aquisicao validada o barramento era encerrado e
  // reiniciado apos o RESET do TCA, com clock conservador (100 kHz).
  Wire.end();
  delay(10);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
}
void resetMuxIsolado(bool inicial = false) {
  // Polaridade do circuito COM PC817 INVERSOR da PCB ensaiada:
  // GPIO19 HIGH => LED do PC817 ativo => RST_ISO LOW (RESET ativo).
  // GPIO19 LOW => LED apagado => RST_ISO HIGH pelo resistor pull-up.
  // ATENCAO: isso pressupoe a montagem inversora; o GPIO nao mede RST_ISO.
  // Restaurado o procedimento inicial do sketch de aquisicao que funcionava.
  const uint32_t duracao = inicial ? 5000UL : 150UL;
  Serial.printf("[MUX RESET] GPIO19 HIGH por %lu ms (RESET ativo via PC817)\n",
                (unsigned long)duracao);
  digitalWrite(PIN_MUX_RST, HIGH);
  delay(duracao);
  digitalWrite(PIN_MUX_RST, LOW);
  delay(50);
  Serial.println("[MUX RESET] GPIO19 LOW (RESET liberado); reiniciando Wire");
  reiniciarBarramentoI2C();
  muxAddr = 0; // qualquer endereco antigo deve ser novamente verificado
}
// Ler o registro de controle do TCA9548A sem alterar os canais.
bool lerControleMux(uint8_t &valor) {
  if (!muxAddr) return false;
  if (Wire.requestFrom(muxAddr, uint8_t(1)) != 1) return false;
  valor = Wire.read();
  return true;
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
    p.obstruido = (p.motivo == Motivo::AUSENTE);
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

// Diagnostico seguro somente com fonte de LEDs parada.
// Le o estado real do seletor TCA, o ACK de cada AS734x e, quando possivel,
// o WHOAMI do AS7341. Nao depende de as7343.begin()/calibracao/irradiancia.
void diagnosticarI2C() {
  Serial.println("\n========== DIAGNOSTICO DO I2C ISOLADO ==========");
  Serial.printf("[I2C] GPIO SDA=%u SCL=%u RESET_PC817=%u (LOW esperado liberado)\n",
                PIN_SDA, PIN_SCL, (unsigned)digitalRead(PIN_MUX_RST));
  if (!encontrarMux()) {
    Serial.println("[I2C] SEM ACK do TCA em 0x70..0x77: verificar ISO1540, 3V3_ISO, RST_ISO, SDA/SCL");
    Serial.println("================================================\n");
    return;
  }
  uint8_t controle = 0;
  bool regOK = lerControleMux(controle);
  Serial.printf("[I2C] MUX=0x%02X; controle antes=%s 0x%02X\n",
                muxAddr, regOK ? "OK" : "ERRO", controle);
  for (uint8_t i = 0; i < N; ++i) {
    const uint8_t mascara = uint8_t(1U << CH[i]);
    const bool escritaOK = selecionarMux(CH[i]);
    uint8_t lido = 0;
    const bool retornoOK = escritaOK && lerControleMux(lido);
    const bool canalOK = retornoOK && lido == mascara;
    const bool sensorACK = canalOK && ack(I2C_SENSOR);
    uint8_t id41=0;
    bool id41OK = sensorACK && lerReg(I2C_SENSOR, 0x92, id41);
    Serial.printf("[I2C] S%u CH%u: escrita=%s retorno=%s reg=0x%02X esperado=0x%02X sensor_0x39=%s WHOAMI_0x92=%s0x%02X\n",
                  i+1, CH[i], escritaOK?"OK":"ERRO", canalOK?"OK":"ERRO",
                  lido, mascara, sensorACK?"ACK":"SEM ACK",
                  id41OK?"":"NA/", id41);
  }
  deselecionarMux();
  Serial.println("[I2C] Se seletor=OK mas todos sensor_0x39=SEM ACK, verificar alimentacao e cabeamento USB-C dos sensores.");
  Serial.println("================================================\n");
}

// Revarredura ao religar/encaixar sensores. Nunca mexer no MUX no meio
// de um ajuste autorizado nem alterar automaticamente o PWM por reconexao.
void tentarRecuperarSensores() {
  if (estado == Estado::CONTROLANDO || estado == Estado::CONFIRMAR) return;
  const uint32_t agora = millis();
  if (agora-ultimaTentativaSensor < RETENTATIVA_SENSOR_MS) return;
  ultimaTentativaSensor = agora;
  // Se TODOS desapareceram, o problema pode ser global (RESET, I2C,
  // alimentacao isolada). Nao continuar tentando apenas detectar modelos.
  // Reexecutar RESET + reinicio Wire somente quando nenhum sensor esta ativo.
  if (!muxAddr || ativosIniciais() == 0) {
    Serial.println("[RECUPERACAO] 0 sensores: rearmando TCA/I2C antes da redeteccao");
    resetMuxIsolado(false);
    if (!encontrarMux()) return;
  }
  bool mudou = false;
  for (uint8_t i = 0; i<N; ++i) {
    if (pontos[i].inicializado) continue;
    if (inicializarSensor(i)) mudou = true;
  }
  if (mudou) {
    Serial.println("[RECUPERACAO] Sensor(es) reinicializado(s); atualizando leituras");
    ultimaLeitura = 0;
    telaSuja = true;
  }
}

uint8_t ativosIniciais() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < N; ++i) if (pontos[i].inicializado) ++n;
  return n;
}

// === LittleFS: montagem conservadora, sem apagar logs existentes ===
void iniciarLogPersistente() {
  Preferences nvs;
  bool nvsOK = nvs.begin("tcc-log", false);
  const bool jaInicializado = nvsOK ? nvs.getBool("fspronto", false) : true;
  logDisponivel = LittleFS.begin(false); // NUNCA reformatar se ja havia logs.
  if (!logDisponivel && !jaInicializado) {
    Serial.println("[LOG] Primeira instalacao: preparando particao LittleFS");
    if (LittleFS.format()) logDisponivel = LittleFS.begin(false);
  }
  if (logDisponivel) {
    if (nvsOK) {
      if (!jaInicializado) nvs.putBool("fspronto", true);
      sessaoLog = nvs.getUInt("sessao", 0) + 1;
      nvs.putUInt("sessao", sessaoLog);
    } else {
      sessaoLog = 1;
    }
    Serial.printf("[LOG] PRONTO; sessao=%lu; tamanho=%lu; ocupado=%lu bytes\n",
        (unsigned long)sessaoLog, (unsigned long)LittleFS.totalBytes(),
        (unsigned long)LittleFS.usedBytes());
    File f = LittleFS.open(ARQUIVO_LOG_ATUAL, FILE_APPEND);
    if (f) {
      if (f.size() == 0) f.println(CABECALHO_LOG);
      f.close();
    } else {
      logDisponivel = false;
      Serial.println("[LOG] ERRO: nao abriu arquivo de historico");
    }
  } else {
    Serial.println("[LOG] ERRO: LittleFS nao montado. Verifique se a particao selecionada inclui SPIFFS/LittleFS.");
    if (jaInicializado) Serial.println("[LOG] NAO formatado automaticamente para proteger possiveis dados anteriores.");
  }
  if (nvsOK) nvs.end();
}

// Uma linha por aquisicao/evento. Nao manter arquivo aberto para que
// o operador possa desligar a fonte apos o teste sem precisar de Serial.
void escreverRegistroPersistente(const char* tipo, const char* evento) {
  if (!logDisponivel) return;
  String linha;
  linha.reserve(320);
  linha += String(sessaoLog); linha += ';';
  linha += String(millis()); linha += ';';
  linha += tipo; linha += ';';
  linha += String(uint8_t(estado)); linha += ';';
  linha += String(pwm,2); linha += ';';
  linha += String(nValidos); linha += ';';
  linha += isfinite(eMedia) ? String(eMedia,3) : String("NA");
  for (uint8_t i=0;i<N;++i) {
    linha+=';';
    linha += isfinite(pontos[i].E) ? String(pontos[i].E,3) : String("NA");
  }
  for (uint8_t i=0;i<N;++i) {
    linha+=';';
    linha+=String(uint8_t(pontos[i].motivo));
  }
  for (uint8_t i=0;i<N;++i) {
    linha+=';';
    linha+=String(pontos[i].rawPrincipal);
  }
  linha+=';'; linha+=String(quantidadeObstruidos);
  linha+=';'; linha+=String(duracaoObstrucaoDuplaMs);
  linha+=';'; linha+=String(passosControle);
  linha+=';'; linha+=String(pagina);
  linha+=';'; linha+=objetivoAtingido() ? "1" : "0";
  linha+=';'; linha+=extrapolando() ? "1" : "0";
  linha+=';'; linha+=evento;
  if (linha.length() > 500) { avisoLogFalha = true; telaSuja=true; return; }

  // Politica FIFO de 2 arquivos, tamanho limitado. Renomear antes de escrever.
  File f = LittleFS.open(ARQUIVO_LOG_ATUAL, FILE_APPEND);
  if (!f) { avisoLogFalha = true; telaSuja=true; return; }
  if (f.size() + linha.length() + 3 > TAMANHO_LOG_JANELA) {
    f.close();
    if (LittleFS.exists(ARQUIVO_LOG_ANTERIOR)) LittleFS.remove(ARQUIVO_LOG_ANTERIOR);
    if (!LittleFS.rename(ARQUIVO_LOG_ATUAL, ARQUIVO_LOG_ANTERIOR)) {
      avisoLogFalha = true;
      telaSuja=true;
      Serial.println("[LOG] ERRO: rotacao do historico falhou");
      return;
    }
    f = LittleFS.open(ARQUIVO_LOG_ATUAL, FILE_WRITE);
    if (!f) { avisoLogFalha = true; telaSuja=true; return; }
    f.println(CABECALHO_LOG);
  }
  const size_t bytes = f.println(linha);
  f.flush();
  f.close();
  if (bytes != linha.length()+2 && bytes != linha.length()+1) {
    avisoLogFalha = true;
    telaSuja=true;
    Serial.println("[LOG] ERRO: escrita incompleta");
  }
  if (tipo[0]=='M') ++amostrasPersistidas;
  else ++eventosPersistidos;
}

void registrarEvento(const char* evento) { escreverRegistroPersistente("E",evento); }
void registrarAmostra() {
  // Nao usar todo o limite da flash com a placa em OFF ou parada em falha.
  if (estado==Estado::OFF) return;
  if (estado==Estado::FALHA_SENSORES) {
    if (amostrasFalhaSalvas>=4) return;
    ++amostrasFalhaSalvas;
  } else amostrasFalhaSalvas=0;
  escreverRegistroPersistente("M","AMOSTRA");
}

// Gera relatorio da ULTIMA operacao iniciada, mesmo apos reiniciar pelo USB.
// Le uma linha por vez; nao aloca o arquivo inteiro na RAM do ESP32.
void processarResumoArquivo(const char* path,
                           uint32_t &nAmostras, uint32_t &validos5,
                           uint32_t &validos4, uint32_t &validosMenos4,
                           uint32_t &metas, uint32_t &obstrucoes,
                           uint32_t &duplas, uint32_t &eventosBotoes,
                           uint32_t &falhasCom, uint32_t &ajustesAutorizados,
                           uint32_t &atualizacoesControle,
                           uint32_t &intervalos, uint64_t &somaIntervalos,
                           uint32_t &minIntervalo, uint32_t &maxIntervalo,
                           uint32_t &ultimaHora, uint32_t &ultimaSessao,
                           bool &temUltima, bool &achouInicio,
                           uint32_t &seqNormal, uint32_t &maxNormal,
                           uint32_t &seqIntervalos, uint32_t &maxSeqIntervalos,
                           uint32_t *ultimos50, uint8_t &qtde50, uint8_t &indice50) {
  File f=LittleFS.open(path,FILE_READ);
  if (!f) return;
  while (f.available()) {
    String linha=f.readStringUntil('\n');
    linha.trim();
    if (linha.length()==0 || linha.startsWith("sessao;")) continue;
    // Somente campos necessarios, com limites simples para CSV sem aspas.
    String v[29];
    uint8_t idx=0;
    int inicio=0;
    for (; idx<28; ++idx) {
      int fim=linha.indexOf(';',inicio);
      if (fim<0) break;
      v[idx]=linha.substring(inicio,fim);
      inicio=fim+1;
    }
    if (idx!=28) continue;
    v[28]=linha.substring(inicio);
    const String &ev=v[28];
    if (v[2]=="E" && ev=="OPERACAO_INICIADA") {
      achouInicio=true;
      nAmostras=validos5=validos4=validosMenos4=metas=0;
      obstrucoes=duplas=eventosBotoes=falhasCom=0;
      ajustesAutorizados=atualizacoesControle=0;
      intervalos=0; somaIntervalos=0;
      minIntervalo=0xFFFFFFFFUL; maxIntervalo=0;
      temUltima=false;
      ultimaSessao=0;
      seqNormal=maxNormal=seqIntervalos=maxSeqIntervalos=0;
      qtde50=indice50=0;
      continue;
    }
    if (!achouInicio) continue;
    if (v[2]=="E") {
      if(ev.startsWith("BT_")) ++eventosBotoes;
      if(ev=="AJUSTE_AUTORIZADO") ++ajustesAutorizados;
      if(ev=="OBSTRUCAO_DUPLA_INICIO") ++duplas;
      if(ev.startsWith("S") && ev.endsWith("_AUSENTE")) ++falhasCom;
      if(ev=="PWM_ALTERADO") ++atualizacoesControle;
      continue;
    }
    if (v[2]!="M") continue;
    ++nAmostras;
    const int numValidos=v[5].toInt();
    if(numValidos==5) ++validos5;
    else if(numValidos==4) ++validos4;
    else ++validosMenos4;
    if(v[26]=="1") ++metas;
    if(v[22].toInt()>=1) ++obstrucoes;
    // Normal = 5 validos, nenhuma obstrucao, LED energizado.
    if(numValidos==5 && v[22].toInt()==0 && v[4].toFloat()>0.1f) {
      ++seqNormal;
      if(seqNormal>maxNormal) maxNormal=seqNormal;
    } else seqNormal=0;
    const uint32_t s=v[0].toInt();
    const uint32_t t=v[1].toInt();
    if(temUltima && s==ultimaSessao) {
      const uint32_t delta=t-ultimaHora;
      // Ultimos 50 intervalos: usa lista compacta de ate 50 valores.
      // Para nao carregar grande vetor, o resumo mostra todos os intervalos
      // dentro da ultima operacao; CSV permite selecionar exatamente 50.
      if(delta>=100UL && delta<=10000UL) {
        ++intervalos; somaIntervalos+=delta;
        if(delta<minIntervalo) minIntervalo=delta;
        if(delta>maxIntervalo) maxIntervalo=delta;
        ++seqIntervalos;
        if(seqIntervalos>maxSeqIntervalos) maxSeqIntervalos=seqIntervalos;
        ultimos50[indice50]=delta;
        indice50=uint8_t((indice50+1)%50);
        if(qtde50<50) ++qtde50;
      } else {
        seqIntervalos=0;
        qtde50=indice50=0;
      }
    } else {
      seqIntervalos=0;
      qtde50=indice50=0;
    }
    ultimaHora=t; ultimaSessao=s; temUltima=true;
  }
  f.close();
}

void resumirLog() {
  if (estado!=Estado::OFF && estado!=Estado::FALHA_SENSORES) {
    Serial.println("[LOG] Desligue a operacao antes de calcular o resumo.");
    return;
  }
  if (!logDisponivel) { Serial.println("[LOG] INDISPONIVEL - nada foi salvo"); return; }
  uint32_t n=0,n5=0,n4=0,nMenos4=0,metas=0,obs=0,duplas=0,bot=0,
           ausentes=0,ajustes=0,mudancasPWM=0,intv=0,minI=0xFFFFFFFFUL,maxI=0,lastMs=0,lastSes=0;
  uint64_t somaI=0;
  bool temUltima=false, achouInicio=false;
  uint32_t seqNormal=0,maxNormal=0,seqInt=0,maxSeqInt=0,ultimos50[50]={0};
  uint8_t qtd50=0,pos50=0;
  processarResumoArquivo(ARQUIVO_LOG_ANTERIOR,n,n5,n4,nMenos4,metas,obs,duplas,
      bot,ausentes,ajustes,mudancasPWM,intv,somaI,minI,maxI,lastMs,lastSes,
      temUltima,achouInicio,seqNormal,maxNormal,seqInt,maxSeqInt,ultimos50,qtd50,pos50);
  processarResumoArquivo(ARQUIVO_LOG_ATUAL,n,n5,n4,nMenos4,metas,obs,duplas,
      bot,ausentes,ajustes,mudancasPWM,intv,somaI,minI,maxI,lastMs,lastSes,
      temUltima,achouInicio,seqNormal,maxNormal,seqInt,maxSeqInt,ultimos50,qtd50,pos50);
  Serial.println("\n================ RESUMO PERSISTENTE TCC ================");
  Serial.printf("[LOG] Sessao de leitura USB=%lu | Flash total=%lu usado=%lu bytes\n",
    (unsigned long)sessaoLog,(unsigned long)LittleFS.totalBytes(),
    (unsigned long)LittleFS.usedBytes());
  Serial.printf("[LOG] Escritos neste boot: amostras=%lu eventos=%lu\n",
    (unsigned long)amostrasPersistidas,(unsigned long)eventosPersistidos);
  if (!achouInicio) {
    Serial.println("[LOG] Nenhum evento OPERACAO_INICIADA encontrado na janela retida.");
    Serial.println("[LOG] Use H para inspecionar CSV completo; talvez arquivo antigo tenha sido rotacionado.");
    Serial.println("=======================================================\n");
    return;
  }
  Serial.println("[LOG] Ultima operacao iniciada, inclusive se houver reboot posterior:");
  Serial.printf("[LOG] ciclos=%lu | 5 validos=%lu | 4 validos=%lu | <=3 validos=%lu\n",
      (unsigned long)n,(unsigned long)n5,(unsigned long)n4,(unsigned long)nMenos4);
  Serial.printf("[LOG] sequencia maxima de ciclos normais=%lu | max intervalos seguidos=%lu\n",
      (unsigned long)maxNormal,(unsigned long)maxSeqInt);
  Serial.printf("[LOG] amostras na meta 8-9=%lu | com obstrucao >=1=%lu\n",
      (unsigned long)metas,(unsigned long)obs);
  Serial.printf("[LOG] eventos inicio duplo=%lu | perdas comunicacao=%lu | botoes=%lu\n",
      (unsigned long)duplas,(unsigned long)ausentes,(unsigned long)bot);
  Serial.printf("[LOG] ajustes autorizados=%lu | mudancas PWM=%lu\n",
      (unsigned long)ajustes,(unsigned long)mudancasPWM);
  if(intv>0)
    Serial.printf("[LOG] intervalos=%lu media=%.1f ms min=%lu max=%lu\n",
      (unsigned long)intv,double(somaI)/double(intv),
      (unsigned long)minI,(unsigned long)maxI);
  if(qtd50==50) {
    uint64_t soma50=0;
    uint32_t min50=0xFFFFFFFFUL,max50=0;
    for(uint8_t i=0;i<50;++i){
      soma50+=ultimos50[i];
      if(ultimos50[i]<min50) min50=ultimos50[i];
      if(ultimos50[i]>max50) max50=ultimos50[i];
    }
    const double media50=double(soma50)/50.0;
    double somaQuadrados=0;
    for(uint8_t i=0;i<50;++i){
      const double d=double(ultimos50[i])-media50;
      somaQuadrados+=d*d;
    }
    Serial.printf("[LOG] ULTIMOS 50 intervalos: media=%.1f ms dp=%.1f ms min=%lu max=%lu\n",
      media50,sqrt(somaQuadrados/49.0),
      (unsigned long)min50,(unsigned long)max50);
  }
  Serial.printf("[CHECK] >=30 ciclos NORMAIS consecutivos: %s | >=50 intervalos CONSECUTIVOS: %s\n",
      maxNormal>=30?"SIM":"NAO",maxSeqInt>=50?"SIM":"NAO");
  Serial.println("[NOTA] Resumo nao comprova fotos nem duracao de um ensaio especifico.");
  Serial.println("[NOTA] H exporta o CSV para analise detalhada de 30 ciclos/50 intervalos.");
  Serial.println("=======================================================\n");
}

void exportarArquivoLog(const char* path, bool &cabecalhoFeito) {
  File f=LittleFS.open(path,FILE_READ);
  if(!f) return;
  while(f.available()) {
    String l=f.readStringUntil('\n');
    l.trim();
    if(l.length()==0) continue;
    if(l.startsWith("sessao;")) {
      if(cabecalhoFeito) continue;
      cabecalhoFeito=true;
    }
    Serial.println(l);
    // Exportacao so e permitida em OFF: tempo de transmissao nao perturba malha.
    delay(1);
  }
  f.close();
}
void exportarLog() {
  if(!logDisponivel) { Serial.println("[LOG] INDISPONIVEL"); return; }
  if(estado!=Estado::OFF && estado!=Estado::FALHA_SENSORES) {
    Serial.println("[LOG] Desligue operacao (X ou SELECT 2s) antes de exportar.");
    return;
  }
  Serial.println("======= INICIO CSV TCC (copie somente entre marcadores) =======");
  bool header=false;
  exportarArquivoLog(ARQUIVO_LOG_ANTERIOR,header);
  exportarArquivoLog(ARQUIVO_LOG_ATUAL,header);
  Serial.println("======= FIM CSV TCC =======");
}
void apagarLogConfirmado() {
  if(estado!=Estado::OFF) { Serial.println("[LOG] So apague em OFF"); return; }
  if(!limparLogArmado || millis()-instanteArmadoLog>10000UL) {
    Serial.println("[LOG] Cancelado. Envie Z e depois Y em ate 10 segundos.");
    return;
  }
  limparLogArmado=false;
  if(!logDisponivel) return;
  LittleFS.remove(ARQUIVO_LOG_ANTERIOR);
  LittleFS.remove(ARQUIVO_LOG_ATUAL);
  File f=LittleFS.open(ARQUIVO_LOG_ATUAL,FILE_WRITE);
  if(f) { f.println(CABECALHO_LOG); f.close(); }
  Serial.println("[LOG] Historico apagado por solicitacao explicita do operador");
  registrarEvento("LIMPEZA_LOG");
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
  registrarEvento("PWM_ALTERADO");
}
void desligarTudo() {
  pwmSet(0.0f);
  estado = Estado::OFF;
  limparTemporizadorObstrucao();
  // OFF deixa PWM=0 mas NAO significa que os sensores estao ausentes.
  // Uma nova leitura com LEDs apagados atualiza status e valores a ~0.
  nValidos = 0;
  eMedia = NAN;
  for (auto &p : pontos) {
    p.obstruido = false;
    p.leituraFisica = false;
    p.E = NAN;
  }
  ultimaLeitura = 0;
  consecutivosOK = 0;
  ciclosFora = 0;
  telaSuja = true;
  pagina = 0;
  Serial.println("[SISTEMA] OFF / PWM=0");
  registrarEvento("OPERACAO_DESLIGADA");
}
void iniciarOperacao() {
  if (!pwmPronto || !muxAddr || ativosIniciais() < SENSORES_MIN ||
      nValidos < SENSORES_MIN) {
    estado = Estado::FALHA_SENSORES;
    pwmSet(0.0f);
    pagina = 0;
    telaSuja = true;
    Serial.println("[BLOQUEIO] Necessarias >=4 leituras validas; execute D no Serial");
    registrarEvento("INICIO_BLOQUEADO");
    return;
  }
  for (auto &p : pontos) {
    p.obstruido = false;
  }
  consecutivosOK = 0;
  ciclosFora = 0;
  passosControle = 0;
  limitePorTentativas = false;
  limparTemporizadorObstrucao();
  estado = Estado::INICIANDO;
  pwmSet(PWM_INICIO);
  ultimaLeitura = 0;
  pagina = 0;
  telaSuja = true;
  Serial.println("[SISTEMA] Partida com PWM=50%; aguardando estabilizacao");
  registrarEvento("OPERACAO_INICIADA");
}

// Os eventos registrados na ISR sao processados fora da interrupcao.
void eventoBotao(uint8_t b, bool longo) {
  Serial.printf("[BOTAO] %s %s, pagina=%u, estado=%u\n",
                b==0?"SELECT":b==1?"VOLTAR":"AVANCAR",
                longo?"LONGO":"CURTO", pagina, uint8_t(estado));
  registrarEvento(b==0 ? (longo?"BT_SEL_2S":"BT_SEL_CURTO") :
                  b==1 ? (longo?"BT_VOL_LONGO":"BT_VOL_CURTO") :
                         (longo?"BT_AVA_LONGO":"BT_AVA_CURTO"));

  // POWER exclusivamente no SELECT longo (>= 2 s), em qualquer pagina.
  if (b == 0 && longo) {
    if (estado == Estado::OFF || estado == Estado::FALHA_SENSORES)
      iniciarOperacao();
    else
      desligarTudo();
    telaSuja = true;
    return;
  }

  // Mantem a opcao de CANCELAR o controle sem desligar a operacao:
  // segurar VOLTAR >= 1 s durante o ajuste.
  if (b == 1 && longo && estado == Estado::CONTROLANDO) {
    Serial.println("[CONTROLE] Cancelado pelo operador; PWM mantido");
    registrarEvento("AJUSTE_CANCELADO");
    estado = Estado::MONITORANDO;
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
    // SELECT curto: todas as confirmacoes anteriores sao preservadas.
    if (estado == Estado::CONFIRMAR) {
      if (nValidos >= SENSORES_MIN && isfinite(eMedia) &&
          totalSaturados() == 0 && !objetivoAtingido()) {
        estado = Estado::CONTROLANDO;
        consecutivosOK = 0;
        passosControle = 0;
        limitePorTentativas = false;
        Serial.println("[CONTROLE] AUTORIZADO PELO OPERADOR");
        registrarEvento("AJUSTE_AUTORIZADO");
      } else estado = Estado::MONITORANDO;
    } else if (estado == Estado::FORA_FAIXA || estado == Estado::LIMITE_PWM) {
      if (nValidos >= SENSORES_MIN && !objetivoAtingido() &&
          totalSaturados() == 0) estado = Estado::CONFIRMAR;
    }
  }
  telaSuja = true;
}
void processarBotoes() {
  // Detecta SELECT segurado por 2 s MESMO sem soltar o botao.
  // A marca 'longoConsumido' evita novo acionamento no release.
  const uint32_t agoraUs = micros();
  noInterrupts();
  BotaoIRQ &sel = botoesIRQ[0];
  if (sel.pressionado && !sel.longoConsumido &&
      uint32_t(agoraUs - sel.iniciouUs) >= SELECT_POWER_US &&
      digitalRead(PIN_SELECT) == LOW) {
    sel.longoConsumido = true;
    if (sel.longos < 255) ++sel.longos;
  }
  interrupts();

  for (uint8_t i = 0; i < TOTAL_BOTOES; ++i) {
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
  p.referenciaMaisProxima = 0;
  p.divergenciaRel = NAN;
  if (!p.inicializado || !selecionarMux(CH[i]) || !ack(I2C_SENSOR)) {
    // Sem resposta I2C nao e uma medicao 0: e sensor indisponivel.
    // Para o indicador operacional solicitado, ausencia = obstrucao/COM.
    p.motivo = Motivo::AUSENTE;
    p.obstruido = true;
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
    p.obstruido = false;
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
    p.obstruido = false;
    return false;
  }
  // Contagem abaixo do fundo ou mesmo RAW=0 e leitura fisica VALIDA.
  // Uma falha I2C nunca e substituida artificialmente por zero.
  float e = CAL[i].a * fmaxf(0.0f, float(principal)-CAL[i].fundo);
  if (!isfinite(e) || e < 0.0f || e > 150.0f) {
    p.motivo = Motivo::FALHA;
    p.obstruido = false;
    return false;
  }
  p.E = e;
  p.leituraFisica = true;
  // Toda aquisicao fisica bem-sucedida limpa o alerta anterior; a
  // comparacao entre sensores sera refeita sobre o conjunto atual.
  p.obstruido = false;
  p.motivo = Motivo::OK;
  return true;
}
void analisarObstrucoes(const bool *obstruidoAntes) {
  // Compara SOMENTE irradiancias estimadas (E), sem perfil espacial.
  // No estado OFF, a obstrucao corresponde EXCLUSIVAMENTE a falta de ACK.
  // Na operacao, nao compara 90% se a media preliminar de E <=0.50.
  // Acima de 75% de PWM as estimativas continuam extrapoladas (THOR).
  // A ausencia de comunicacao e detectada independentemente da irradiancia.
  for (uint8_t i = 0; i < N; ++i) {
    Ponto &p = pontos[i];
    p.referenciaMaisProxima = 0;
    p.divergenciaRel = NAN;
    if (p.motivo == Motivo::AUSENTE) p.obstruido = true;
    else if (p.motivo != Motivo::OK) p.obstruido = false;
    else p.obstruido = false; // recomputar a partir desta leitura
  }

  // Media PRELIMINAR, antes de excluir sensores por discrepancia,
  // evita usar uma media final NA/NaN para decidir se o ambiente esta escuro.
  float somaPreliminar = 0.0f;
  uint8_t totalPreliminar = 0;
  for (uint8_t i=0; i<N; ++i) {
    const Ponto &p = pontos[i];
    if (p.leituraFisica && p.motivo == Motivo::OK && isfinite(p.E)) {
      somaPreliminar += p.E;
      ++totalPreliminar;
    }
  }
  const float mediaPreliminar = totalPreliminar ?
      somaPreliminar / float(totalPreliminar) : 0.0f;
  const bool compararPercentual = estado != Estado::OFF && pwm > 0.01f &&
      totalPreliminar >= 2 && mediaPreliminar > PISO_MEDIA_ESCURA_UW;

  // Passo 1: buscar o OUTRO sensor com irradiancia mais proxima.
  // Calcula todas as discrepancias antes de aplicar as exclusoes.
  bool discrepante[N] = {false, false, false, false, false};
  if (compararPercentual) for (uint8_t i = 0; i < N; ++i) {
    Ponto &p = pontos[i];
    if (!p.leituraFisica || p.motivo != Motivo::OK || !isfinite(p.E)) continue;
    int8_t proximo = -1;
    float menorDistancia = INFINITY;
    for (uint8_t j = 0; j < N; ++j) {
      if (i == j || !pontos[j].leituraFisica ||
          pontos[j].motivo != Motivo::OK || !isfinite(pontos[j].E)) continue;
      float distancia = fabsf(p.E - pontos[j].E);
      if (distancia < menorDistancia) {
        menorDistancia = distancia;
        proximo = int8_t(j);
      }
    }
    if (proximo < 0) continue; // sem outro sensor valido para comparar
    const float referencia = pontos[proximo].E;
    p.referenciaMaisProxima = uint8_t(proximo + 1);
    const float denominador = fmaxf(p.E, referencia);
    if (denominador < PISO_COMPARACAO_UW) {
      p.divergenciaRel = 0.0f;
      continue; // todos muito proximos do escuro: nao excluir zeros
    }
    p.divergenciaRel = menorDistancia / denominador;
    discrepante[i] = p.divergenciaRel >= LIMIAR_DIVERGENCIA_REL;
  }

  // Passo 2: atualizar estados, sem confundir desconexao com luz zero.
  for (uint8_t i = 0; i < N; ++i) {
    Ponto &p = pontos[i];
    if (discrepante[i]) {
      p.obstruido = true;
      p.motivo = Motivo::OBSTRUIDO;
    }
    if (!obstruidoAntes[i] && p.obstruido) {
      if (p.motivo == Motivo::AUSENTE) {
        Serial.printf("[OBSTRUCAO] S%u sem comunicacao/identificacao (SEM ACK)\n", i+1);
      } else {
        Serial.printf("[OBSTRUCAO] S%u vs S%u: E=%.3f; referencia=%.3f; diferenca=%.1f%% (>=90%%)\n",
                      i+1, p.referenciaMaisProxima, p.E,
                      pontos[p.referenciaMaisProxima-1].E, 100.0f*p.divergenciaRel);
      }
    } else if (obstruidoAntes[i] && !p.obstruido) {
      Serial.printf("[OBSTRUCAO] S%u liberado; medicao novamente valida\n", i+1);
    }
  }
}
void adquirirTodos() {
  // Com PWM=0 (inclusive OFF) tambem fazemos leituras! Um valor 0.00
  // e um dado valido; status AUSENTE/FALHA fica reservado a erro real.
  bool obstruidoAntes[N];
  Motivo motivoAntes[N];
  for (uint8_t i=0; i<N; ++i) {
    obstruidoAntes[i] = pontos[i].obstruido;
    motivoAntes[i] = pontos[i].motivo;
  }
  for (uint8_t i=0; i<N; ++i) lerSensor(i);
  deselecionarMux();
  analisarObstrucoes(obstruidoAntes);
  atualizarTemporizadorObstrucao();
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
  // Transicoes individuais: permite comprovar desconexao e recuperacao.
  for (uint8_t i=0; i<N; ++i) {
    if (motivoAntes[i] != pontos[i].motivo) {
      char ev[28];
      const char *estadoSensor = pontos[i].motivo==Motivo::AUSENTE ? "AUSENTE" :
                                 pontos[i].motivo==Motivo::OBSTRUIDO ? "OBSTR90" :
                                 pontos[i].motivo==Motivo::OK ? "OK" :
                                 pontos[i].motivo==Motivo::SATURADO ? "SAT" : "FALHA";
      snprintf(ev,sizeof(ev),"S%u_%s",i+1,estadoSensor);
      registrarEvento(ev);
    }
  }
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
  if (obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada)
    Serial.printf("[OBSTRUCAO] %u/5 simultaneos; %.1f/10.0s (PWM mantido, sem atuar)\n",
      quantidadeObstruidos, float(duracaoObstrucaoDuplaMs)/1000.0f);
  telaSuja = true;
}

void atualizarControle() {
  if (estado == Estado::OFF || estado == Estado::FALHA_SENSORES) return;
  if (!isfinite(eMedia) || nValidos < SENSORES_MIN) {
    // A tolerancia de 10s vale APENAS se TODAS as perdas resultarem de
    // duas ou mais obstrucoes simultaneas, sem falhas/saturacao adicionais.
    // A media fica NA: NAO substituir por zero, NAO atuar na malha de controle.
    const bool somenteObstrucoes = obstrucaoDuplaAtiva &&
        (nValidos + quantidadeObstruidos == N);
    if (somenteObstrucoes && !obstrucaoDuplaConfirmada) {
      if (estado == Estado::CONTROLANDO || estado == Estado::CONFIRMAR) {
        Serial.println("[CONTROLE] Ajuste cancelado durante espera de obstrucao dupla; nova autorizacao sera necessaria");
        estado = Estado::MONITORANDO;
      }
      // Mantem o PWM atual; aguarda >=10s continuos antes da parada.
      // Continua lendo os sensores, processando botoes e mostrando alertas.
      telaSuja = true;
      return;
    }
    // Se persistir >=10s ou houver outra falha nao coberta, parar os LEDs.
    Serial.printf("[AVISO] Insuficientes: %u/5. %s; leitura continuara.\n",
      nValidos, obstrucaoDuplaConfirmada?"Obstrucao >=10s":"Outra falha de medicao");
    registrarEvento(obstrucaoDuplaConfirmada?"PARADA_OBSTRUCAO_10S":"PARADA_FALHA_MEDICAO");
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
      registrarEvento("CONTROLE_SATURACAO");
      return;
    }
    // O controle so termina com 3 aquisicoes na META 8,0 a 9,0.
    if (objetivoAtingido()) {
      if (++consecutivosOK >= CICLOS_OK) {
        estado = Estado::CONCLUIDO;
        avisoConclusao = millis();
        registrarEvento("META_CONTROLE_ATINGIDA");
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
      registrarEvento("CONTROLE_LIMITE_PASSOS");
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
      registrarEvento("CONTROLE_LIMITE_PWM");
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
// INTERFACE (PERSISTENTE): ELEMENTOS FIXOS + SOBRESCRITA DOS CAMPOS VARIAVEIS
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
  CAM_LOG=0, CAM_SELO, CAM_RODAPE1, CAM_RODAPE2, CAM_META, CAM_IRRAD,
  CAM_PWM, CAM_VALIDOS,
  CAM_SENSOR_MODELO_0, CAM_SENSOR_MODELO_1, CAM_SENSOR_MODELO_2,
  CAM_SENSOR_MODELO_3, CAM_SENSOR_MODELO_4,
  CAM_SENSOR_E_0, CAM_SENSOR_E_1, CAM_SENSOR_E_2, CAM_SENSOR_E_3, CAM_SENSOR_E_4,
  CAM_SENSOR_STATUS_0, CAM_SENSOR_STATUS_1, CAM_SENSOR_STATUS_2,
  CAM_SENSOR_STATUS_3, CAM_SENSOR_STATUS_4,
  CAM_ESTADO, CAM_PASSOS, CAM_SELO_SUBTITULO, TOTAL_CAMPOS
};
struct CacheCampo {
  String texto;
  uint16_t tinta=0, fundo=0;
  bool valido=false;
};
CacheCampo cacheTela[TOTAL_CAMPOS];
uint8_t paginaDesenhada = 255; // 255 = nenhuma pagina desenhada
uint16_t corSeloAnterior = 0xFFFF;
bool seloGrandeAnterior = false;
int larguraBarraAnterior = -1;
uint16_t corBarraAnterior = 0xFFFF;

void invalidarCampos() {
  for(uint8_t i=0;i<TOTAL_CAMPOS;++i) cacheTela[i].valido=false;
  corSeloAnterior=0xFFFF;
  seloGrandeAnterior=false;
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
  // Indicador LOG e dinamico: vermelho/amarelo em caso de falha de gravacao.
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
    // Modelo e atualizado como campo dinamico, pois pode reconectar apos boot.
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
bool exibirAvisoGrande() {
  // A faixa amarela GRANDE aparece para irradiancia fora do objetivo,
  // estado de advertencia e no periodo de obstrucao dupla simultanea.
  if (estado == Estado::OFF) return false;
  if (obstrucaoDuplaAtiva || estado == Estado::FORA_FAIXA ||
      estado == Estado::LIMITE_PWM || estado == Estado::SATURACAO ||
      estado == Estado::FALHA_SENSORES) return true;
  return estado == Estado::MONITORANDO && isfinite(eMedia) && !objetivoAtingido();
}

const char *subtituloAviso() {
  if (obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada) return "2+ OBSTRUIDOS";
  if (estado == Estado::FALHA_SENSORES) return "FALHA SENSORES";
  if (estado == Estado::SATURACAO) return "SATURACAO";
  if (estado == Estado::LIMITE_PWM) return "LIMITE PWM";
  if (estado == Estado::FORA_FAIXA ||
      (isfinite(eMedia) && (eMedia < E_OBJ_INF || eMedia > E_OBJ_SUP)))
    return "FORA DO ALVO";
  return "VERIFICAR";
}

void atualizarSelo() {
  const bool okLog=logDisponivel && !avisoLogFalha;
  sobrescrever(CAM_LOG,145,14,okLog?String("LOG OK"):String("LOG ERR"),
              okLog?VERDE:AMARELO,FUNDO,1,7);
  const bool grande = exibirAvisoGrande();
  const uint16_t cor = grande ? AMARELO : corEstado();
  if (cor != corSeloAnterior || grande != seloGrandeAnterior) {
    // Apaga APENAS a regiao do aviso ao trocar tamanho/cor/estado.
    // Nao redesenha o fundo, os cartoes nem o restante do cabecalho.
    tft.fillRect(191,3,120,33,FUNDO);
    if (grande) tft.fillRoundRect(192,3,119,33,8,AMARELO);
    else tft.fillRoundRect(220,6,91,21,7,cor);
    corSeloAnterior=cor;
    seloGrandeAnterior=grande;
    cacheTela[CAM_SELO].valido=false;
    cacheTela[CAM_SELO_SUBTITULO].valido=false;
  }
  if (grande) {
    sobrescrever(CAM_SELO,208,6,"ATENCAO",FUNDO,AMARELO,2,7);
    sobrescrever(CAM_SELO_SUBTITULO,204,25,String(subtituloAviso()),FUNDO,AMARELO,1,16);
  } else {
    sobrescrever(CAM_SELO,227,13,String(textoSelo()),FUNDO,cor,1,12);
  }
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
  if(estado==Estado::OFF) atualizarRodape("SELEC 2s: ligar | LEDs desativados");
  else if(obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada)
    atualizarRodape(String("OBSTRUCAO ")+String(quantidadeObstruidos)+
      "/5: "+String(duracaoObstrucaoDuplaMs/1000.0f,1)+"/10s | PWM mantido");
  else if(millis()-alterouPWM < ESTABILIZACAO_PWM_MS)
    atualizarRodape("Estabilizando fonte | leitura anterior");
  else if(estado==Estado::CONFIRMAR) atualizarRodape("SELEC: confirmar  VOLTAR: cancelar");
  else if(estado==Estado::CONTROLANDO) atualizarRodape("SELEC 2s: OFF | VOLTAR 1s: cancelar");
  else if(estado==Estado::FORA_FAIXA) atualizarRodape("SELEC: solicitar ajuste | setas: paginas");
  else if(estado==Estado::LIMITE_PWM)
    atualizarRodape(limitePorTentativas ? "Max passos | SELEC: tentar novamente":
                                         "Limite 100% ou 10% | meta nao alcancada");
  else if(estado==Estado::SATURACAO) atualizarRodape("SATURACAO: pare e confira THOR");
  else if(estado==Estado::FALHA_SENSORES) atualizarRodape("Falha: desligue/verifique sensores");
  else atualizarRodape("Setas: paginas | SELEC 2s: desligar");
}
void atualizarSensores() {
  for(uint8_t i=0;i<N;++i) {
    const int y=67+i*27;
    sobrescrever(static_cast<uint8_t>(CAM_SENSOR_MODELO_0+i),46,y+8,
      String(tipoStr(pontos[i].tipo)),CINZA,CARTAO,1,4);
    String valor=isfinite(pontos[i].E)?String(pontos[i].E,2):String("--.--");
    if(isfinite(pontos[i].E) && pontos[i].E>99.99f) valor=">99.9";
    sobrescrever(static_cast<CampoTela>(CAM_SENSOR_E_0+i),100,y+7,
      valor,isfinite(pontos[i].E)?BRANCO:AMARELO,CARTAO,2,7);
    // "OBSTR/COM" = sensor sem comunicacao (nao se sabe se o cabo
    // esta fisicamente desconectado); "OBSTR/90%" = disparidade de E.
    const char *status = pontos[i].motivo==Motivo::AUSENTE ? "OBSTR/COM" :
                         pontos[i].motivo==Motivo::OBSTRUIDO ? "OBSTR/90%" :
                         motivoStr(pontos[i].motivo);
    sobrescrever(static_cast<uint8_t>(CAM_SENSOR_STATUS_0+i),237,y+8,
      String(status),
      pontos[i].motivo==Motivo::OK?VERDE:AMARELO,CARTAO,1,11);
  }
  atualizarRodape("S1..S5 | E estimada em uW/cm2/nm");
}
void atualizarControleTela() {
  const String statusControle = (obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada) ?
      String("OBSTR ")+String(duracaoObstrucaoDuplaMs/1000.0f,1)+"/10s" :
      String(nomeEstado());
  sobrescrever(CAM_ESTADO,20,95,statusControle,
      obstrucaoDuplaAtiva?AMARELO:corEstado(),CARTAO,2,22);
  sobrescrever(CAM_PASSOS,75,177,
               String(passosControle)+" / "+String(MAX_PASSOS),CINZA,CARTAO,1,12);
  if(obstrucaoDuplaAtiva && !obstrucaoDuplaConfirmada)
    atualizarRodape("2+ OBSTRUCOES: controle pausado, aguarde 10s");
  else if(estado==Estado::CONFIRMAR) atualizarRodape("CONFIRME: SELEC | CANCELE: VOLTAR");
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
  Serial.printf("[DIAG] obstruidos=%u dupla=%u duracao=%.1fs confirmada=%u\n",
                quantidadeObstruidos,obstrucaoDuplaAtiva?1U:0U,
                float(duracaoObstrucaoDuplaMs)/1000.0f,obstrucaoDuplaConfirmada?1U:0U);
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
    if(p.motivo == Motivo::AUSENTE)
      Serial.printf("[DIAG] S%u OBSTR/COM: sem comunicacao ou sem identificacao, NAO equivale a E=0\n",i+1);
    else if(p.referenciaMaisProxima > 0 && isfinite(p.divergenciaRel))
      Serial.printf("[DIAG] S%u vizinho=S%u divergencia=%.1f%% obstruido=%u\n",
                    i+1,p.referenciaMaisProxima,100.0f*p.divergenciaRel,p.obstruido?1U:0U);
  }
}
void comandoSerial() {
  while (Serial.available()) {
    char c = toupper((unsigned char)Serial.read());
    if(c=='?') Serial.println("G=resumo FLASH H=exportar CSV Z+Y=apagar log; SELECT 2s=on/off; X=OFF I=iniciar P=prox V=voltar D=agora B=botoes R=reset T=I2C ?=ajuda");
    else if(c=='X') desligarTudo();
    else if(c=='I' && (estado==Estado::OFF || estado==Estado::FALHA_SENSORES)) iniciarOperacao();
    else if(c=='P') { pagina=(pagina+1)%3; telaSuja=true; }
    else if(c=='V') { pagina=(pagina+2)%3; telaSuja=true; }
    else if(c=='D') imprimirDiagnostico();
    else if(c=='G') resumirLog();
    else if(c=='H') exportarLog();
    else if(c=='Z') {
      if (estado==Estado::OFF && logDisponivel) {
        limparLogArmado=true;
        instanteArmadoLog=millis();
        Serial.println("[LOG] ATENCAO: para apagar historico, envie Y em 10 segundos.");
      } else Serial.println("[LOG] So e possivel preparar exclusao em OFF com LittleFS ativo");
    }
    else if(c=='Y') apagarLogConfirmado();
    else if(c=='T') {
      if(estado==Estado::OFF || estado==Estado::FALHA_SENSORES) diagnosticarI2C();
      else Serial.println("[T] Desligue os LEDs com X ou SELECT 2s antes do teste I2C");
    }
    else if(c=='B') {
      Serial.printf("[BOTOES] SEL GPIO18=%d VOL GPIO21=%d AVA GPIO22=%d (0=pressionado; SELECT 2s=POWER)\n",
        digitalRead(PIN_SELECT),digitalRead(PIN_BACK),digitalRead(PIN_NEXT));
    } else if(c=='R') {
      if(estado==Estado::OFF || estado==Estado::FALHA_SENSORES) {
        pwmSet(0);
        // RESET inicial completo (5 s), mesmo que todos sensores sumiram.
        resetMuxIsolado(true);
        if(encontrarMux()) inicializarSensores();
        diagnosticarI2C();
        ultimaLeitura=0;
        telaSuja=true;
      } else Serial.println("[R] Desligue com SELECT 2s ou X antes de reiniciar sensores");
    }
  }
}
void setup() {
  Serial.begin(115200);
  delay(120);
  // NUNCA montar/escrever flash antes de garantir saida PWM em zero.
  pinMode(PIN_PWM,OUTPUT);
  digitalWrite(PIN_PWM,LOW);
  pwmPronto = ledcAttach(PIN_PWM,PWM_FREQ,PWM_BITS);
  if (pwmPronto) ledcWrite(PIN_PWM,0);
  iniciarLogPersistente();
  registrarEvento("BOOT"); // ESP32 Arduino core 3.x
  pwmSet(0);
  pinMode(PIN_SELECT,INPUT_PULLUP);
  pinMode(PIN_BACK,INPUT_PULLUP);
  pinMode(PIN_NEXT,INPUT_PULLUP);
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
  resetMuxIsolado(true);
  if(encontrarMux()) inicializarSensores();
  diagnosticarI2C();
  desligarTudo();
  // Botoes ativos em LOW (fecham no GND do ESP32).
  // Interrupcoes preservam cliques durante leitura I2C demorada.
  for(uint8_t i=0;i<TOTAL_BOTOES;++i) {
    botoesIRQ[i].pressionado = (digitalRead(PIN_BOTOES[i])==LOW);
    botoesIRQ[i].iniciouUs = micros();
    botoesIRQ[i].ultimaSolturaUs = micros()-60000UL;
  }
  attachInterrupt(digitalPinToInterrupt(PIN_SELECT),irqSelecionar,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BACK),irqVoltar,CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_NEXT),irqAvancar,CHANGE);
  // Primeira leitura com LEDs apagados: 0.00 e um valor valido.
  if(muxAddr) adquirirTodos();
  ultimaLeitura = millis();
  Serial.println("[BOOT] Firmware TCC v1.2.7 / LOG FLASH / SELECT 2s=POWER / CAL_30 / PWM max 100 / 5 sensores");
  Serial.println("[BOOT] G=resumo salvo | H=CSV salvo | Z depois Y=apagar historico (OFF)");
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
    registrarAmostra();
  }
  // Se a captura I2C demorou, trata os cliques que a interrupcao guardou.
  processarBotoes();
  mostrarTela();
  delay(5);
}
