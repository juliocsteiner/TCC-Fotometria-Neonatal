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

// ---------------- PINAGEM EXISTENTE DA PCB ----------------
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
bool lerUmAS7341(uint8_t indice) {
  // Leitura em duas etapas do SMUX, com limite de tempo em vez de
  // readAllChannels() bloqueante (que esperaria para sempre em falha).
  Adafruit_AS7341 &sensor = sensor7341[indice];
  if (!sensor.startReading()) return false;
  uint32_t inicio = millis();
  bool concluido = false;
  while (millis() - inicio < 1200) {
    // Interrompe a espera quando o operador solicita parada.
    if (eventoSelecionar()==3) return false;
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


// ===============================================================
// CALIBRACAO RAPIDA - REFERENCIAS DO THOR PRE-CADASTRADAS
// Calibracao por substituicao: cada modulo ocupa A MESMA POSICAO
// da sonda THOR usada no S1. Os pontos A1..V3 nao correspondem a
// posicoes espaciais dos cinco sensores na montagem final.
// IMPORTANTE: conferir estabilidade luminosa e geometria da bancada
// antes de reutilizar a referencia do THOR para S2..S5.
// ===============================================================
Preferences banco;
bool bancoOK = false, pwmOK = false;
constexpr uint8_t N = 10;
constexpr uint8_t NUM_BLOCOS = 7;  // Fundo + seis pontos com LED aceso
const char *blocos[NUM_BLOCOS] = {"Fundo", "A1", "A2", "A3", "V1", "V2", "V3"};
const uint8_t pwmPontos[NUM_BLOCOS] = {0, 10, 30, 50, 60, 75, 68};
// Valores do THOR em decimos de uW/cm2/nm: 1.7 -> 17.
// Referencias medidas no S1 e informadas pelo operador.
const uint16_t thorDecimos[NUM_BLOCOS] = {0, 17, 46, 74, 83, 103, 92};

// IMPORTAR_S1: popula APENAS blocos S1 ainda nao existentes no NVS.
// Nunca substitui uma coleta S1 ja salva. Pode ser desligado apos a importacao.
constexpr bool IMPORTAR_S1 = true;

// Leituras originais S1/CH2/AS7343, N=10, sem arredondamentos.
// A ordem corresponde a {Fundo,A1,A2,A3,V1,V2,V3}.
const uint16_t s1Principal[NUM_BLOCOS][N] = {
  {223,223,223,223,224,223,223,223,223,223},
  {2351,2347,2350,2350,2277,2350,2350,2321,2351,2350},
  {5909,5907,5850,5908,5908,5811,5907,5909,5851,5854},
  {9356,9343,9453,9357,9358,9453,9377,9355,9454,9453},
  {11153,11151,11251,11151,11151,11243,11222,11153,11160,11250},
  {13787,13787,13812,13818,13786,13785,13884,13786,13785,13831},
  {12530,12520,12629,12529,12529,12609,12530,12529,12607,12563}
};
const uint16_t s1Auxiliar[NUM_BLOCOS][N] = {
  {73,73,73,74,74,74,74,73,73,73},
  {2250,2251,2248,2248,2209,2249,2249,2169,2247,2248},
  {5868,5871,5773,5866,5868,5771,5867,5865,5771,5845},
  {9296,9312,9395,9295,9332,9394,9293,9290,9393,9390},
  {11088,11074,11188,11088,11086,11188,11119,11085,11128,11184},
  {13698,13697,13761,13698,13697,13688,13794,13697,13695,13783},
  {12432,12440,12532,12430,12427,12531,12431,12423,12530,12430}
};

// Tela enxuta: um unico fluxo para TODOS os sensores.
// MENU -> POSICIONAR -> REVISAR -> [grava NVS e avanca bloco/sensor].
enum Tela { MENU, POSICIONAR, REVISAR, RESULT };
Tela tela = MENU;
int mod = 1;    // Inicia em S2; os dados do S1 ja foram fornecidos/importados.
int bloco = 0;  // Comeca pela leitura de fundo.
int duty = 0;
String aviso;

// Mesmo layout e namespace do codigo original: nao perde calibracoes ja salvas.
struct Registro {
  uint32_t versao;
  uint8_t tipo, pwm;
  uint16_t antes, depois, principal[10], auxiliar[10];
};

// Prototipos explicitos APOS a declaracao da estrutura Registro.
// Evitam que o pre-processador da Arduino IDE gere assinaturas
// com Registro antes de o tipo ser conhecido pelo compilador.
bool lerSalvo(int m, int p, Registro &r);
bool salvar(int m, int p, const Registro &r);
float media(const Registro &r);
float mediaAuxiliar(const Registro &r);
float referencia(const Registro &r);

Registro atual = {};

String chave(int m, int p) { return String("s") + m + "p" + p; }
bool lerSalvo(int m, int p, Registro &r) {
  return bancoOK && banco.getBytesLength(chave(m,p).c_str()) == sizeof(r) &&
         banco.getBytes(chave(m,p).c_str(), &r, sizeof(r)) == sizeof(r) &&
         r.versao == 1;
}
bool salvar(int m, int p, const Registro &r) {
  if (!bancoOK) return false;
  const String id = chave(m,p);
  Registro copia = {};
  return banco.putBytes(id.c_str(), &r, sizeof(r)) == sizeof(r) &&
         lerSalvo(m,p,copia) && memcmp(&copia,&r,sizeof(r)) == 0;
}

void importarDadosS1() {
  if (!bancoOK || !IMPORTAR_S1) return;
  int importados = 0;
  Registro r = {};
  for (uint8_t p = 0; p < NUM_BLOCOS; ++p) {
    if (lerSalvo(0,p,r)) continue;  // NUNCA sobrescrever medicao existente
    r = {};
    r.versao = 1;
    r.tipo = (uint8_t)TipoSensor::AS7343;
    r.pwm = pwmPontos[p];
    r.antes = thorDecimos[p];
    r.depois = thorDecimos[p];
    for (uint8_t i = 0; i < N; ++i) {
      r.principal[i] = s1Principal[p][i];
      r.auxiliar[i] = s1Auxiliar[p][i];
    }
    if (salvar(0,p,r)) {
      ++importados;
      Serial.printf("[IMPORTADO] S1 %s: %u amostras, THOR %.1f, PWM %u%%\n",
                    blocos[p],N,thorDecimos[p]/10.0f,pwmPontos[p]);
    } else {
      Serial.printf("[ERRO NVS] Nao foi possivel importar S1 %s\n",blocos[p]);
    }
  }
  Serial.printf("[S1] %d blocos importados; registros existentes preservados.\n",importados);
}

float media(const Registro &r) {
  uint32_t soma = 0;
  for (uint8_t i=0;i<N;++i) soma += r.principal[i];
  return soma / float(N);
}
float mediaAuxiliar(const Registro &r) {
  uint32_t soma = 0;
  for (uint8_t i=0;i<N;++i) soma += r.auxiliar[i];
  return soma / float(N);
}
float referencia(const Registro &r) { return (r.antes+r.depois)/20.0f; }

// A1,A2,A3 = ajustamento da reta; V1,V2,V3 = verificacao independente.
bool reta(int m, float &a, float &b, float &fundo) {
  Registro r;
  if (!lerSalvo(m,0,r)) return false;
  const uint8_t tipo = r.tipo;
  fundo = media(r);
  float x[3],y[3],mx=0,my=0;
  for (int j=0;j<3;j++) {
    if (!lerSalvo(m,j+1,r) || r.tipo!=tipo) return false;
    x[j]=media(r)-fundo;
    y[j]=referencia(r);
    mx+=x[j]/3.0f;
    my+=y[j]/3.0f;
  }
  if (!(x[0]<x[1] && x[1]<x[2] && y[0]<y[1] && y[1]<y[2])) return false;
  float xx=0,xy=0;
  for (int j=0;j<3;j++) { xx+=sq(x[j]-mx); xy+=(x[j]-mx)*(y[j]-my); }
  if (xx <= 0) return false;
  a=xy/xx;
  b=my-a*mx;
  return true;
}

void saida(int valor) {
  duty=constrain(valor,0,100);
  if (pwmOK) ledcWrite(PWM_PIN,(duty*255)/100);
}
void linha(const String &s) { tft.println(s); }
void pintar() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0,0);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  linha(String("S")+(mod+1)+" CH"+canaisSensor[mod]+" "+nomeTipo(tipoSensor[mod]));
  linha(String(blocos[bloco])+" PWM "+pwmPontos[bloco]+"%");
  linha(String("THOR fixo: ")+String(thorDecimos[bloco]/10.0f,1));
  Registro r;
  if (tela==MENU) {
    linha(lerSalvo(mod,bloco,r)?"Memoria: GRAVADO":"Memoria: LIVRE");
    linha("VOLTAR: sensor");
    linha("AVANCAR: ponto");
    linha("SELECT: iniciar");
    linha("SELECT 1s: resultado");
    linha("LED desligado");
  } else if (tela==POSICIONAR) {
    tft.setTextColor(ILI9341_CYAN);
    linha("LED ESTABILIZANDO");
    tft.setTextColor(ILI9341_WHITE);
    linha("Mesmo ponto do THOR");
    linha("Aguarde estabilizar");
    linha("SELECT: ler 10 RAW");
    linha("SELECT 1s: cancela");
  } else if (tela==REVISAR) {
    linha(String("RAW princ ")+String(media(atual),1));
    linha(String("RAW aux   ")+String(mediaAuxiliar(atual),1));
    linha(String("N=10 THOR ")+String(referencia(atual),1));
    linha("SELECT: grava NVS");
    linha("1s: descarta");
    linha("LED desligado");
  } else { // RESULT
    float a,b,d;
    if (reta(mod,a,b,d)) {
      linha(String("a = ")+String(a,8));
      linha(String("b = ")+String(b,3));
      linha(String("fundo = ")+String(d,1));
      for (int p=4;p<NUM_BLOCOS;p++) {
        if (lerSalvo(mod,p,r))
          linha(String(blocos[p])+" erro "+String(a*(media(r)-d)+b-referencia(r),2));
      }
    } else linha("Fundo + A1<A2<A3?");
    linha("SELECT: voltar");
  }
  if (aviso.length()) {
    tft.setTextColor(ILI9341_YELLOW);
    linha(aviso);
  }
}

void cancelar(const String &motivo) {
  saida(0); tela=MENU; aviso=motivo; pintar();
}
bool pausaComParada(uint32_t ms) {
  const uint32_t inicio=millis();
  while (millis()-inicio < ms) {
    if (eventoSelecionar()==3) return false;
    delay(5);
  }
  return true;
}
bool capturar() {
  if (!selecionarCanal(canaisSensor[mod])) return false;
  bool ok=true;
  // Amostra adicional descartada para estabilizacao do canal SMUX.
  for (int j=-1;j<N && ok;j++) {
    if (!pausaComParada(300)) { ok=false; break; }
    ok = tipoSensor[mod]==TipoSensor::AS7341 ? lerUmAS7341(mod) : lerUmAS7343(mod);
    if (ok && j>=0) {
      atual.principal[j]=leituraB[mod];
      atual.auxiliar[j]=leituraA[mod];
      if (leituraB[mod]>=17990 || leituraA[mod]>=17990) {
        Serial.printf("[SATURACAO] S%d %s amostra %d: %u/%u\n",mod+1,blocos[bloco],j+1,leituraB[mod],leituraA[mod]);
        ok=false;
      } else {
        Serial.printf("[RAW] S%d CH%d %-5s PWM=%u THOR=%.1f n=%d principal=%u aux=%u\n",
                      mod+1,canaisSensor[mod],blocos[bloco],
                      atual.pwm,referencia(atual),j+1,
                      atual.principal[j],atual.auxiliar[j]);
      }
      tft.fillRect(0,208,320,32,ILI9341_BLACK);
      tft.setCursor(0,208);
      tft.setTextColor(ILI9341_YELLOW);
      tft.print(String("Amostra ")+(j+1)+"/10");
    }
  }
  desligarCanais();
  return ok;
}

void exportar() {
  Serial.println("sensor;ch;modelo;bloco;pwm;thor_antes;thor_depois;n;raw_principal;raw_aux;origem_referencia");
  Registro r;
  for (int m=0;m<NUM_SENSORES;m++) for (int p=0;p<NUM_BLOCOS;p++) if (lerSalvo(m,p,r)) {
    // Os campos THOR 'antes/depois' do S2..S5 sao referencias reaproveitadas,
    // nao leituras fisicas repetidas antes e depois dessas capturas.
    // Para arquivos legados de outros sensores, a procedencia deve ser conferida.
    const char *origem = m==0 ? "THOR_S1_original" : "THOR_S1_reutilizado_ou_legado";
    for (int j=0;j<N;j++) Serial.printf("S%d;%d;%s;%s;%d;%.1f;%.1f;%d;%u;%u;%s\n",
      m+1,canaisSensor[m],nomeTipo(static_cast<TipoSensor>(r.tipo)),blocos[p],
      r.pwm,r.antes/10.0f,r.depois/10.0f,j+1,r.principal[j],r.auxiliar[j],origem);
  }
  Serial.println("FIM");
}

// Avanca para o ponto seguinte; depois de V3, seleciona sensor seguinte.
void proximaColeta() {
  bloco++;
  if (bloco>=NUM_BLOCOS) {
    bloco=0;
    mod=(mod+1)%NUM_SENSORES;
  }
}

// Cada botao gera um evento por toque; neste firmware o PWM e fixo.
int tecla() {
  int select=eventoSelecionar();
  if (select>=0) return select;
  const int pins[]={BOTAO_VOLTAR,BOTAO_AVANCAR};
  static int lida=-1,estavel=-1;
  static uint32_t mudou=0;
  int k=-1;
  for (int i=0;i<2;i++) if (digitalRead(pins[i])==LOW) k=i+1;
  uint32_t t=millis();
  if (k!=lida) { lida=k; mudou=t; }
  if (t-mudou>=40 && k!=estavel) { estavel=k; return k; }
  return -1;
}

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN,OUTPUT);
  digitalWrite(PWM_PIN,LOW);
  pwmOK=ledcAttach(PWM_PIN,PWM_FREQ,PWM_BITS);
  saida(0);
  for (int p:{int(BOTAO_SELECIONAR),int(BOTAO_VOLTAR),int(BOTAO_AVANCAR)})
    pinMode(p,INPUT_PULLUP);
  bancoOK=banco.begin("thorQ13v1",false); // preservar registros do sketch anterior
  importarDadosS1();
  pinMode(TFT_LED,OUTPUT);
  digitalWrite(TFT_LED,HIGH);
  SPI.begin(TFT_SCK,TFT_MISO,TFT_MOSI,TFT_CS);
  tft.begin(4000000);
  tft.setRotation(1);
  executarReset(true);
  if (procurarMUX()) iniciarSensores();
  if (!bancoOK||!pwmOK) aviso="Falha NVS / PWM";
  Serial.println("\n[CALIBRACAO RAPIDA] Fundo + 6 pontos pre-definidos.\n"
                 "[ATENCAO] Referencias THOR do S1 somente com mesma geometria/estabilidade.\n"
                 "[SERIAL] Digite E no MENU para exportar CSV.\n");
  pintar();
}

void loop() {
  if (Serial.available()) {
    char comando=toupper((unsigned char)Serial.read());
    if (comando=='E' && tela==MENU) exportar();
  }
  const int k=tecla();
  if (k<0) return;
  aviso="";
  if (k==3 && tela!=MENU) { cancelar("Coleta cancelada"); return; }
  if (tela==MENU) {
    if (k==1) mod=(mod+1)%NUM_SENSORES;
    if (k==2) bloco=(bloco+1)%NUM_BLOCOS;
    if (k==3) tela=RESULT;
    if (k==0) {
      Registro prev;
      if (!bancoOK||!pwmOK||!sensorInicializado[mod]) {
        aviso="Verifique NVS/PWM/sensor";
      } else if (bloco>0 && (!lerSalvo(mod,0,prev) ||
                           prev.tipo!=(uint8_t)tipoSensor[mod])) {
        aviso="Faca Fundo deste S";
      } else {
        atual={};
        atual.versao=1;
        atual.tipo=(uint8_t)tipoSensor[mod];
        atual.pwm=pwmPontos[bloco];
        atual.antes=thorDecimos[bloco];
        atual.depois=thorDecimos[bloco];
        if (lerSalvo(mod,bloco,prev)) aviso="Gravar substitui bloco";
        saida(pwmPontos[bloco]);
        tela=POSICIONAR;
        Serial.printf("[PREPARAR] S%d CH%d %s PWM=%u THOR=%.1f (fixo)\n",
                      mod+1,canaisSensor[mod],blocos[bloco],
                      atual.pwm,referencia(atual));
      }
    }
  } else if (tela==POSICIONAR && k==0) {
    linha("Capturando 10 amostras...");
    if (!capturar()) { cancelar("Falha/saturacao/parada"); return; }
    saida(0);
    tela=REVISAR;
  } else if (tela==REVISAR && k==0) {
    const int sensorGravado=mod;
    const int pontoGravado=bloco;
    const bool ok=salvar(mod,bloco,atual);
    if (ok) {
      Serial.printf("[NVS OK] S%d %s media=%.2f aux=%.2f ref=%.1f\n",
                    sensorGravado+1,blocos[pontoGravado],media(atual),
                    mediaAuxiliar(atual),referencia(atual));
      proximaColeta();
    }
    cancelar(ok?String("Salvo S")+(sensorGravado+1)+" "+blocos[pontoGravado]:
                "Falha de gravacao");
    return;
  } else if (tela==RESULT && k==0) {
    tela=MENU;
  }
  pintar();
}
