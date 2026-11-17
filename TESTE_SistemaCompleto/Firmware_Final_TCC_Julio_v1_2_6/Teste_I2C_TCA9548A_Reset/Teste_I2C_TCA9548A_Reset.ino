/*
  Diagnostico independente - TCC - 5 sensores / ISO1540 / TCA9548A / PC817
  Arduino-ESP32 3.x. Somente monitor serial, sem TFT ou bibliotecas AS734x.
  Nao usar o sistema em pacientes. Corrente e LEDs sempre desligados.
  Pressupoe PC817 inversor: GPIO19 HIGH ativa RESET_ISO LOW;
  GPIO19 LOW libera RESET_ISO para o pull-up do dominio isolado.
*/
#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t SDA_P = 17, SCL_P = 16, RESET_PC817 = 19, PWM_LED = 32;
constexpr uint8_t AS_ADDR = 0x39;
const uint8_t CANAIS[5] = {2,3,5,6,7};
uint8_t muxAddr = 0;

bool responde(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
void iniciarI2C() {
  Wire.end();
  delay(10);
  Wire.begin(SDA_P, SCL_P);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
}
void resetTCA(uint32_t ms = 5000) {
  Serial.printf("RESET GPIO19 HIGH por %lu ms - meca RST_ISO contra GND_ISO agora\n", (unsigned long)ms);
  digitalWrite(RESET_PC817, HIGH);
  delay(ms);
  digitalWrite(RESET_PC817, LOW);
  Serial.println("RESET GPIO19 LOW - meca RST_ISO; deve estar ALTO (pull-up isolado)");
  delay(50);
  iniciarI2C();
}
bool encontrarMUX() {
  muxAddr = 0;
  for (uint8_t a = 0x70; a <= 0x77; ++a) {
    if (responde(a)) { muxAddr=a; break; }
  }
  if (muxAddr) Serial.printf("MUX responde em 0x%02X\n", muxAddr);
  else Serial.println("MUX NAO RESPONDE (0x70-0x77). Confira 3V3_ISO, RESET_ISO, ISO1540, SDA/SCL");
  return muxAddr != 0;
}
bool selecionar(uint8_t canal) {
  Wire.beginTransmission(muxAddr);
  Wire.write(uint8_t(1U << canal));
  return Wire.endTransmission() == 0;
}
bool controle(uint8_t &value) {
  if (Wire.requestFrom(muxAddr, uint8_t(1)) != 1) return false;
  value = Wire.read();
  return true;
}
void scan() {
  Serial.println("\n--- DIAGNOSTICO 5 SENSORES (PWM SEMPRE 0) ---");
  if (!encontrarMUX()) return;
  for (uint8_t i=0;i<5;++i) {
    uint8_t v=0;
    bool gravou=selecionar(CANAIS[i]);
    bool leu=gravou && controle(v);
    bool corresponde=leu && v == uint8_t(1U << CANAIS[i]);
    bool asACK=corresponde && responde(AS_ADDR);
    Serial.printf("S%u canal %u: MUX escrita=%s leitura=%s bits=0x%02X sensor 0x39=%s\n",
      i+1,CANAIS[i], gravou?"OK":"ERRO", corresponde?"OK":"ERRO",
      v, asACK?"ACK":"SEM ACK");
  }
  Wire.beginTransmission(muxAddr);
  Wire.write(uint8_t(0));
  Wire.endTransmission();
  Serial.println("--- FIM ---");
}
void setup() {
  Serial.begin(115200);
  pinMode(PWM_LED, OUTPUT);
  digitalWrite(PWM_LED,LOW);
  ledcAttach(PWM_LED,500,8);
  ledcWrite(PWM_LED,0);
  pinMode(RESET_PC817,OUTPUT);
  digitalWrite(RESET_PC817,LOW);
  Wire.begin(SDA_P,SCL_P);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  delay(100);
  resetTCA(5000);
  scan();
  Serial.println("Comandos: T=scan | R=reset 5 segundos + scan | H=reset HIGH 5 segundos + scan");
}
void loop() {
  if (Serial.available()) {
    char c = toupper((unsigned char)Serial.read());
    if (c == 'T') scan();
    else if (c == 'R' || c == 'H') { resetTCA(5000); scan(); }
  }
  delay(10);
}
