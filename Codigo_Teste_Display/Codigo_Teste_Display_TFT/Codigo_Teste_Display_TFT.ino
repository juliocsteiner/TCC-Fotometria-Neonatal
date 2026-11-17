#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// =====================================================
// PINAGEM FINAL
// =====================================================
#define TFT_CS    26
#define TFT_DC    33
#define TFT_RST   13
#define TFT_MOSI  14
#define TFT_SCK   25
#define TFT_MISO  36  // VP na placa
#define TFT_LED   27

// GPIO32 reservado para o PWM do projeto.

// =====================================================
// CONFIGURACOES
// =====================================================
const bool CONTROLAR_BACKLIGHT = true;
const uint8_t BACKLIGHT_LIGADO = HIGH;

// Frequencia conservadora para teste na protoboard.
const uint32_t SPI_FREQUENCIA = 4000000;

// Ordem: SPI, DC, CS, RESET.
Adafruit_ILI9341 tft(&SPI, TFT_DC, TFT_CS, TFT_RST);

uint32_t ultimaAtualizacao = 0;
uint32_t contador = 0;

// =====================================================
// TESTE DE CORES
// =====================================================
void testarCores() {
  const uint16_t cores[] = {
    ILI9341_RED,
    ILI9341_GREEN,
    ILI9341_BLUE,
    ILI9341_WHITE,
    ILI9341_BLACK
  };

  const char* nomes[] = {
    "VERMELHO",
    "VERDE",
    "AZUL",
    "BRANCO",
    "PRETO"
  };

  for (uint8_t i = 0; i < 5; i++) {
    Serial.print("Teste de cor: ");
    Serial.println(nomes[i]);

    tft.fillScreen(cores[i]);
    delay(800);
  }
}

// =====================================================
// TELA PRINCIPAL
// =====================================================
void desenharTela() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(false);

  // Borda da tela.
  tft.drawRect(
    0, 0,
    tft.width(), tft.height(),
    ILI9341_WHITE
  );

  // Titulo.
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(16, 12);
  tft.print("TESTE DISPLAY TFT");

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(16, 38);
  tft.print("ESP32 | ILI9341 | SPI 4 MHz");

  tft.drawFastHLine(12, 54, 296, ILI9341_WHITE);

  // Formas geometricas.
  tft.fillRect(25, 70, 60, 45, ILI9341_RED);

  tft.fillCircle(155, 92, 23, ILI9341_GREEN);

  tft.fillTriangle(
    225, 115,
    255, 70,
    285, 115,
    ILI9341_BLUE
  );

  // Legendas.
  tft.setCursor(30, 123);
  tft.print("VERMELHO");

  tft.setCursor(140, 123);
  tft.print("VERDE");

  tft.setCursor(245, 123);
  tft.print("AZUL");

  // Contador.
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(16, 145);
  tft.print("Contador:");

  // Moldura da barra animada.
  tft.drawRect(15, 178, 290, 22, ILI9341_WHITE);

  // Pinagem e instrucoes.
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);

  tft.setCursor(16, 210);
  tft.print("CS26 DC33 RST13 MOSI14 SCK25 MISO36");

  tft.setCursor(16, 224);
  tft.print("Envie T no Serial para repetir");
}

// =====================================================
// EXECUTA A SEQUENCIA DE TESTE
// =====================================================
void executarTeste() {
  testarCores();
  desenharTela();

  contador = 0;
  ultimaAtualizacao = millis();

  Serial.println();
  Serial.println("Comandos enviados ao display.");
  Serial.println("Confira cores, formas, contador e barra.");
  Serial.println("Envie T para repetir.");
}

// =====================================================
// INICIALIZACAO
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==================================");
  Serial.println("TESTE TFT - VERSAO CORRIGIDA");
  Serial.println("Controlador configurado: ILI9341");
  Serial.println("==================================");

  Serial.println("CS   = GPIO26");
  Serial.println("DC   = GPIO33");
  Serial.println("RST  = GPIO13");
  Serial.println("MOSI = GPIO14");
  Serial.println("SCK  = GPIO25");
  Serial.println("MISO = GPIO36 / VP");
  Serial.println("BL   = GPIO27");

  // CORRECAO: configurar como saida ANTES de escrever.
  if (CONTROLAR_BACKLIGHT) {
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, BACKLIGHT_LIGADO);

    Serial.println("GPIO27 configurado como saida HIGH.");
  }

  // CORRECAO: mesma ordem para o CS.
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  // SPI com a pinagem escolhida.
  // Ordem: SCK, MISO, MOSI, SS.
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  // A biblioteca inicializa o display e aciona TFT_RST.
  tft.begin(SPI_FREQUENCIA);

  // Paisagem: 320 x 240 para ILI9341.
  tft.setRotation(1);

  executarTeste();
}

// =====================================================
// LOOP PRINCIPAL
// =====================================================
void loop() {
  // Comando para repetir o teste.
  while (Serial.available() > 0) {
    char comando = Serial.read();

    if (comando == 'T' || comando == 't') {
      executarTeste();
    }
  }

  // Atualiza contador e barra a cada 250 ms.
  if (millis() - ultimaAtualizacao >= 250) {
    ultimaAtualizacao = millis();

    // Limpa apenas o numero anterior.
    tft.fillRect(135, 143, 170, 24, ILI9341_BLACK);

    tft.setCursor(135, 145);
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_WHITE);
    tft.print(contador);

    // Barra de 0 a 100%.
    int largura = (contador % 101) * 286 / 100;

    tft.fillRect(17, 180, 286, 18, ILI9341_BLACK);

    if (largura > 0) {
      tft.fillRect(17, 180, largura, 18, ILI9341_CYAN);
    }

    contador++;
  }
}