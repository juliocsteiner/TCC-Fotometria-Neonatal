#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 

// Endereço I2C padrão do display OLED SSD1306 (0x3C ou 0x3D)
#define SCREEN_ADDRESS 0x3C 

// Pinagem I2C padrão do ESP32 WROOM
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  delay(500);

  // Inicializa o barramento I2C nos pinos 21 e 22
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicializa o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("[ERRO] Falha ao inicializar o display SSD1306!"));
    Serial.println(F("Dica: verifique a alimentacao (3.3V/GND) e tente alterar o endereco para 0x3D."));
    while(true); // Trava o programa em caso de erro
  }

  Serial.println(F("[OK] Display detectado com sucesso!"));

  // Limpa o buffer gráfico inicial
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  // Tela 1: Apresentação do Projeto
  display.clearDisplay();
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE); // Borda externa
  
  display.setTextSize(1);
  display.setCursor(18, 12);
  display.println(F("FOTOTERAPIA"));
  
  display.setCursor(35, 26);
  display.println(F("NEONATAL"));

  display.setCursor(14, 44);
  display.println(F("TESTE DO DISPLAY"));
  display.display();
  delay(3000);

  // Tela 2: Simulação de Leitura do Sistema
  display.clearDisplay();
  display.drawLine(0, 16, 128, 16, SSD1306_WHITE); // Linha divisória do cabeçalho
  
  display.setTextSize(1);
  display.setCursor(4, 4);
  display.println(F("MONITOR IRRADIANCIA"));

  display.setCursor(4, 24);
  display.println(F("Valor Medio:"));

  display.setTextSize(2);
  display.setCursor(10, 36);
  display.println(F("9.8 uW")); // Valor simulado dentro da faixa terapêutica

  display.setTextSize(1);
  display.setCursor(4, 54);
  display.println(F("Status: OPERACIONAL"));
  display.display();
  delay(3000);

  // Tela 3: Teste de Padrões Gráficos
  display.clearDisplay();
  for (int16_t i = 0; i < display.height() / 2; i += 3) {
    display.drawRect(i * 2, i, display.width() - 4 * i, display.height() - 2 * i, SSD1306_WHITE);
  }
  display.display();
  delay(2000);
}