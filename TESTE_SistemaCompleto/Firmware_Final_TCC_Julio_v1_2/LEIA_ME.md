# Firmware final TCC — v1.2 — interface com sobrescrita de valores

## Alteração solicitada: sem apagar e redesenhar a tela a cada aquisição

Esta versão conserva a lógica experimental da v1.1 (5 sensores, calibração CAL_30, controle semiautomático e PWM 0–100%), mudando **somente o mecanismo de renderização do display**.

- A estrutura fixa de cada página (fundo, cartões, títulos, linhas e unidades) é desenhada **uma vez ao entrar naquela página**.
- As regiões numéricas e de estado usam `setTextColor(cor, corDeFundo)` com **largura de caracteres fixa** e espaços à direita. Isso sobrescreve os caracteres anteriores no mesmo lugar, evitando resíduos quando o número diminui (por exemplo, `10.00` → `9.50`). Não há apagamento da tela inteira.
- **Cache por campo:** texto e cor são comparados com o último valor exibido. Se forem idênticos, o programa não transmite nenhum comando de desenho para aquele campo.
- A barra horizontal de irradiância altera somente o trecho que aumentou ou diminuiu. Mudança de cor pode atualizar apenas essa faixa de 5 pixels.
- O selo de estado no cabeçalho só é redesenhado quando sua cor muda; seu texto só muda quando necessário.
- A **única chamada `fillScreen()`** está na função `desenharPaginaEstatico()`, executada **somente ao abrir ou mudar de página**, inclusive no início.
- A taxa de aquisição de sensores e a cadência lógica de controle permanecem as mesmas; a taxa de **desenho não acompanha a taxa de leitura** quando o conteúdo não mudou.

## Verificações realizadas nesta revisão

- Análise de compilação C++17 contra cabeçalhos *simulados*, que **não substituem** as bibliotecas reais Adafruit/SparkFun e a Arduino IDE.
- Teste automatizado simulado: 10 solicitações de atualização sem alteração de conteúdo resultaram em **0 comandos de escrita** para o TFT.
- Mudança isolada da irradiância no painel: atualiza o texto de irradiância e o trecho da barra, sem redesenhar os cartões ou o fundo.
- Mudança isolada de valor/estado de S1: atualiza somente os dois campos respectivos.
- Troca de página: um único desenho da estrutura fixa da nova página, como esperado.

## Como instalar

Abra `Firmware_Final_TCC_Julio_v1_2.ino` na pasta `Firmware_Final_TCC_Julio_v1_2` pela Arduino IDE. Selecione o ESP32 com Arduino-ESP32 3.x e instale Adafruit GFX, Adafruit ILI9341, Adafruit AS7341 e SparkFun AS7343. Compilar e confirmar no display físico antes de operar a malha de LEDs.

## Atenção

A alteração é exclusivamente gráfica: **não amplia a faixa radiométrica validada**, não certifica o uso clínico e não garante segurança térmica em PWM 100%. Os avisos de extrapolação >75% e o bloqueio por falha/saturação foram preservados.

---

## Documentação técnica das funcionalidades da v1.1 preservadas

# Firmware final do TCC — versão experimental v1.1 (PWM até 100%)

**Projeto:** monitoramento da irradiância estimada em cinco pontos e controle incremental semiautomático da fonte experimental de dez LEDs azuis (dois drivers PT4115, um PWM comum). **Protótipo exclusivamente de bancada. Não utilizar em pacientes.**

## Mudanças da versão 1.1

- **Limite físico de PWM:** `PWM_MAX = 100.0f`, mantendo 500 Hz e 8 bits; ao solicitar 100%, o ESP32 envia `ledcWrite(GPIO32, 255)` (duty máximo de 8 bits). **O botão POWER/rotina OFF e a proteção de falha grave enviam PWM 0.**
- **Objetivo específico:** o firmware verifica a faixa **8,0 a 9,0 µW/cm²/nm**. A referência interna do controlador incremental passa a **8,5 µW/cm²/nm**. A faixa anteriormente documentada de **8,0 a 10,0 µW/cm²/nm** permanece como *faixa geral informativa*, mas um valor entre 9,0 e 10,0 **não** encerra a correção da nova meta 8–9.
- **Confirmação:** o operador pressiona SELECIONAR para solicitar o ajuste e pressiona novamente para autorizá-lo; a atuação pode ultrapassar 75%, **até 100%**. O limite de 5 pontos percentuais por ciclo, a estabilização entre mudanças e as 25 tentativas máximas são preservados.
- **Sucesso:** `META 8-9 ATINGIDA` apenas depois de **3 ciclos de aquisição consecutivos** com a média dos pontos válidos entre **8,0 e 9,0** (inclusive), sem sensor saturado.
- **Insucesso:** se atingir **100%** e a média continuar abaixo de 8,0, mostra `LIMITE DO PWM`. Se houver saturação, mostra `SENSOR SATURADO` e suspende o ajuste. `MAX DE PASSOS` indica que foram consumidas 25 correções sem atingir o objetivo. Nenhum desses estados informa sucesso.
- **Extrapolação:** >75% o display exibe aviso de estimativa **EXTRAPOLADA** e nunca apresenta verde como indicador de validação; o texto `8-9: SIM*` significa **estimativa**, **não comprovação metrológica pelo THOR**. O perfil de detecção de obstruções fica desativado acima de 75%, pois só foi caracterizado até esse ponto.
- **Serial:** além das linhas `DADOS;...`, emite `ALVO_8_9;tempo_ms;pwm;SIM_ESTIMADO/NAO;EXTRAPOLADO=0/1;SATURADOS=n`. Pode ser utilizado para registrar cada ponto da experiência.

## Configuração da PCB atual

| Função | GPIO ou conexão |
|:--|:--|
| I²C SDA / SCL | 17 / 16 |
| Reset do TCA9548A por PC817 inversor | 19 |
| PWM comum dos dois drivers PT4115 | 32 (500 Hz, 8 bits) |
| TFT MOSI / SCK / MISO | 14 / 25 / 36 |
| TFT CS / DC / RESET / iluminação | 26 / 33 / 13 / 27 |
| Selecionar / Voltar / Avançar | 18 / 21 / 22 |
| Liga/Desliga lógico | 2 — **pino de strapping**: não pressionar durante energização |
| Canal TCA para S1…S5 | 2 / 3 / 5 / 6 / 7 |
| Modelo S1…S5 | AS7343 / AS7341 / AS7343 / AS7341 / AS7343 |

## Dependências na Arduino IDE

Arduino-ESP32 **3.x**; Adafruit GFX, Adafruit ILI9341, Adafruit AS7341 (e dependências Adafruit BusIO) e SparkFun AS7343. Abra `Firmware_Final_TCC_Julio.ino` em uma pasta de mesmo nome.

## Calibração utilizada e limite de validade

A base continua sendo a planilha `Analise_calibracao_THOR_17-11-2026.xlsx`, aba **CAL_30** — cinco pontos fixos a **30 cm**. A equação do firmware é `E_i = a_i × max(RAW_principal,i − Fundo_i, 0)`; `E_rep` é a média dos pontos válidos (no mínimo 4). Os coeficientes são os mesmos da versão anterior:

| Sensor | Modelo | Fundo RAW | Coeficiente angular a |
|:--|:--|--:|--:|
| S1 | AS7343 | 70,025 | 0,007380975596 |
| S2 | AS7341 | 214,000 | 0,002344222903 |
| S3 | AS7343 | 63,350 | 0,001207907889 |
| S4 | AS7341 | 199,750 | 0,005849934637 |
| S5 | AS7343 | 80,875 | 0,016314020884 |

As calibrações e os perfis locais foram obtidos em **10%, 30%, 50%, 60% e 75%**, com **68%** reservado para verificação. **Não foram obtidos coeficientes de referência acima de 75%. Portanto, entre 75% e 100%, a estimativa numérica está extrapolada e não tem exatidão demonstrada.** Não assumir linearidade nem ausência de saturação nessa faixa. A comparação com o THOR nos mesmos pontos, distância, tampa e orientação é necessária para afirmar que a fonte realmente atingiu 8–9. A calibração de centro e os registros de 40 cm não substituem o perfil CAL_30.

## Controles

- **POWER**: ativa/desativa o sistema; partida em aproximadamente 50% se pelo menos quatro sensores forem inicializados; OFF em PWM=0.
- **AVANÇAR / VOLTAR**: alternam `Painel`, `Sensores` e `Controle`.
- **SELECIONAR** com média fora de 8–9: solicita ajuste; um segundo toque confirma autorização para atuar **até 100%**. `VOLTAR` cancela antes da autorização.
- **SELECIONAR pressionado por 0,9 s durante ajuste**: interrompe as correções e mantém o último PWM; POWER desliga.
- **Serial 115200:** `?` ajuda, `X` desliga, `I` inicia, `P` muda página.

A tela principal apresenta irradiância representativa, PWM, quantidade de sensores válidos e `8-9: SIM*` ou `8-9: NAO`. Na barra de irradiância, duas linhas brancas marcam 8 e 9. A página Controle mostra objetivo, referência central de 8,5, faixa geral 8–10, PWM 0–100 e número de passos.

**Sinalização:** verde indica meta estimada dentro da faixa **apenas até 75%** e com cinco sensores válidos; amarelo indica desvio, sensor faltante, saturação, limite ou estimativa extrapolada; ciano indica controle em execução. Não há alarme sonoro. Trata-se de indicadores experimentais, não alarmes clínicos certificados.

## Roteiro mínimo de validação ao operar acima de 75%

1. **Antes de conectar os LEDs:** verifique com osciloscópio o GPIO32 (500 Hz e duty máximo de 100%); revise fonte 24 V, drivers, cabeamento e dissipação. Tenha um meio independente de retirar alimentação.
2. **Com a fonte em bancada e sem pacientes:** confirme que o sistema permite solicitar controle, mostra `PWM>75: EXTRAPOLADO` e registra o duty efetivo no Serial.
3. **Registre os patamares reais 75, 80, 85, 90, 95 e 100% (se eletricamente/termicamente seguro):** monitore corrente, tensão, temperaturas e indicação do THOR. Atenção a saturação dos AS7343 (limiar operacional RAW ≈17.990).
4. **No mesmo plano óptico de 30 cm:** compare a leitura real do THOR à média dos cinco pontos. Registre se 8–9 foi atingido **pelo instrumento de referência**, e não somente pelo firmware.
5. **Teste o ramo de insucesso:** se a estimativa permanecer abaixo de 8 em 100%, deve exibir `LIMITE DO PWM`; se um sensor saturar, deve suspender o ajuste. Teste desconexão de 1 e 2 sensores e cancelamento por POWER.
6. **Não extrapole automaticamente para 40 cm ou outro espectro:** a diferença espacial e a geometria invalidam a aplicação direta dos coeficientes.

## Verificação do código e limitações

A v1.1 foi compilada com **interfaces simuladas em C++** (não são as bibliotecas Arduino reais) e teve a lógica exercitada para: chegada a 100% com indicação de limite, três leituras 8–9, redução a partir de 9,5 e suspensão quando há saturação. Isso confirma a consistência básica do código, **não garante compilação na sua configuração local nem funcionamento físico**. Após instalar as bibliotecas, compile na Arduino IDE e execute os testes de bancada.

**Advertência especial:** o firmware não mede temperatura de LEDs/driver, corrente elétrica, tensão de alimentação, nem possui intertravamento físico independente. Os testes documentados do sistema não qualificam operação contínua em PWM 100%; a possibilidade de comando até 100% é para investigação supervisionada, não uma declaração de segurança da matriz. A isolação B0505S/ISO1540/PC817 não configura por si só certificação Classe II.
