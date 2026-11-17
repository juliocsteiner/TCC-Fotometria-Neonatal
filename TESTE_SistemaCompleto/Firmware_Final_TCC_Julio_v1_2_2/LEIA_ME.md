# Firmware final do TCC — v1.2.2

**Correções da v1.2.1:** leitura de irradiância igual a zero e navegação dos botões.

**Escopo:** bancada experimental; **não utilizar em pacientes**. Firmware preparado para ESP32-WROOM-32 com Arduino-ESP32 3.x, TFT ILI9341, multiplexador TCA9548A e 5 sensores AS7341/AS7343. Não se trata de alarme clínico certificado.

## O que mudou

1. **Irradiância zero é leitura válida.** Se a comunicação e a leitura espectral forem bem-sucedidas, `RAW_principal <= fundo` resulta em `E=0.00` e sensor `OK`, inclusive com os LEDs desligados e PWM=0. Não existe limiar mínimo de irradiância para aceitar um sensor.
2. **Leituras continuam no modo OFF.** Antes, o firmware pulava aquisição quando PWM=0; o painel ficava permanentemente com `0/5` e os sensores apareciam sem leitura. Agora a saída PWM permanece em 0% em OFF, mas os sensores são consultados normalmente. **OFF não significa desligar a eletrônica.**
3. **Falha de sensor não encerra as tentativas.** Se restarem menos de quatro sensores válidos, o PWM é colocado em 0%, a correção é bloqueada, mas as leituras prosseguem. O operador pode voltar a ligar somente após serem novamente observados pelo menos quatro sensores válidos.
4. **Reconexão:** se um módulo deixar de inicializar ou acumular três falhas consecutivas de leitura, é programada uma nova tentativa de detecção em até 5 segundos. A reconexão não liga os LEDs automaticamente.
5. **Botões:** Avançar e Voltar navegam entre as três páginas, mesmo em OFF ou FALHA. O firmware utiliza interrupções GPIO com eventos curtos/longos armazenados até que a leitura dos sensores termine, evitando perder toques rápidos durante leituras I²C. Os comandos executam **ao soltar o botão**.
6. **Interface TFT preservada:** somente os valores, rótulos de status e partes alteradas da barra gráfica são sobrescritos. A estrutura fixa é desenhada exclusivamente na entrada ou troca de página.
7. **Sem mudanças na calibração nem no controle:** coeficientes da aba `CAL_30` para 30 cm; controle incremental semiautomático, PWM até 100%, meta de 8–9 µW/cm²/nm e aviso de extrapolação acima de 75%.

## Instalação

1. Extraia o ZIP inteiro, sem misturar arquivos de versões anteriores.
2. Abra `Firmware_Final_TCC_Julio_v1_2_2/Firmware_Final_TCC_Julio_v1_2_2.ino` na Arduino IDE.
3. Selecione sua placa ESP32 e Arduino-ESP32 **3.x**. Instale as bibliotecas **Adafruit GFX**, **Adafruit ILI9341**, **Adafruit AS7341** e **SparkFun AS7343** e dependências.
4. Abra o **Monitor Serial a 115200 baud**. Reinicie a placa, preferencialmente sem pressionar POWER (GPIO2 é um pino de strapping no ESP32).
5. Deixe os LEDs desativados (PWM 0%) para o primeiro teste. Mesmo em OFF, a página Sensores deve mostrar `0.00` com status `OK` se os cinco sensores responderem corretamente. A contagem principal deve indicar `5/5` (ou `4/5` se um estiver realmente indisponível).
6. Pressione e **solte** Avançar e Voltar. A navegação deve passar por Painel, Sensores e Controle sem precisar ligar os LEDs.
7. Somente depois, inicie uma operação pelo botão POWER e verifique que a leitura se atualiza corretamente antes de solicitar controle.

## GPIOs / botões

| Componente | GPIO |
|---|---:|
| SDA / SCL | 17 / 16 |
| TCA RESET via PC817 inversor | 19 |
| PWM drivers | 32 |
| TFT MOSI / SCK / MISO | 14 / 25 / 36 |
| TFT CS / DC / RESET / LED | 26 / 33 / 13 / 27 |
| Selecionar | 18 |
| Voltar | 21 |
| Avançar | 22 |
| POWER | 2 |
| Sensores S1 a S5 / TCA | 2, 3, 5, 6, 7 |

Botões são **ativos em nível baixo**: um terminal no GPIO respectivo, outro no **GND do ESP32**; `INPUT_PULLUP` está habilitado. **Não conecte os botões ao GND_ISO**, que pertence ao circuito isolado de sensoriamento.

## Diagnóstico pelo Monitor Serial

Envie um dos caracteres abaixo com 115200 baud:

| Comando | Resultado |
|---|---|
| `D` | Estado, endereço I²C do MUX, contagem de sensores e **RAW, E, motivo e inicialização de S1 a S5** |
| `B` | Nível elétrico instantâneo dos quatro botões; `0` = pressionado, `1` = solto |
| `P` / `V` | Próxima / anterior página (teste sem usar os botões) |
| `R` | Reinicializa multiplexador e sensores, **apenas em OFF ou FALHA** |
| `I` | Liga a operação, apenas se houver pelo menos quatro sensores válidos |
| `X` | Desliga os LEDs (PWM=0) |
| `?` | Ajuda |

Cada clique físico também deve imprimir `[BOTAO] AVANCAR CURTO`, `[BOTAO] VOLTAR CURTO` etc. Para validar a fiação, mantenha cada botão pressionado enquanto envia `B`: o GPIO associado deve ser `0`. Se não mudar, verificar fiação/pino/GND antes de alterar código.

### Como distinguir zero de falha

| Sintoma no TFT e em `D` | Significado |
|---|---|
| `S1 ... 0.00 OK` | Sensor respondeu, intensidade próxima de zero; **válido** |
| `S1 ... --.-- AUSENTE` | Sensor não respondeu/identificação ausente; **inválido** |
| `S1 ... --.-- FALHA` | Aquisição não foi concluída; **inválido** |
| `S1 ... --.-- SAT` | Canal saturado; **inválido** |
| `S1 ... 0.00 OBSTR` | Suspeita de obstrução; valor adquirido mas temporariamente excluído da média |

A média só existe com pelo menos quatro sensores **válidos**. Falha real não é convertida em zero porque isso distorceria a realimentação de PWM.

## Testes realizados e limitações

O código passou por checagem C++17 contra cabeçalhos **simulados** e testes automatizados simulados de: 5 sensores com RAW zero → 5/5 válidos e E=0; sensor ausente → E indisponível e 4/5 válidos; toques de navegação capturados durante aquisição; navegação em OFF e FALHA; e manutenção do cache gráfico sem redesenho integral. **Ainda precisa ser compilado com as bibliotecas reais na Arduino IDE e testado na sua PCB.**

Ao usar PWM acima de 75%, as estimativas radiométricas estão fora da faixa de calibração experimental e devem ser confirmadas com o THOR. Verifique corrente, temperatura e tensão; o firmware não implementa monitoramento térmico independente nem certifica segurança elétrica/uso clínico.
