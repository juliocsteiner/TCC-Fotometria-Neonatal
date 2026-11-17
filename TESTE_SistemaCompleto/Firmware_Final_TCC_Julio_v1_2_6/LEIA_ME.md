# Firmware TCC v1.2.6 — tolerância à obstrução e aviso ampliado

Este pacote parte **exatamente do código v1.2.5 enviado na conversa**. Mantém a calibração individual THOR a 30 cm, três sensores AS7343 e dois AS7341, TFT 320 × 240, controle com autorização explícita, PWM até 100%, botão Selecionar pressionado por 2 segundos para ligar/desligar, diagnósticos I²C e tela persistente com campos sobrescritos (sem apagamento geral a cada leitura).

**Protótipo de bancada; não é equipamento clínico.** Entre 75% e 100% de PWM, as estimativas extrapolam a calibração disponível, exigindo confirmação com THOR.

## 1. Comparação de obstrução em irradiância baixa

O sensor que responde normalmente pode medir **0,00 µW/cm²/nm** e continuar com status `OK`. Valores próximos de zero não representam desconexão.

O programa calcula primeiro a **média preliminar** das irradiâncias estimadas com leituras físicas bem-sucedidas, antes de excluir qualquer sensor por discrepância. Quando essa média preliminar é **menor ou igual a 0,50 µW/cm²/nm**, a diferença de 90% fica desativada para todos os sensores naquela aquisição. Isso evita alarmes espúrios porque uma comparação percentual em torno de zero é instável. O limiar pode ser ajustado no código em `PISO_MEDIA_ESCURA_UW`.

Quando a operação está **OFF**, ou o PWM está em 0%, a comparação de 90% também fica desativada. Nesse estado somente o sensor **sem resposta de comunicação/identificação** recebe o indicador de obstrução `OBSTR/COM`. O programa não pode distinguir um cabo desconectado de falha de alimentação ou I²C apenas pelo ACK: `OBSTR/COM` é um nome operacional, não a comprovação de desconexão física.

Com PWM ligado e média preliminar acima de 0,50 µW/cm²/nm, continua valendo a regra anterior: a leitura de cada sensor é comparada ao outro sensor de irradiância mais próxima, com `D = |Ei-Ej| / max(Ei,Ej)`. Quando `D >= 90%`, exibe `OBSTR/90%`. A posição do sensor no plano não entra no cálculo.

## 2. Duas ou mais obstruções ao mesmo tempo: espera de 10 segundos

O temporizador começa quando pelo menos **dois dos cinco sensores estão simultaneamente em `OBSTR/COM` ou `OBSTR/90%`**, estando a operação ligada.

- **Antes de completar 10 segundos:** o firmware continua adquirindo dados, navegando entre telas e mostrando o alerta. Mantém o PWM aplicado, mas **não emite novos comandos do ajuste automático**, pois a média representativa precisa de ao menos quatro sensores válidos. Se um ajuste automático estava em andamento, ele é cancelado e precisará ser autorizado novamente depois da recuperação.
- **Se a quantidade de obstruídos voltar a menos de dois antes de 10 segundos:** o contador é zerado e não ocorre desligamento por essa ocorrência.
- **Se persistirem pelo menos duas obstruções durante 10 segundos contínuos, confirmados por novas aquisições:** entra em `FALHA_SENSORES`, aplica PWM = 0% e continua lendo os sensores para diagnóstico. Uma retomada da operação exige novo acionamento do botão Selecionar (2 segundos) e pelo menos quatro leituras válidas.
- **Se houver outras falhas além das obstruções** (por exemplo, saturação e falha de leitura, deixando menos de quatro sensores válidos), o bloqueio de segurança anterior continua imediato. A tolerância de 10 s não deve mascarar falhas de outra natureza.

Enquanto as duas obstruções são temporárias, **o valor representativo fica `--.--`/`NA`, nunca um zero inventado**. A tela informa, por exemplo, `OBSTRUCAO 2/5: 4.2/10s | PWM mantido` e o Serial registra `[OBSTRUCAO]`.

A duração é medida com `millis()` e verificada após cada aquisição. Por isso, a atuação efetiva pode ocorrer um pouco **depois** de 10 s, mas nunca antes do limiar programado. O tempo não é somado entre obstruções separadas.

**Nota de segurança:** manter o PWM por até 10 s com menos de quatro leituras válidas é uma escolha experimental solicitada para a bancada. Não é uma função de segurança validada para uso em pacientes. O desligamento manual por Selecionar pressionado por 2 segundos permanece disponível.

## 3. Aviso amarelo ATENCAO ampliado

Quando a irradiância está fora do objetivo de **8,0 a 9,0 µW/cm²/nm**, o cabeçalho mostra o selo amarelo maior `ATENCAO`, usando fonte de tamanho 2 e subtítulo `FORA DO ALVO`. A mesma área mostra `2+ OBSTRUIDOS`, `FALHA SENSORES`, `SATURACAO` ou `LIMITE PWM` nas condições correspondentes.

O aviso ampliado ocupa somente a área do cabeçalho (`x = 192..310`, `y = 3..35`). **O fundo e os cartões não são apagados nem redesenhados em cada leitura**: a área do selo só é reconstruída quando ocorre uma mudança efetiva de cor ou formato, e as mensagens são atualizadas por sobrescrita.

## 4. Procedimento de teste no hardware

1. Extraia o ZIP. Abra somente `Firmware_Final_TCC_Julio_v1_2_6.ino` na Arduino IDE. Bibliotecas necessárias: **Adafruit GFX**, **Adafruit ILI9341**, **Adafruit AS7341**, **SparkFun AS7343** e **Arduino-ESP32 3.x**.
2. Ligue o ESP32, com operação OFF. Os cinco sensores presentes devem aparecer `OK` mesmo com valores próximos de zero. Desconectar um módulo deve produzir `OBSTR/COM`, sem alertas `OBSTR/90%` nos demais.
3. Mantenha Selecionar (GPIO 18) pressionado por 2 segundos e verifique a fonte em bancada. Fora da meta, o aviso `ATENCAO / FORA DO ALVO` deve aparecer em tamanho maior.
4. Com os LEDs ligados, provoque **duas leituras simultaneamente classificadas como `OBSTR`**. Até o instante anterior a 10 s o PWM deve permanecer no valor anterior. O rodapé e a terceira tela mostram o tempo decorrido.
5. Remova uma obstrução antes dos 10 s. Verifique que o temporizador é zerado e a operação não é desligada por essa ocorrência.
6. Repita e mantenha as duas obstruções por mais de 10 s. O Serial deverá anunciar a confirmação e o PWM deverá ir para 0%. Verifique por medição elétrica que os LEDs realmente foram desligados.
7. Em operação com irradiância média **<= 0,50 µW/cm²/nm**, verifique que variações entre leituras próximas do escuro não criam `OBSTR/90%`.
8. Se o sensor não for reconhecido, envie `T` no Serial (OFF) para diagnóstico do MUX/ACK. `D` lista valores, status e a contagem do temporizador. `R` reinicia o TCA e a identificação em OFF. Monitor Serial: **115200 baud**.

### GPIOs mantidos

- SDA 17, SCL 16; TCA9548A RESET por PC817 GPIO 19.
- PWM GPIO 32 (500 Hz, 8 bits); limite 100%.
- TFT MOSI 14, SCK 25, MISO 36, CS 26, DC 33, RST 13, LED 27.
- Selecionar GPIO 18 (longo 2 s = liga/desliga; curto = confirmar), Voltar GPIO 21, Avançar GPIO 22.

### Verificações

- Arquivo verificado sintaticamente em C++17 com **bibliotecas Arduino simuladas**, sem erros ou avisos de compilação nesse ambiente.
- Testes C++ de lógica: OFF sem divergência percentual; média próxima de zero; assimetria do plano; duas obstruções por 9,999 s sem desligamento; 10,000 s com bloqueio; recuperação antes de 10 s e reinício do prazo; suspensão de ajuste automático; outras falhas mantendo parada imediata; alerta ampliado.
- **Não houve compilação com as bibliotecas reais nem teste físico neste ambiente.** Validar no ESP32/TFT e com o radiômetro THOR antes de usar os resultados na monografia.
