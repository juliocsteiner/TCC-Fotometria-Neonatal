# TCC v1.2.7 — registro de ensaios na flash do ESP32

Esta versão é uma evolução **do firmware v1.2.6 enviado pelo usuário**, adicionando histórico persistente sem mudar os coeficientes de calibração, canais do multiplexador, pinos, lógica de obstrução de 10 segundos, PWM até 100%, autorização de controle ou o comportamento de sobrescrita da interface TFT.

**Protótipo experimental de bancada: não utilizar em pacientes.** As estimativas acima de 75% de PWM extrapolam a calibração existente e devem ser conferidas com o THOR. Não é sistema de alarmes IEC certificado.

## 1. O que mudou

A versão 1.2.6 transmitia `DADOS;...` pela Serial, mas **não preservava o histórico** depois de retirar a alimentação. A v1.2.7 adiciona `LittleFS` na flash interna (partição SPIFFS/LittleFS) e um contador de inicializações em `Preferences` (NVS).

- Em cada aquisição com a operação ativa: tempo em milissegundos desde o boot, número da sessão de boot, PWM, estado, quantidade de sensores válidos, média representativa, cinco irradiâncias, status de cada sensor, valores RAW principais, obstruções, tempo de obstrução dupla, passos de controle, página, meta e extrapolação.
- Quando ocorrem eventos: acionamentos dos botões, início/paro da operação, mudanças de PWM, falha/recuperação individual de sensores, autorização do controle, início/recuperação/limiar de 10 s de obstruções, conclusão e bloqueios do controle.
- Em `OFF`: não grava milhares de leituras de fundo desnecessárias. Os **eventos** continuam registrados. Em `FALHA_SENSORES` com PWM desligado, salva até quatro aquisições posteriores ao bloqueio, evitando sobrescrever a campanha anterior enquanto a placa aguarda o operador.
- Os arquivos são gravados e fechados a cada linha. Isso reduz a perda de dados ao desligar, embora uma falha abrupta durante uma gravação ainda possa afetar a linha final.
- O cabeçalho TFT mostra `LOG OK` quando montado e sem erro de escrita; `LOG ERR` caso contrário. O cabeçalho `ATENCAO` continua maior e a interface continua sobrescrevendo apenas valores.

Os dois arquivos `/tcc_anterior.csv` e `/tcc_atual.csv` mantêm aproximadamente **até 440 KiB de histórico total**. Quando a janela atual ultrapassa ~220 KiB, o firmware substitui a janela antiga. Isso é um histórico **deslizante**, não armazenamento ilimitado; número de leituras depende do comprimento das linhas e do tamanho da partição. **Exporte após cada sessão de testes** para evitar perdas por rotação.

## 2. Preparação antes dos testes

1. Na Arduino IDE, escolha a placa ESP32 correta e o núcleo `esp32` **3.x**. Selecione um esquema de partição que **contenha área SPIFFS/LittleFS** (não use `Huge APP` se remover a partição de arquivos).
2. Abra `Firmware_Final_TCC_Julio_v1_2_7.ino` e compile/grave. Bibliotecas: Adafruit GFX, Adafruit ILI9341, Adafruit AS7341, SparkFun AS7343; LittleFS e Preferences são fornecidas pelo pacote ESP32.
3. Na **primeira execução deste firmware**, uma partição LittleFS ainda não formatada pode ser inicializada automaticamente. **ATENÇÃO: isso pode apagar arquivos anteriores armazenados na partição de arquivos**. Depois da primeira inicialização, um erro de montagem NÃO provoca reformatação automática, para proteger logs existentes.
4. Confirme no TFT o indicador verde **`LOG OK`** ou, na Serial 115200, a linha `[LOG] PRONTO; sessao=...`. **Não inicie ensaios que dependam do histórico se aparecer `LOG ERR`.** O firmware pode continuar controlando sem gravar.
5. Monte os cinco sensores na geometria calibrada de **30 cm**. Faça um teste curto e confirme que o histórico tem amostras.

**Não use Arduino IDE > Ferramentas > Erase All Flash Before Sketch Upload para recuperar dados** e não altere o esquema de partições depois do teste: isso pode apagar as gravações. A versão 1.2.6 **não gravava** dados recuperáveis por este método; somente ensaios feitos com v1.2.7 têm histórico.

## 3. Como recuperar depois de retirar o ESP32 do sistema

1. Termine cada ensaio e segure **Selecionar (GPIO18) por 2 s**, desligando a operação e aplicando PWM = 0%.
2. Desligue a fonte externa, espere a placa perder energia, desconecte o ESP32 do circuito e só então conecte-o ao PC pelo **USB**. Não energize a placa pela fonte e pelo USB simultaneamente sem verificar a arquitetura elétrica e o risco de retorno de corrente.
3. Abra a Arduino IDE > Monitor Serial em **115200 baud**. A placa pode reiniciar; espere a rotina de RESET/I²C terminar (até ~5 s no boot).
4. Digite **`G`** e pressione Enviar. É emitido um **resumo persistente da última operação iniciada** (30 leituras normais consecutivas, 50 intervalos consecutivos, quantidade de leituras com quatro/menos sensores, obstruções, eventos de botões e ajustes). Pode ser solicitado **sem os sensores montados**. A falta dos sensores ao reconectar NÃO apaga o histórico.
5. Digite **`H`** e pressione Enviar para transmitir o CSV histórico completo, delimitado por `INICIO CSV TCC` e `FIM CSV TCC`. Salve o texto entre os marcadores num arquivo `historico_tcc.csv`; o cabeçalho é incluído uma só vez, com separador `;`.
6. Opcionalmente, use `capturar_log_usb.py` para salvar o CSV direto na porta COM. Feche antes o Monitor Serial da IDE. Instale o pacote Python `pyserial`: `python -m pip install pyserial`. Exemplo Windows: `python capturar_log_usb.py COM5`. O programa salva `historico_tcc.csv` e `historico_tcc_resumo_serial.txt`.
7. Execute `python analisar_historico.py historico_tcc.csv` para gerar relatório textual com evidências por operação, médias/desvios dos últimos 50 intervalos consecutivos e exemplos de leituras com quatro sensores.

**Outros comandos existentes:** `?` ajuda; `D` estado atual em RAM (NÃO consulta histórico); `T` diagnóstico do I²C; `B` leitura de GPIOs; `R` reinicializa o MUX em OFF; `X` desliga; `I` liga; `P` e `V` navegam. Os novos são `G` (resumo) e `H` (exportação). Para **apagar deliberadamente** o histórico antes de nova campanha, com a operação OFF, envie `Z` e depois `Y` em até 10 segundos. Sem isso, o histórico não é limpo.

## 4. Formato CSV e significados

Colunas principais: `sessao;ms;tipo;estado;pwm_pct;validos;Erep;E1;E2;E3;E4;E5;S1;S2;S3;S4;S5;RAW1;RAW2;RAW3;RAW4;RAW5;obstruidos;dupla_ms;passos;pagina;meta8a9;extrapolado;evento`.

- `tipo=M`: aquisição; `tipo=E`: evento.
- `Erep` = média **apenas dos sensores válidos**, ou `NA` se houver menos de quatro. Os cinco E individuais permanecem no log, mesmo que um seja excluído por obstrução percentual.
- `S1`–`S5`: `0=OK`, `1=AUSENTE` (I²C/identificação), `2=FALHA` (leitura/configuração), `3=SATURADO`, `4=OBSTRUIDO` (diferença >=90%).
- `estado`: `0=OFF`, `1=INICIANDO`, `2=MONITORANDO`, `3=FORA_FAIXA`, `4=CONFIRMAR`, `5=CONTROLANDO`, `6=CONCLUIDO`, `7=LIMITE_PWM`, `8=SATURACAO`, `9=FALHA_SENSORES`, `10=SEM_CALIBRACAO`.
- `ms` **não é data/hora absoluta**: o ESP32 não tem RTC com bateria neste projeto; o número de `sessao` diferencia reinicializações. Anote a data e hora em seu caderno de laboratório.
- O registro `BOOT` aparece a cada vez que a placa é ligada. Ao conectar pelo USB sem sensores, novos eventos de inicialização podem ocorrer; eles não alteram os registros do ensaio anterior. O comando `G` procura o evento `OPERACAO_INICIADA` mais recente para calcular o resumo.
- Mudança de `pwm_pct` corresponde ao **PWM comandado pelo ESP32**. Não comprova, sem medição elétrica ou THOR, a luz realmente produzida.

## 5. Relação com os critérios de validação

| Critério | Evidência automática disponível? | O que confirmar pessoalmente |
|---|---|---|
| Montagem dos cinco sensores | `S1`–`S5`, modelos/estado, RAW | Foto e desenho do arranjo |
| 30 ciclos de funcionamento normal | `G`/analisador contam 30 leituras consecutivas com cinco válidos, sem obstrução e PWM>0 | Condições de bancada estáveis |
| Fotos da tela principal/sensores | Não | Tirar fotos durante o experimento |
| Falha de um sensor, média de quatro | CSV registra `validos=4` e `Erep` | Confirmar qual sensor foi desconectado |
| Falha de dois sensores/bloqueio | `validos<=3`, `OBSTRUCAO_DUPLA_INICIO`, `OBSTRUCAO_DUPLA_10S`, `PARADA_OBSTRUCAO_10S` | Medir que o PWM físico passou a 0% |
| Obstrução localizada e recuperação | `Sx_OBSTR90`, `Sx_OK`, `OBSTRUCAO_DUPLA_RECUPEROU` | Registro visual e geometria de obstrução |
| Redução global de PWM | `PWM_ALTERADO`, `pwm_pct`, irradiâncias e estado | Registrar a intervenção e confirmar eletricamente |
| Botões / sinalizações | `BT_SEL_CURTO`, `BT_SEL_2S`, `BT_VOL_CURTO`, `BT_AVA_CURTO` etc. | **Três botões físicos e quatro funções**; tirar fotos da sinalização |
| 50 intervalos de atualização | `ms` consecutivos; `G`/analisador verificam >=50 e estatísticas | Confirmar em ensaio com fonte estável |
| Falha durante controle autorizado | `AJUSTE_AUTORIZADO` seguido de `Sx_AUSENTE`/`PARADA_...` | Observar bloqueio/LEDs na bancada |

## 6. Limitações e cuidados

- Gravação LittleFS aumenta ligeiramente a latência do loop e usa flash interna. Para grandes campanhas de longa duração, prefira cartão SD ou buffer controlado; aqui o objetivo é validação de bancada de curta duração.
- Repetição frequente de escrita desgasta flash; esta versão usa arquivos com rotação e grava somente durante operação (mais alguns registros de falha e eventos), mas não é solução de datalogger contínuo de meses.
- Desligamento inesperado durante escrita pode perder/corromper a **última linha**. Para reduzir o risco, desligue pelos botões antes de cortar a fonte.
- `G` resume a **última operação**. Se executar vários experimentos separados por desligamentos do modo operação, `H` preserva todas as operações **enquanto elas estiverem nas duas janelas de arquivo**; `analisar_historico.py` apresenta cada uma.
- Com o ESP32 retirado da PCB, o código continua tentando ler sensores que não estão conectados. Essa ausência **não invalida o CSV já registrado**; não use `D` para revisar medições antigas.
- Se o TFT mostrar `LOG ERR`, a memória não foi montada ou ocorreu erro na escrita: **não considerar que os dados estejam protegidos**.
- Evite conectar simultaneamente o ESP32 por USB e por outra alimentação sem comprovar prevenção de retorno de corrente e compatibilidade dos GNDs. Se a PCB usa a fonte de 24 V, retire a energia da fonte antes de plugar o USB.

## 7. Validação deste pacote

- A sintaxe C++ passou em `g++ -std=gnu++17 -fsyntax-only` com interfaces Arduino **simuladas**; não equivale a compilar o sketch com as bibliotecas ESP32 reais.
- Os scripts auxiliares Python foram verificados com `py_compile` e o analisador foi exercitado com CSV sintético.
- O sistema de arquivos, serialização, rotação, interação com o loop, intervalos temporais e persistência depois de desligar **ainda precisam ser verificados no ESP32 físico**, antes de coletar dados para a monografia.
