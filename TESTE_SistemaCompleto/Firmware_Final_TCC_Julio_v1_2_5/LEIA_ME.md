# Firmware TCC v1.2.5 — nova identificação operacional de obstrução

Projeto de bancada, **não destinado ao uso clínico**. Base: firmware v1.2.4. Foram preservados os botões, diagnósticos I²C, calibração THOR do plano a 30 cm, controle autorizado (meta 8–9 µW/cm²/nm), PWM até 100% e interface TFT com sobrescrita apenas dos valores.

## Nova regra de obstrução (pedido do projeto)

**A antiga comparação com o perfil esperado da posição do sensor foi removida.** A posição no plano e a diferença espacial de iluminação, por si sós, não constituem motivo para alerta.

Um sensor só é sinalizado operacionalmente como obstruído se:

1. **Não responder/for identificado como ausente no barramento I²C:** a tela mostra `OBSTR/COM` e o Serial conserva o motivo `AUSENTE`. A ausência de ACK **não prova** que o cabo está fisicamente desconectado: também pode ser falha no multiplexador, alimentação ou comunicação.
2. **Sua irradiância estimada diferir em 90% ou mais da irradiância estimada do OUTRO sensor com valor mais próximo:** a tela mostra `OBSTR/90%`. A referência é escolhida **pelo valor medido**, não pela posição física no retângulo.

Cálculo da diferença, após encontrar o sensor válido com irradiância mais próxima:

```text
D(%) = 100 * |E_sensor - E_mais_proximo| / max(E_sensor, E_mais_proximo)
OBSTR/90% se D >= 90%
```

Como a normalização usa o **maior dos dois valores**, a diferença é simétrica: S3 = 15 e bordas = 2,0 µW/cm²/nm apresentam discrepância de **86,7%** e **não** são excluídos. S1 = 1,0 e outros sensores = 10,0 µW/cm²/nm apresentam exatamente **90%** e S1 será sinalizado.

### Proteção para pouca luz e zero

Quando ambos os valores comparados estão abaixo de **0,50 µW/cm²/nm**, o firmware **não aplica o teste de discrepância percentual**. Isso evita tratar ruído de escuro e pequenas leituras perto de zero como supostas obstruções. Se o sensor responde normalmente e tem irradiância 0, sua leitura permanece `0.00` com status `OK`.

A regra de diferença vale em qualquer PWM. Entretanto, **acima de 75% os coeficientes de calibração estão extrapolados**: um alerta `OBSTR/90%` nessa condição é apenas indicativo e deve ser confrontado com o instrumento THOR.

As decisões são recalculadas **a cada aquisição**, sem manter exclusões pelo histórico do sensor: se a discrepância desaparece, ele volta imediatamente à condição válida. O Serial registra transições e o comando `D` exibe o sensor de referência e a discrepância percentual.

**Outros problemas não são disfarçados de obstrução:** falha de leitura (`FALHA`) e saturação (`SAT`) mantêm seus próprios estados e não entram na média espacial. Uma medição 0.00 é diferente de um sensor que deixou de responder (`NA`).

## Resultado esperado em exemplos simulados

| Irradiâncias individuais (µW/cm²/nm) | Sensores válidos | Comportamento |
|---|---:|---|
| 2,0 / 1,9 / 15,0 / 1,8 / 2,1 | 5/5 | Assimetria espacial aceita; centro não é obstruído |
| 1,0 / 10 / 10 / 10 / 10 | 4/5 | S1 = `OBSTR/90%` |
| 20 / 1 / 1 / 1 / 1 | 4/5 | S1 = `OBSTR/90%` (95%) |
| 0 / 0 / 0 / 0 / 0 | 5/5 | Todos `0.00 OK` |
| 0 / 0,10 / 0,18 / 0,02 / 0,03 | 5/5 | Não exclui ruído próximo do escuro |
| Sensor S3 sem comunicação, demais respondendo | 4/5 | S3 = `OBSTR/COM`; valor da S3 é `NA` |

O controle continua exigindo pelo menos **4 sensores válidos**. Uma grande diferença espacial menor que 90% **não** impede o controle. Quando a diferença atinge 90% ela é excluída da média pela regra solicitada; essa heurística não substitui avaliação metrológica da distribuição espacial.

## Instruções para instalação e diagnóstico

1. Extraia o ZIP e abra **`Firmware_Final_TCC_Julio_v1_2_5.ino`** na Arduino IDE. Necessário ESP32 Arduino core 3.x, Adafruit GFX, Adafruit ILI9341, Adafruit AS7341, SparkFun AS7343.
2. O painel usa os mesmos pinos: I²C SDA17/SCL16, RESET via PC817 GPIO19, PWM32, Selecionar18, Voltar21, Avançar22.
3. Mantenha os LEDs desligados na primeira inicialização. O firmware executa o RESET do MUX como na v1.2.4 e faz leituras também no estado `OFF`.
4. Acesse a **segunda página** com AVANÇAR para visualizar `OK`, `OBSTR/COM`, `OBSTR/90%`, `SAT` ou `FALHA` por sensor.
5. Abra o Monitor Serial em **115200 baud** e envie **`D`**: veja o valor de cada sensor, o vizinho de irradiância mais próxima e a divergência percentual. O comando `T` verifica MUX e ACK; `R`, em OFF, reinicia o MUX e os sensores.
6. Para iniciar a operação, mantenha **SELECIONAR pressionado por 2 s**. Durante monitoramento fora da meta, **dois toques curtos em SELECIONAR** autorizam o ajuste automático. Segurar VOLTAR durante controle cancela o ajuste mantendo o PWM.

### Verificações realizadas

Compilação de sintaxe **C++17 com bibliotecas simuladas**, não uma compilação completa com as bibliotecas reais do Arduino. Foram exercitados os cenários da tabela, um caso de saturação, recuperação de sensor e a lógica dos três botões (toque curto, SELECT longo, navegação, autorização/cancelamento). **Teste físico na PCB e medição com THOR continuam necessários.**

### Nota de engenharia

Chamar uma falha de comunicação de “obstrução” segue a convenção de interface solicitada; tecnicamente, uma obstrução óptica real não necessariamente produz falta de ACK I²C, e uma diferença entre irradiâncias pode ocorrer por geometria, não por obstrução. O critério de 90% é experimental e não foi validado como mecanismo de segurança clínica.
