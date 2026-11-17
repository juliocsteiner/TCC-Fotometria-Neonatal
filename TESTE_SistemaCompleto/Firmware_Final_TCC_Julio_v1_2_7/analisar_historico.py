#!/usr/bin/env python3
"""Analisa o CSV persistente do TCC sem dependencias externas.
Uso: python analisar_historico.py historico_tcc.csv
"""
import argparse
import csv
import statistics
from pathlib import Path


def to_int(value, default=0):
    try:
        return int(value)
    except (ValueError, TypeError):
        return default


def to_float(value, default=None):
    try:
        return float(value)
    except (ValueError, TypeError):
        return default


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', type=Path)
    args = parser.parse_args()
    with args.csv.open('r', encoding='utf-8-sig', newline='') as f:
        linhas = list(csv.DictReader(f, delimiter=';'))
    if not linhas or 'evento' not in linhas[0]:
        print('CSV vazio ou cabecalho nao reconhecido. Exporte novamente com H.')
        return 1

    campanhas = []
    atual = None
    for reg in linhas:
        if reg['tipo'] == 'E' and reg['evento'] == 'OPERACAO_INICIADA':
            atual = {'inicio': reg, 'amostras': [], 'eventos': []}
            campanhas.append(atual)
        if atual is not None:
            if reg['tipo'] == 'M':
                atual['amostras'].append(reg)
            elif reg['tipo'] == 'E':
                atual['eventos'].append(reg)

    saida = []
    def out(msg=''):
        saida.append(str(msg))

    out('RELATORIO DE VALIDACAO — MONITOR DE IRRADIANCIA TCC')
    out(f'Arquivo: {args.csv.name}; registros totais: {len(linhas)}; operacoes iniciadas: {len(campanhas)}')
    out('NOTA: tempo em ms relativo a cada boot; sessao incrementa a cada inicializacao.')
    out('Os status usam 0=OK, 1=AUSENTE/sem comunicacao, 2=FALHA, 3=SAT, 4=OBSTR/90%.')
    out('Fotos do TFT e condicoes fisicas dos ensaios devem ser documentadas separadamente.')

    total_amostras = 0
    botoes_global = set()
    for num, camp in enumerate(campanhas, 1):
        amostras = camp['amostras']
        eventos = camp['eventos']
        total_amostras += len(amostras)
        tipos = [e['evento'] for e in eventos]
        botoes_global.update(e for e in tipos if e.startswith('BT_'))
        seq_normal = max_normal = 0
        seq_int = max_int = 0
        janela50 = []
        ultima = None
        n4 = n3 = nObs = nMeta = 0
        exemplos4 = []
        for reg in amostras:
            nval = to_int(reg['validos'])
            pwm = to_float(reg['pwm_pct'], 0.0)
            obs = to_int(reg['obstruidos'])
            if nval == 4:
                n4 += 1
                if len(exemplos4) < 3:
                    exemplos4.append(f"{reg['ms']} ms: Erep={reg['Erep']}, sensores={reg['S1']}/{reg['S2']}/{reg['S3']}/{reg['S4']}/{reg['S5']}")
            if nval <= 3:
                n3 += 1
            if obs:
                nObs += 1
            if reg['meta8a9'] == '1':
                nMeta += 1
            if nval == 5 and obs == 0 and pwm > 0.1:
                seq_normal += 1
                max_normal = max(max_normal, seq_normal)
            else:
                seq_normal = 0

            if ultima is not None and reg['sessao'] == ultima['sessao']:
                dt = to_int(reg['ms']) - to_int(ultima['ms'])
                if 100 <= dt <= 10000:
                    seq_int += 1
                    max_int = max(max_int, seq_int)
                    janela50.append(dt)
                    janela50 = janela50[-50:]
                else:
                    seq_int = 0
                    janela50 = []
            else:
                seq_int = 0
                janela50 = []
            ultima = reg

        out()
        out(f'OPERAÇÃO {num} | sessao de inicio {camp["inicio"]["sessao"]} | {len(amostras)} aquisicoes')
        out(f'  30 leituras NORMAIS consecutivas: {"SIM" if max_normal >= 30 else "NAO"} (max {max_normal})')
        out(f'  50 intervalos consecutivos: {"SIM" if max_int >= 50 else "NAO"} (max {max_int})')
        out(f'  Registros com 4 sensores validos: {n4}; <=3 validos: {n3}; com obstrucao: {nObs}; na meta 8-9: {nMeta}')
        for v in exemplos4:
            out('    Exemplo 4 pontos: ' + v)
        out(f'  Evento de 2+ obstruidos por 10s: {"SIM" if "PARADA_OBSTRUCAO_10S" in tipos else "NAO"}')
        out(f'  Obstrucao localizada detectada: {"SIM" if any(e.endswith("_OBSTR90") for e in tipos) else "NAO"}')
        out(f'  Recuperacao de sensor documentada: {"SIM" if any(e.endswith("_OK") for e in tipos) else "NAO"}')
        out(f'  Controle autorizado: {"SIM" if "AJUSTE_AUTORIZADO" in tipos else "NAO"}')
        out(f'  Eventos de botoes: {sum(e.startswith("BT_") for e in tipos)}')
        if len(janela50) == 50:
            out(f'  Ultimos 50 intervalos continuos: media={statistics.mean(janela50):.1f} ms; '
                f'desvio={statistics.stdev(janela50):.1f} ms; min={min(janela50)}; max={max(janela50)}')
        else:
            out('  Ultima sequencia nao contem 50 intervalos continuos; outras podem ter ocorrido.')
    out()
    out('RESUMO DE EVIDENCIAS')
    out('  Preparacao da montagem: verificacao manual.')
    out('  Fotografias das duas telas e alarmes: coletar manualmente.')
    out(f'  Operacoes registradas: {len(campanhas)}; amostras armazenadas: {total_amostras}.')
    out('  Quatro FUNCOES de botao: SELECT curto, SELECT longo, VOLTAR e AVANCAR.')
    out('  Eventos de botao encontrados: ' + ', '.join(sorted(botoes_global)) if botoes_global else '  Nenhum evento de botao registrado.')
    out('  Queda global do nivel de luz: exige marcar o instante do ensaio; PWM comandado nao e necessariamente PWM fisico.')
    out('  Para dizer que houve falha DURANTE ajuste autorizado, confira ordem dos eventos em CSV.')
    out('  CSV documenta estimativas; acima de 75% de PWM confirmar valores com THOR.')

    texto = '\n'.join(saida) + '\n'
    destino = args.csv.with_name(args.csv.stem + '_analise.txt')
    destino.write_text(texto, encoding='utf-8')
    print(texto)
    print(f'Relatorio salvo em {destino}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
