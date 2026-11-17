#!/usr/bin/env python3
"""Exporta o historico gravado no LittleFS do ESP32 via USB, apos os testes.
Requer: pip install pyserial
Uso: python capturar_log_usb.py COM5  (Windows)
     python capturar_log_usb.py /dev/ttyUSB0 (Linux)
Nao executar com a operacao e os LEDs ativos.
"""
import argparse
import sys
import time
from pathlib import Path


def main():
    pa = argparse.ArgumentParser(description='Recupera CSV historico da flash do ESP32 (firmware TCC v1.2.7)')
    pa.add_argument('porta', help='Porta serial, ex: COM5 ou /dev/ttyUSB0')
    pa.add_argument('--baud', type=int, default=115200)
    pa.add_argument('--saida', default='historico_tcc.csv')
    pa.add_argument('--espera-boot', type=float, default=8.0)
    pa.add_argument('--timeout', type=float, default=180.0)
    args = pa.parse_args()
    try:
        import serial
    except ImportError:
        print('Instale pyserial primeiro: python -m pip install pyserial', file=sys.stderr)
        return 2
    destino = Path(args.saida)
    print(f'Conectando {args.porta} a {args.baud} baud...')
    with serial.Serial(args.porta, args.baud, timeout=0.5) as porta:
        print(f'Aguardando boot e RESET do mux ({args.espera_boot:.0f}s)...')
        time.sleep(args.espera_boot)
        porta.reset_input_buffer()
        porta.write(b'X\n')  # Garantir PWM desligado antes de exportar.
        time.sleep(0.3)
        porta.reset_input_buffer()
        porta.write(b'H\n')
        iniciou = False
        terminou = False
        gravadas = 0
        inicio = time.monotonic()
        with destino.open('w', encoding='utf-8', newline='') as arquivo:
            while time.monotonic() - inicio < args.timeout:
                linha = porta.readline().decode('utf-8', errors='replace').strip()
                if not linha:
                    continue
                if 'INICIO CSV TCC' in linha:
                    iniciou = True
                    continue
                if 'FIM CSV TCC' in linha and iniciou:
                    terminou = True
                    break
                if iniciou:
                    # Logs validos sao CSV com 29 colunas; descarta mensagens Serial incidentais.
                    if linha.count(';') == 28:
                        arquivo.write(linha + '\n')
                        gravadas += 1
        if not terminou:
            print('Exportacao incompleta. Aumente --timeout ou confira a porta.', file=sys.stderr)
            return 1
        print(f'Historico: {destino.resolve()} ({gravadas} linhas)')
        resumo = destino.with_name(destino.stem + '_resumo_serial.txt')
        porta.write(b'G\n')
        texto = []
        inicio = time.monotonic()
        leu = False
        while time.monotonic() - inicio < 25:
            linha = porta.readline().decode('utf-8', errors='replace').rstrip('\r\n')
            if 'RESUMO PERSISTENTE TCC' in linha:
                leu = True
            if leu:
                texto.append(linha)
                if linha.startswith('=====') and 'RESUMO PERSISTENTE TCC' not in linha and len(texto)>1:
                    break
        if texto:
            resumo.write_text('\n'.join(texto)+'\n', encoding='utf-8')
            print(f'Resumo: {resumo.resolve()}')
        else:
            print('Nao foi possivel recuperar resumo automatico; use G no Monitor Serial.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
