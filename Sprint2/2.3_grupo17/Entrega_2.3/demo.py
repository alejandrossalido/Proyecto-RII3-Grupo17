"""Ejecuta las dos demostraciones. Con --sin-ventana también comprueba los resultados."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sin-ventana', action='store_true')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    pasos = [['rectificar.py'], ['detectar_aruco.py']]
    if args.sin_ventana:
        pasos = [p + ['--sin-ventana'] for p in pasos] + [['comprobar.py']]
    try:
        for paso in pasos:
            subprocess.run([sys.executable, str(base / paso[0]), *paso[1:]], check=True)
    except subprocess.CalledProcessError as exc:
        parser.exit(exc.returncode, f'La demostración se ha detenido en {paso[0]}.\n')


if __name__ == '__main__':
    main()
