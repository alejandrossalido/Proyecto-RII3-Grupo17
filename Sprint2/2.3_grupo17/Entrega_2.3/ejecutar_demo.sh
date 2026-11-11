#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -x .venv/bin/python ]]; then
    echo 'Crea primero el entorno .venv siguiendo el README.md.' >&2
    exit 1
fi
# Evita que módulos externos de la sesión reemplacen las dependencias del entorno.
exec env -u PYTHONPATH .venv/bin/python demo.py "$@"
