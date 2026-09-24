"""
Abre el lanzador de corridas de P00.

Uso desde la raíz del repositorio.
    python P00_control_lateral/abrir_lanzador.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from lanzador.app import main  # noqa: E402

if __name__ == "__main__":
    main()
