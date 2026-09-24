"""
Procedencia de las piezas copiadas y huellas de archivos.

Las clases de lectura de Assetto Corsa, trazada, perfil de velocidad, PID,
conformador de pedal y salida a vJoy se copiaron el 2026-09-15 desde
mpc_monza_Completo_barrido.py. Si ese archivo cambia después, este módulo lo
detecta al comparar su huella actual con la registrada al copiar, y el
manifiesto de cada corrida deja constancia.
"""
import hashlib
from pathlib import Path

RAIZ_REPO = Path(__file__).resolve().parents[2]
RAIZ_P00 = Path(__file__).resolve().parents[1]

ORIGEN_BARRIDO = RAIZ_REPO / "Model Predictive Control" / "Python" / "mpc_monza_Completo_barrido.py"
SHA256_BARRIDO_AL_COPIAR = "ce6cb785d767d27e5152f415a58d2f957ae2f0123d58c8a978bf5f6447f2ab08"

TRAZADA_MONZA = RAIZ_REPO / "Model Predictive Control" / "Python" / "monza_fast_lane.csv"
SHA256_TRAZADA_AL_COPIAR = "90c6e5a1955e320c0d00f49b490f1adc3abb0af3ac70144f7e7478f750670915"


def sha256(ruta):
    """Huella SHA256 de un archivo, o None si no se puede leer."""
    try:
        h = hashlib.sha256()
        with open(ruta, "rb") as f:
            for bloque in iter(lambda: f.read(65536), b""):
                h.update(bloque)
        return h.hexdigest()
    except OSError:
        return None


def estado_origen():
    """Compara las huellas actuales de los archivos de origen con las registradas."""
    actual_barrido = sha256(ORIGEN_BARRIDO)
    actual_trazada = sha256(TRAZADA_MONZA)
    return {
        "barrido_sha256_al_copiar": SHA256_BARRIDO_AL_COPIAR,
        "barrido_sha256_actual": actual_barrido,
        "barrido_sin_cambios": actual_barrido == SHA256_BARRIDO_AL_COPIAR,
        "trazada_sha256_al_copiar": SHA256_TRAZADA_AL_COPIAR,
        "trazada_sha256_actual": actual_trazada,
        "trazada_sin_cambios": actual_trazada == SHA256_TRAZADA_AL_COPIAR,
    }


def huellas_codigo_p00():
    """Huella de cada archivo de código de P00, para el manifiesto."""
    huellas = {}
    for ruta in sorted(RAIZ_P00.rglob("*.py")):
        if "__pycache__" in ruta.parts:
            continue
        huellas[str(ruta.relative_to(RAIZ_P00)).replace("\\", "/")] = sha256(ruta)
    return huellas
