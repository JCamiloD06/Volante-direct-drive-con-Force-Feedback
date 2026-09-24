"""
Pruebas del vigilante de abandono automático. No tocan Assetto Corsa.

    python P00_control_lateral/pruebas/prueba_abandono.py
"""
import json
import sys
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from plataforma.abandono import Abandono  # noqa: E402

RESULTADOS = []


def verificar(nombre, ok, detalle=""):
    RESULTADOS.append((nombre, bool(ok)))
    print(f"[{'OK' if ok else 'FALLA'}] {nombre}" + (f" {detalle}" if detalle else ""))


def correr(v, pasos, en_pits=False, ruedas=0, t0=0.0, ts=0.05, vig=None):
    """Alimenta el vigilante con condiciones constantes y devuelve el motivo."""
    vig = vig or Abandono()
    motivo = None
    for k in range(pasos):
        motivo = vig.actualizar(t0 + k * ts, v, en_pits, ruedas)
        if motivo:
            return motivo, vig, k * ts
    return None, vig, pasos * ts


def main():
    # Salida de pits, el vehículo está quieto y el vigilante no debe dispararse.
    motivo, _, _ = correr(0.0, 1000)
    verificar("no abandona antes de arrancar", motivo is None, str(motivo))

    # Arranca y luego se detiene fuera de pits.
    _, vig, _ = correr(120.0, 20)
    motivo, _, t = correr(0.0, 1000, vig=vig, t0=1.0)
    verificar("abandona con el vehículo detenido tras arrancar", motivo is not None, str(motivo))
    verificar("espera los 8 s antes de abandonar", 8.0 < t <= 8.1, f"{t:.2f} s")

    # Detenido pero en pits, no cuenta.
    _, vig, _ = correr(120.0, 20)
    motivo, _, _ = correr(0.0, 1000, en_pits=True, vig=vig, t0=1.0)
    verificar("no abandona con el vehículo en pits", motivo is None, str(motivo))

    # Una parada corta no dispara.
    _, vig, _ = correr(120.0, 20)
    motivo, vig, _ = correr(0.0, 100, vig=vig, t0=1.0)
    verificar("una parada de 5 s no abandona", motivo is None, str(motivo))
    motivo, _, _ = correr(120.0, 200, vig=vig, t0=10.0)
    verificar("el contador se reinicia al volver a rodar", motivo is None, str(motivo))

    # Ruedas fuera sostenidas.
    _, vig, _ = correr(120.0, 20)
    motivo, _, t = correr(120.0, 1000, ruedas=4, vig=vig, t0=1.0)
    verificar("abandona con cuatro ruedas fuera sostenidas", motivo is not None, str(motivo))
    verificar("espera los 3 s de ruedas fuera", 3.0 < t <= 3.1, f"{t:.2f} s")

    # Dos ruedas fuera no llegan al umbral de tres.
    _, vig, _ = correr(120.0, 20)
    motivo, _, _ = correr(120.0, 1000, ruedas=2, vig=vig, t0=1.0)
    verificar("dos ruedas fuera no abandonan", motivo is None, str(motivo))

    # Una corrida normal no abandona nunca.
    _, vig, _ = correr(120.0, 20)
    motivo, _, _ = correr(90.0, 5000, vig=vig, t0=1.0)
    verificar("una vuelta normal no abandona", motivo is None, str(motivo))

    # Los valores vienen de base.json y no de los de respaldo.
    cfg = json.load(open(RAIZ_P00 / "configs" / "base.json", encoding="utf-8"))
    verificar("base.json trae la sección abandono", "abandono" in cfg)
    vig = Abandono(cfg.get("abandono"))
    verificar("toma los valores de base.json",
              vig.t_quieto_max == cfg["abandono"]["tiempo_quieto_s"], str(vig.t_quieto_max))

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} verificaciones correctas")
    if fallas:
        print("Fallas:", fallas)
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
