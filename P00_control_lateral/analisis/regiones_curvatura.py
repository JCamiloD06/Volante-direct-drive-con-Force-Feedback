"""
Cobertura de las regiones de curvatura sobre la trazada, decisión 2.2.

Reproduce en script la auditoría del 2026-09-14. Escribe
results/p00/auditoria/cobertura_regiones.csv y tramos_region_alta.csv.

Uso desde la raíz del repositorio.
    python P00_control_lateral/analisis/regiones_curvatura.py
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from plataforma import procedencia  # noqa: E402
from plataforma.regiones import asignar_region  # noqa: E402
from plataforma.trazada import Trazada  # noqa: E402


def tramos_contiguos(mascara, longitudes, s):
    tramos = []
    n = len(mascara)
    i = 0
    while i < n:
        if mascara[i]:
            j = i
            while j < n and mascara[j]:
                j += 1
            tramos.append((float(s[i]), float(longitudes[i:j].sum())))
            i = j
        else:
            i += 1
    return tramos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(RAIZ_P00 / "configs" / "base.json"))
    args = ap.parse_args()
    with open(args.config, encoding="utf-8") as f:
        cfg = json.load(f)

    ruta_csv = procedencia.RAIZ_REPO / cfg["trazada"]["csv"]
    trazada = Trazada(ruta_csv, cfg["trazada"]["suavizado_curvatura_m"])
    reg = cfg["regiones"]
    regiones = asignar_region(trazada.curvature, reg["radio_baja_m"], reg["radio_alta_m"])
    longitudes = trazada.longitud_segmento
    total = float(longitudes.sum())

    salida = procedencia.RAIZ_REPO / "results" / "p00" / "auditoria"
    salida.mkdir(parents=True, exist_ok=True)

    print(f"Trazada {ruta_csv.name}, {trazada.n} puntos, {total:.1f} m, "
          f"suavizado {cfg['trazada']['suavizado_curvatura_m']} m ({trazada.ventana_curvatura_puntos} puntos)")
    with open(salida / "cobertura_regiones.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["region", "longitud_m", "porcentaje"])
        for nombre in ("baja", "media", "alta"):
            largo = float(longitudes[regiones == nombre].sum())
            w.writerow([nombre, round(largo, 1), round(100.0 * largo / total, 2)])
            print(f"  {nombre:5s} {largo:7.1f} m  {100.0 * largo / total:5.1f} %")

    tramos = tramos_contiguos(regiones == "alta", longitudes, trazada.s)
    with open(salida / "tramos_region_alta.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["s_inicio_m", "longitud_m"])
        for s0, largo in tramos:
            w.writerow([round(s0, 1), round(largo, 1)])
    print(f"  tramos contiguos en region alta {len(tramos)}, longitudes "
          f"{[round(largo) for _, largo in tramos]}")
    print(f"  radio minimo {1.0 / np.abs(trazada.curvature).max():.1f} m")
    print(f"Archivos escritos en {salida}")


if __name__ == "__main__":
    main()
