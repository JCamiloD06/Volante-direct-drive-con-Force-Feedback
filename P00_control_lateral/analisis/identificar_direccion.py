"""
Identificación dedicada de la constante de la cadena de dirección.

La constante rueda_rad_eje_completo son los radianes de rueda directriz que el
vehículo toma con el eje de vJoy en su extremo. Se identifica con la relación
cinemática δ igual a atan(L r sobre v), donde r es la tasa de guiñada y L la
batalla medida con los puntos de contacto, ajustada contra el comando
normalizado enviado a vJoy.

Filtros. Ciclos con contactos válidos, sin saturación de magnitud ni de tasa,
sin dirección retenida, con velocidad dentro de un rango, aceleración lateral
baja para que el modelo cinemático valga, comando por encima de un mínimo y
ángulo de deriva pequeño.

El signo del eje de guiñada local de Assetto Corsa no está verificado, así que
se reporta el signo de la pendiente por separado y se usa su valor absoluto.

El intervalo de confianza sale de remuestrear bloques de ciclos consecutivos,
no ciclos sueltos, porque los ciclos vecinos no son independientes.

Uso desde la raíz del repositorio.
    python P00_control_lateral/analisis/identificar_direccion.py data/raw/p00/corridas/<id> [...]
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


def _columnas(carpeta, nombres):
    datos = {n: [] for n in nombres}
    with open(Path(carpeta) / "telemetria.csv", newline="", encoding="utf-8") as f:
        for fila in csv.DictReader(f):
            for n in nombres:
                v = fila.get(n, "")
                if v in ("True", "False"):
                    datos[n].append(1.0 if v == "True" else 0.0)
                else:
                    try:
                        datos[n].append(float(v))
                    except (TypeError, ValueError):
                        datos[n].append(np.nan)
    return {n: np.array(v) for n, v in datos.items()}


def _ajuste_por_origen(x, y):
    pendiente = float(np.sum(x * y) / np.sum(x * x))
    residuo = y - pendiente * x
    r2 = 1.0 - float(np.sum(residuo ** 2) / np.sum((y - y.mean()) ** 2))
    return pendiente, r2


def _intervalo_bloques(x, y, tam_bloque=100, repeticiones=2000, semilla=20260915):
    rng = np.random.default_rng(semilla)
    n = len(x)
    inicios = np.arange(0, n - tam_bloque + 1, tam_bloque)
    if len(inicios) < 3:
        return None, None
    pendientes = []
    for _ in range(repeticiones):
        elegidos = rng.choice(inicios, size=len(inicios), replace=True)
        idx = np.concatenate([np.arange(i, i + tam_bloque) for i in elegidos])
        pendientes.append(np.sum(x[idx] * y[idx]) / np.sum(x[idx] * x[idx]))
    return float(np.percentile(pendientes, 2.5)), float(np.percentile(pendientes, 97.5))


def procesar(carpeta, args):
    cols = _columnas(carpeta, ["v_kmh", "acc_g_y", "steer_norm", "tasa_guinada_local", "L_usada_m",
                               "L_medida_m", "contactos_ok", "sat_magnitud", "sat_tasa",
                               "direccion_retenida", "deriva_rad"])
    v_ms = cols["v_kmh"] / 3.6
    m = ((cols["v_kmh"] >= args.v_min) & (cols["v_kmh"] <= args.v_max)
         & (np.abs(cols["acc_g_y"]) <= args.ay_max)
         & (np.abs(cols["steer_norm"]) >= args.cmd_min)
         & (cols["contactos_ok"] > 0.5)
         & (cols["sat_magnitud"] < 0.5) & (cols["sat_tasa"] < 0.5)
         & np.isfinite(cols["tasa_guinada_local"]) & np.isfinite(cols["L_usada_m"])
         & (np.abs(np.degrees(cols["deriva_rad"])) <= args.deriva_max))
    retenida = cols["direccion_retenida"]
    if np.isfinite(retenida).any():
        m &= ~(retenida > 0.5)
    resultado = {"carpeta": str(carpeta), "ciclos_utiles": int(m.sum())}
    if m.sum() < args.min_ciclos:
        resultado["aviso"] = f"menos de {args.min_ciclos} ciclos útiles"
        return resultado
    x = cols["steer_norm"][m]
    y = np.arctan(cols["L_usada_m"][m] * cols["tasa_guinada_local"][m] / v_ms[m])
    pendiente, r2 = _ajuste_por_origen(x, y)
    bajo, alto = _intervalo_bloques(x, y, args.bloque)
    resultado.update({
        "pendiente": pendiente,
        "rueda_rad_eje_completo": abs(pendiente),
        "signo_pendiente": "+" if pendiente > 0 else "-",
        "r2": r2,
        "ic95": None if bajo is None else [abs(alto), abs(bajo)] if pendiente < 0 else [bajo, alto],
        "L_mediana_m": float(np.nanmedian(cols["L_medida_m"])),
        "v_kmh_mediana": float(np.median(cols["v_kmh"][m])),
    })
    return resultado


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corridas", nargs="+")
    ap.add_argument("--config", default=str(RAIZ_P00 / "configs" / "base.json"))
    ap.add_argument("--v-min", type=float, default=40.0)
    ap.add_argument("--v-max", type=float, default=120.0)
    ap.add_argument("--ay-max", type=float, default=0.2, help="aceleración lateral máxima en g")
    ap.add_argument("--cmd-min", type=float, default=0.05)
    ap.add_argument("--deriva-max", type=float, default=3.0, help="grados")
    ap.add_argument("--bloque", type=int, default=100)
    ap.add_argument("--min-ciclos", type=int, default=200)
    args = ap.parse_args()
    with open(args.config, encoding="utf-8") as f:
        cfg = json.load(f)
    configurado = cfg["direccion"]["rueda_rad_eje_completo"]

    salida = {"configurado": configurado, "filtros": vars(args), "corridas": []}
    print(f"Constante configurada {configurado} rad")
    for carpeta in args.corridas:
        r = procesar(Path(carpeta), args)
        salida["corridas"].append(r)
        if "rueda_rad_eje_completo" not in r:
            print(f"  {Path(carpeta).name}: {r.get('aviso')} ({r['ciclos_utiles']} ciclos)")
            continue
        ic = r["ic95"]
        dif = (r["rueda_rad_eje_completo"] - configurado) / configurado * 100.0
        print(f"  {Path(carpeta).name}: {r['rueda_rad_eje_completo']:.4f} rad "
              f"{'' if ic is None else f'IC95 {ic[0]:.4f} a {ic[1]:.4f} '}"
              f"R² {r['r2']:.3f} signo {r['signo_pendiente']} n={r['ciclos_utiles']} "
              f"diferencia {dif:+.1f} %")

    validas = [r for r in salida["corridas"] if "rueda_rad_eje_completo" in r]
    if len(validas) > 1:
        valores = [r["rueda_rad_eje_completo"] for r in validas]
        salida["mediana_entre_corridas"] = float(np.median(valores))
        salida["dispersion_entre_corridas"] = float(np.max(valores) - np.min(valores))
        print(f"  mediana entre corridas {salida['mediana_entre_corridas']:.4f} rad, "
              f"dispersión {salida['dispersion_entre_corridas']:.4f} rad")
    destino = procedencia.RAIZ_REPO / "results" / "p00" / "auditoria"
    destino.mkdir(parents=True, exist_ok=True)
    with open(destino / "identificacion_direccion.json", "w", encoding="utf-8") as f:
        json.dump(salida, f, indent=2, ensure_ascii=False)
    print(f"Escrito en {destino / 'identificacion_direccion.json'}")


if __name__ == "__main__":
    main()
