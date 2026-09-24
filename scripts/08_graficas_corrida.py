"""Genera las graficas de una sola corrida del MPC completo, a partir de su
telemetry.csv. No modifica ni ejecuta el controlador, solo lee lo que una
corrida ya guardo.

Cinco figuras, pensadas para mostrar una corrida en la sustentacion.
  Figura_corrida_1_trayectoria.png    posicion del carro contra la trazada
  Figura_corrida_2_error_lateral.png  e_y en el tiempo
  Figura_corrida_3_velocidad.png      velocidad real contra objetivo
  Figura_corrida_4_direccion.png      angulo real contra angulo objetivo
  Figura_corrida_5_convergencia.png   ciclos donde cada MPC no convergio

Uso:
    python scripts/08_graficas_corrida.py <carpeta_de_la_corrida>
    python scripts/08_graficas_corrida.py --ultima

--ultima toma la corrida mas reciente dentro de
Model Predictive Control/Python/runs (recursivo), util para conectarla
directo al boton del lanzador.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def cargar_telemetria(carpeta):
    path_csv = os.path.join(carpeta, "telemetry.csv")
    if not os.path.exists(path_csv):
        raise FileNotFoundError(f"No hay telemetry.csv en {carpeta}")
    with open(path_csv, "r", encoding="utf-8", newline="") as f:
        filas = list(csv.DictReader(f))
    if not filas:
        raise ValueError(f"telemetry.csv esta vacio en {carpeta}")
    columnas = {k: np.array([float(r[k]) for r in filas]) for k in filas[0]}
    return columnas


def cargar_trazada(carpeta):
    path_csv = os.path.join(carpeta, "reference_profile.csv")
    if not os.path.exists(path_csv):
        return None
    with open(path_csv, "r", encoding="utf-8", newline="") as f:
        filas = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in filas]) for k in filas[0]}


def corrida_mas_reciente(runs_dir):
    manifiestos = glob.glob(os.path.join(runs_dir, "**", "manifest.json"), recursive=True)
    if not manifiestos:
        raise FileNotFoundError(f"No se encontro ninguna corrida bajo {runs_dir}")
    mas_reciente = max(manifiestos, key=os.path.getmtime)
    return os.path.dirname(mas_reciente)


def fig_trayectoria(tel, trazada, out_path):
    fig, ax = plt.subplots(figsize=(7.5, 7.0))
    if trazada is not None:
        ax.plot(trazada["x"], trazada["z"], color="#999999", linewidth=1.2,
                label="Trazada de referencia", zorder=2)
    ax.plot(tel["car_x"], tel["car_z"], color="#3b6fa0", linewidth=1.0,
            label="Trayectoria real del carro", zorder=3)
    ax.set_xlabel("x (m)", fontsize=9)
    ax.set_ylabel("z (m)", fontsize=9)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(linestyle=":", color="#999999", linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    ax.legend(fontsize=8.5, frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)


def fig_error_lateral(tel, out_path):
    fig, ax = plt.subplots(figsize=(8.6, 3.6))
    ax.plot(tel["t_s"], tel["e_y_m"], color="#3b6fa0", linewidth=0.9, zorder=3)
    ax.axhline(0.0, color="#999999", linewidth=0.8, zorder=2)
    ax.set_xlabel("Tiempo (s)", fontsize=9)
    ax.set_ylabel("Error lateral e_y (m)", fontsize=9)
    ax.grid(linestyle=":", color="#999999", linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)


def fig_velocidad(tel, out_path):
    fig, ax = plt.subplots(figsize=(8.6, 3.6))
    ax.plot(tel["t_s"], tel["v_real_kmh"], color="#3b6fa0", linewidth=0.9,
            label="Velocidad real", zorder=3)
    ax.plot(tel["t_s"], tel["v_target_kmh"], color="#a8322d", linewidth=0.9,
            linestyle="--", label="Velocidad objetivo", zorder=3)
    ax.set_xlabel("Tiempo (s)", fontsize=9)
    ax.set_ylabel("Velocidad (km/h)", fontsize=9)
    ax.grid(linestyle=":", color="#999999", linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    ax.legend(fontsize=8.5, frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)


def fig_direccion(tel, out_path):
    fig, ax = plt.subplots(figsize=(8.6, 3.6))
    ax.plot(tel["t_s"], tel["theta_new_deg"], color="#3b6fa0", linewidth=0.9,
            label="Angulo real del volante", zorder=3)
    ax.plot(tel["t_s"], tel["theta_ref0_deg"], color="#a8322d", linewidth=0.8,
            linestyle="--", label="Angulo objetivo (theta_ref)", zorder=3)
    ax.set_xlabel("Tiempo (s)", fontsize=9)
    ax.set_ylabel("Angulo de volante (grados)", fontsize=9)
    ax.grid(linestyle=":", color="#999999", linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    ax.legend(fontsize=8.5, frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)


def fig_convergencia(tel, out_path):
    fig, ax = plt.subplots(figsize=(8.6, 2.6))
    fallo_dir = tel["mpc_steer_ok"] < 0.5
    fallo_vel = tel["mpc_speed_ok"] < 0.5
    ax.scatter(tel["t_s"][fallo_dir], np.full(fallo_dir.sum(), 1.0), color="#a8322d",
               s=14, marker="|", label=f"MPC direccion no convergio ({int(fallo_dir.sum())} ciclos)",
               zorder=3)
    ax.scatter(tel["t_s"][fallo_vel], np.full(fallo_vel.sum(), 0.0), color="#d98c2b",
               s=14, marker="|", label=f"MPC velocidad no convergio ({int(fallo_vel.sum())} ciclos)",
               zorder=3)
    ax.set_yticks([0.0, 1.0])
    ax.set_yticklabels(["Velocidad", "Direccion"])
    ax.set_ylim(-0.5, 1.5)
    ax.set_xlabel("Tiempo (s)", fontsize=9)
    ax.grid(axis="x", linestyle=":", color="#999999", linewidth=0.6, zorder=0)
    ax.set_axisbelow(True)
    for lado in ("top", "right"):
        ax.spines[lado].set_visible(False)
    ax.legend(fontsize=8, frameon=False, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=600)
    plt.close(fig)


def main():
    raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("carpeta", nargs="?", help="Carpeta de la corrida (contiene telemetry.csv)")
    ap.add_argument("--ultima", action="store_true",
                    help="Usa la corrida mas reciente bajo Model Predictive Control/Python/runs")
    ap.add_argument("--salida", default=None,
                    help="Carpeta de salida para las figuras. Por defecto, dentro de la propia corrida.")
    args = ap.parse_args()

    if args.ultima:
        runs_dir = os.path.join(raiz, "Model Predictive Control", "Python", "runs")
        carpeta = corrida_mas_reciente(runs_dir)
    elif args.carpeta:
        carpeta = args.carpeta
    else:
        ap.error("indica una carpeta de corrida o usa --ultima")

    if not os.path.isdir(carpeta):
        print(f"No existe la carpeta {carpeta}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.salida or os.path.join(carpeta, "figuras")
    os.makedirs(out_dir, exist_ok=True)

    print(f"Corrida: {carpeta}")
    tel = cargar_telemetria(carpeta)
    trazada = cargar_trazada(carpeta)
    if trazada is None:
        print("  Aviso, no hay reference_profile.csv, la figura de trayectoria "
              "se genera sin la trazada de referencia.")

    figuras = [
        ("Figura_corrida_1_trayectoria.png", lambda p: fig_trayectoria(tel, trazada, p)),
        ("Figura_corrida_2_error_lateral.png", lambda p: fig_error_lateral(tel, p)),
        ("Figura_corrida_3_velocidad.png", lambda p: fig_velocidad(tel, p)),
        ("Figura_corrida_4_direccion.png", lambda p: fig_direccion(tel, p)),
        ("Figura_corrida_5_convergencia.png", lambda p: fig_convergencia(tel, p)),
    ]
    for nombre, fn in figuras:
        out_path = os.path.join(out_dir, nombre)
        fn(out_path)
        print(f"  {out_path}")

    print(f"\n{len(tel['t_s'])} ciclos, {tel['t_s'][-1]:.1f} s de duracion.")
    print(f"Figuras guardadas en: {out_dir}")


if __name__ == "__main__":
    main()
