"""
Figuras de la documentación del software para el registro ante la DNDA.

Código auxiliar nuevo, no modifica ningún script existente. Los diagramas
describen la arquitectura tal como está en el código de P00_control_lateral.
La figura de la trazada se calcula con las mismas clases de la plataforma,
Trazada y asignar_region, sobre el archivo de trazada y la configuración base.

Salida en documentacion/figuras/ a 600 dpi, en PNG y TIFF.

Uso desde la raíz del repositorio.
    python P00_control_lateral/documentacion/generar_figuras_doc.py
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
RAIZ_REPO = RAIZ_P00.parent
sys.path.insert(0, str(RAIZ_P00))

from plataforma.regiones import asignar_region  # noqa: E402
from plataforma.trazada import Trazada  # noqa: E402

SALIDA = Path(__file__).resolve().parent / "figuras"
DPI = 600

AZUL = "#003E7E"
AZUL_CLARO = "#DCE8F5"
VERDE_CLARO = "#E3F1E6"
GRIS_CLARO = "#EEEEEE"
NARANJA_CLARO = "#FBE8D3"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9})


def guardar(fig, nombre):
    SALIDA.mkdir(parents=True, exist_ok=True)
    fig.savefig(SALIDA / f"{nombre}.png", dpi=DPI, bbox_inches="tight", facecolor="white")
    fig.savefig(SALIDA / f"{nombre}.tif", dpi=DPI, bbox_inches="tight", facecolor="white",
                pil_kwargs={"compression": "tiff_lzw"})
    plt.close(fig)
    print(f"{nombre} escrita")


def caja(ax, x, y, w, h, texto, color, borde=AZUL, tam=8.5, negrita=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                linewidth=1.0, edgecolor=borde, facecolor=color))
    ax.text(x + w / 2, y + h / 2, texto, ha="center", va="center", fontsize=tam,
            fontweight="bold" if negrita else "normal", wrap=True)


def flecha(ax, x0, y0, x1, y1, texto=None, doble=False, dx_txt=0.0, dy_txt=0.12):
    estilo = "<|-|>" if doble else "-|>"
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=estilo, mutation_scale=10,
                                 linewidth=1.0, color="#333333"))
    if texto:
        ax.text((x0 + x1) / 2 + dx_txt, (y0 + y1) / 2 + dy_txt, texto, ha="center", va="bottom",
                fontsize=7.2, color="#333333")


def figura_arquitectura():
    fig, ax = plt.subplots(figsize=(7.4, 5.6))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 7.6)
    ax.axis("off")
    t = 7.4

    caja(ax, 0.2, 6.3, 4.0, 1.1, "Lanzador\nabrir_lanzador.py, lanzador/app.py\nventana tkinter", AZUL_CLARO, tam=t)
    caja(ax, 0.2, 4.4, 2.7, 1.2, "Planes y corridas\nplan.py, plan_sintonia.py,\nplan_piloto.py, corridas.py",
         AZUL_CLARO, tam=t)
    caja(ax, 5.6, 6.3, 4.2, 1.1, "Preparación del simulador\nplataforma/juego_ac.py\nplantilla, respaldo, clic, prueba vJoy",
         AZUL_CLARO, tam=t)

    caja(ax, 0.2, 1.75, 6.0, 1.95, "", VERDE_CLARO)
    ax.text(3.2, 3.42, "Proceso de corrida, ejecutar_corrida.py", ha="center", fontsize=t + 0.4)
    caja(ax, 0.35, 1.9, 1.8, 1.25, "plataforma/\nmemoria, trazada,\nperfil, PID", "white", tam=t - 0.4)
    caja(ax, 2.3, 1.9, 1.8, 1.25, "controladores/\nPure Pursuit,\nStanley, MPC", "white", tam=t - 0.4)
    caja(ax, 4.25, 1.9, 1.8, 1.25, "plataforma/\nactuador, vuelta,\nabandono, registro", "white", tam=t - 0.4)

    caja(ax, 7.2, 2.9, 2.6, 0.9, "Assetto Corsa\nmemoria compartida", GRIS_CLARO, borde="#666666", tam=t)
    caja(ax, 7.2, 1.5, 2.6, 0.8, "vJoy\ndispositivo virtual", GRIS_CLARO, borde="#666666", tam=t)

    caja(ax, 0.2, 0.1, 4.6, 0.95, "Carpeta de corrida\ntelemetria.csv, perfil.csv, manifiesto.json", NARANJA_CLARO,
         borde="#9A5B13", tam=t)
    caja(ax, 5.4, 0.1, 4.4, 0.95, "analisis/\nmétricas, mapa de campaña", NARANJA_CLARO, borde="#9A5B13", tam=t)

    flecha(ax, 1.2, 6.3, 1.2, 5.6)
    ax.text(1.3, 5.9, "lee y actualiza", fontsize=7, va="center")
    flecha(ax, 4.2, 6.85, 5.6, 6.85)
    ax.text(4.9, 6.95, "prepara", fontsize=7, ha="center")
    flecha(ax, 3.6, 6.3, 3.6, 3.7, doble=True)
    ax.text(3.7, 5.2, "lanza el proceso\ny recibe su salida", fontsize=7, va="center")
    flecha(ax, 8.5, 6.3, 8.5, 3.8)
    ax.text(8.6, 5.0, "abre y deja\nla sesión en vivo", fontsize=7, va="center")
    flecha(ax, 7.2, 3.35, 6.2, 3.35)
    ax.text(6.7, 3.95, "estado\ncada 50 ms", fontsize=7, ha="center", va="center")
    flecha(ax, 6.2, 1.9, 7.2, 1.9)
    ax.text(6.7, 1.3, "dirección,\ngas, freno", fontsize=7, ha="center", va="center")
    flecha(ax, 8.5, 2.3, 8.5, 2.9)
    flecha(ax, 2.5, 1.75, 2.5, 1.05)
    ax.text(2.6, 1.4, "al terminar", fontsize=7, va="center")
    flecha(ax, 4.8, 0.575, 5.4, 0.575)
    guardar(fig, "fig_arquitectura")


def figura_ciclo():
    pasos = [
        "Esperar paquete gráfico nuevo\n(packetId distinto)",
        "Leer física y construir estado\nposición, ψ, ejes, velocidad",
        "Proyectar posición, eje trasero\ny eje delantero sobre la trazada",
        "Velocidad objetivo del perfil\ny comando PID con conformador",
        "Ley lateral del controlador\nelegido, δ pedido",
        "Entrada progresiva, límites de\nmagnitud y tasa, δ aplicado",
        "Enviar a vJoy dirección,\nacelerador y freno",
        "Actualizar vuelta, registrar fila,\nrevisar abandono y detención",
        "Dormir hasta completar\nel periodo de 50 ms",
    ]
    fig, ax = plt.subplots(figsize=(7.4, 4.8))
    ax.set_xlim(0, 10.1)
    ax.set_ylim(0.6, 6.5)
    ax.axis("off")
    pos = [(0.2, 4.6), (3.55, 4.6), (6.9, 4.6), (6.9, 2.7), (3.55, 2.7), (0.2, 2.7), (0.2, 0.8), (3.55, 0.8),
           (6.9, 0.8)]
    w, h = 2.9, 1.15
    for i, ((x, y), texto) in enumerate(zip(pos, pasos)):
        color = VERDE_CLARO if i in (4,) else AZUL_CLARO
        caja(ax, x, y, w, h, f"{i + 1}. {texto}", color, tam=6.9)
    conexiones = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8)]
    for a, b in conexiones:
        (xa, ya), (xb, yb) = pos[a], pos[b]
        if ya == yb:
            if xb > xa:
                flecha(ax, xa + w, ya + h / 2, xb, yb + h / 2)
            else:
                flecha(ax, xa, ya + h / 2, xb + w, yb + h / 2)
        else:
            flecha(ax, xa + w / 2, ya, xb + w / 2, yb + h)
    x9, y9 = pos[8]
    ax.plot([x9 + w, 9.93, 9.93, 1.65], [y9 + h / 2, y9 + h / 2, 6.1, 6.1], color="#777777", lw=0.9,
            linestyle="--")
    flecha(ax, 1.65, 6.1, 1.65, 5.75)
    ax.text(5.8, 6.17, "siguiente ciclo", fontsize=7.2, color="#555555", ha="center")
    guardar(fig, "fig_ciclo_control")


def figura_preparacion():
    pasos = [
        "Pulsar Iniciar corrida",
        "¿Juego abierto?\nsi hay reinicio, cerrar",
        "Comparar plantilla con cfg\nrespaldar y escribir si difiere",
        "Abrir acs.exe con\nsteam_appid.txt",
        "Esperar sesión en vivo\nMonza y Giulietta QV",
        "Comprobar carro en la\nposición de salida",
        "Clic en el volante del menú\ny prueba de freno por vJoy",
        "Lanzar ejecutar_corrida.py\ncon la información de sesión",
    ]
    fig, ax = plt.subplots(figsize=(7.4, 2.9))
    ax.set_xlim(-0.05, 10.1)
    ax.set_ylim(0, 3.6)
    ax.axis("off")
    w, h = 2.3, 1.05
    xs = [0.05, 2.6, 5.15, 7.7]
    for i, texto in enumerate(pasos):
        fila = 0 if i < 4 else 1
        col = i if i < 4 else 7 - i
        x = xs[col]
        y = 2.3 if fila == 0 else 0.3
        caja(ax, x, y, w, h, f"{i + 1}. {texto}", VERDE_CLARO if i == 7 else AZUL_CLARO, tam=5.9)
    for i in range(3):
        flecha(ax, xs[i] + w, 2.3 + h / 2, xs[i + 1], 2.3 + h / 2)
    flecha(ax, xs[3] + w / 2, 2.3, xs[3] + w / 2, 0.3 + h)
    for i in range(3, 0, -1):
        flecha(ax, xs[i], 0.3 + h / 2, xs[i - 1] + w, 0.3 + h / 2)
    guardar(fig, "fig_preparacion_ac")


def figura_trazada():
    with open(RAIZ_P00 / "configs" / "base.json", encoding="utf-8") as f:
        cfg = json.load(f)
    t = Trazada(RAIZ_REPO / cfg["trazada"]["csv"], cfg["trazada"]["suavizado_curvatura_m"])
    reg = asignar_region(t.curvature, cfg["regiones"]["radio_baja_m"], cfg["regiones"]["radio_alta_m"])
    colores = {"baja": "#9DB7D5", "media": "#E0A33B", "alta": "#B2262B"}
    etiquetas = {"baja": f"baja, radio mayor a {cfg['regiones']['radio_baja_m']:g} m",
                 "media": "media, radio entre 100 y 500 m",
                 "alta": f"alta, radio igual o menor a {cfg['regiones']['radio_alta_m']:g} m"}
    fig, ax = plt.subplots(figsize=(6.2, 4.6))
    for nombre in ("baja", "media", "alta"):
        m = reg == nombre
        pct = 100.0 * np.sum(t.longitud_segmento[m]) / np.sum(t.longitud_segmento)
        ax.scatter(t.x[m], t.z[m], s=2.2, color=colores[nombre], label=f"{etiquetas[nombre]} ({pct:.1f} %)",
                   linewidths=0)
    ax.plot(t.x[0], t.z[0], marker="o", color="black", ms=4)
    ax.annotate("inicio de la trazada", (t.x[0], t.z[0]), xytext=(12, 8), textcoords="offset points", fontsize=7.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x en el marco de Assetto Corsa (m)")
    ax.set_ylabel("z en el marco de Assetto Corsa (m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), fontsize=7.2, markerscale=4, frameon=False)
    ax.grid(True, linewidth=0.3, alpha=0.5)
    guardar(fig, "fig_trazada_regiones")
    return {n: float(100.0 * np.sum(t.longitud_segmento[reg == n]) / np.sum(t.longitud_segmento))
            for n in ("baja", "media", "alta")} | {"n_puntos": int(t.n), "longitud_m": float(t.total_length),
                                                   "espaciado_m": float(t.avg_spacing),
                                                   "ventana_puntos": int(t.ventana_curvatura_puntos)}


if __name__ == "__main__":
    figura_arquitectura()
    figura_ciclo()
    figura_preparacion()
    datos = figura_trazada()
    with open(SALIDA / "datos_trazada.json", "w", encoding="utf-8") as f:
        json.dump(datos, f, indent=2)
    print(json.dumps(datos, indent=2))
