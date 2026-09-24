"""
Tablas de resultados de P00, generadas por código.

Regla del proyecto, sección 11 de AGENTS.md. Ningún valor se escribe a mano.
Cada tabla lee su fuente en disco. Si la fuente no existe, la tabla se salta y
se informa, nunca se rellena con supuestos.

Salida en tables/p00/, un CSV por tabla más tablas_p00.md con todas en
Markdown, listo para pegar en el manuscrito.

Uso desde la raíz del repositorio.
    python scripts/17_p00_tablas.py
"""
import argparse
import csv
import json
import statistics as est
import sys
from pathlib import Path

RAIZ_REPO = Path(__file__).resolve().parents[1]
RAIZ_P00 = RAIZ_REPO / "P00_control_lateral"
sys.path.insert(0, str(RAIZ_P00))

CONFIG = RAIZ_P00 / "configs" / "base.json"
RANGOS = RAIZ_P00 / "configs" / "rangos_sintonia.json"
SEL_SINTONIA = RAIZ_P00 / "sintonia_fase4" / "seleccion_sintonia.json"
REPETIBILIDAD = RAIZ_P00 / "piloto_fase5" / "repetibilidad_piloto.json"
MAPA_CAMPANA = RAIZ_P00 / "resultados_campana" / "mapa_campana.json"
CORRIDAS = RAIZ_REPO / "data" / "raw" / "p00" / "corridas"
SALIDA = RAIZ_REPO / "tables" / "p00"

NOMBRE_CTRL = {"pure_pursuit": "Pure Pursuit", "stanley": "Stanley", "mpc_cinematico": "MPC cinemático"}
ORDEN_CTRL = ("pure_pursuit", "stanley", "mpc_cinematico")
ORDEN_PERFIL = ("conservador", "nominal", "rapido")
ORDEN_REGION = ("baja", "media", "alta")


def cargar_json(ruta):
    ruta = Path(ruta)
    if not ruta.exists():
        return None
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def relativa(ruta):
    try:
        return Path(ruta).relative_to(RAIZ_REPO)
    except ValueError:
        return Path(ruta)


class Tabla:
    def __init__(self, numero, nombre, titulo, columnas):
        self.numero = numero
        self.nombre = nombre
        self.titulo = titulo
        self.columnas = columnas
        self.filas = []

    def fila(self, *valores):
        self.filas.append([("" if v is None else v) for v in valores])

    def escribir(self):
        SALIDA.mkdir(parents=True, exist_ok=True)
        ruta = SALIDA / f"Tabla_{self.numero}_{self.nombre}.csv"
        with open(ruta, "w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.columnas)
            w.writerows(self.filas)
        print(f"  Tabla {self.numero} escrita en {relativa(ruta)}, {len(self.filas)} filas")
        return ruta

    def markdown(self):
        lineas = [f"Tabla {self.numero}. {self.titulo}", "",
                  "| " + " | ".join(self.columnas) + " |",
                  "|" + "|".join(["---"] * len(self.columnas)) + "|"]
        for fila in self.filas:
            lineas.append("| " + " | ".join(str(v) for v in fila) + " |")
        return "\n".join(lineas) + "\n"


def saltar(numero, motivo):
    print(f"  Tabla {numero} SALTADA, {motivo}")
    return None


# ---------------------------------------------------------------- tabla 4
def tabla_parametros():
    """Parámetros congelados de los tres controladores y de la rama longitudinal."""
    cfg = cargar_json(CONFIG)
    if cfg is None:
        return saltar(4, f"no existe {relativa(CONFIG)}")
    t = Tabla(4, "parametros_congelados",
              "Parámetros adoptados para la campaña, congelados antes de ejecutarla.",
              ["Bloque", "Parámetro", "Valor", "Origen"])
    ctrl = cfg["controladores"]
    origen_sint = "tanda de sintonía, fase 4"
    for clave, etiqueta in (("L0_m", "L0 en m"), ("kv_s", "kv en s")):
        t.fila("Pure Pursuit", etiqueta, f"{ctrl['pure_pursuit'][clave]:.6g}", origen_sint)
    t.fila("Stanley", "k", f"{ctrl['stanley']['k']:.6g}", origen_sint)
    t.fila("Stanley", "epsilon en m/s", f"{ctrl['stanley']['epsilon_ms']:.6g}", "fijado, evita dividir por cero")
    mpc = ctrl["mpc_cinematico"]
    t.fila("MPC cinemático", "N, pasos del horizonte", f"{mpc['N']:g}", "decisión de implementación")
    for clave, etiqueta in (("Qy", "Q del error lateral"), ("Qpsi", "Q del error de rumbo"),
                            ("Rdelta", "R del ángulo"), ("Rddelta", "R de la tasa")):
        origen = "normalizado a 1" if clave == "Qy" else origen_sint
        t.fila("MPC cinemático", etiqueta, f"{mpc[clave]:.6g}", origen)
    lo = cfg["longitudinal"]
    for clave, etiqueta in (("kp", "kp"), ("ki", "ki"), ("kd", "kd"), ("limite_integral", "límite integral")):
        t.fila("PID longitudinal", etiqueta, f"{lo[clave]:.6g}", "regla SIMC sobre planta identificada")
    t.fila("Sesión", "grip_usage_factor", f"{cfg['perfil']['grip_usage_factor']:g}", "aceptado en el piloto, fase 5")
    t.fila("Sesión", "tiempo límite en s", f"{cfg['vuelta']['tiempo_limite_s']:g}", "doble de la mediana del perfil más lento")
    t.fila("Regiones", "radio de curvatura baja en m", f"{cfg['regiones']['radio_baja_m']:g}", "decisión 2.2")
    t.fila("Regiones", "radio de curvatura alta en m", f"{cfg['regiones']['radio_alta_m']:g}", "decisión 2.2")
    return t


# ---------------------------------------------------------------- tabla 5
def tabla_sintonia():
    """Rangos explorados, descartes y configuración adoptada por controlador."""
    sel = cargar_json(SEL_SINTONIA)
    if sel is None:
        return saltar(5, f"no existe {relativa(SEL_SINTONIA)}")
    rangos = cargar_json(RANGOS) or {}
    t = Tabla(5, "sintonia_fase4",
              "Resultado de la tanda de sintonía. Veinte configuraciones por controlador, "
              "muestreadas al azar con semilla fija dentro de rangos declarados antes de correr.",
              ["Controlador", "Rango explorado", "Evaluadas", "Candidatas", "Descartadas",
               "Configuración adoptada", "RMSE de e_y en m", "Razón de esfuerzo"])
    for ctrl in ORDEN_CTRL:
        bloque = sel["controladores"][ctrl]
        corridas = bloque["corridas"]
        descartadas = sum(1 for c in corridas if c["estado"] != "candidata")
        gan = bloque["ganadora"]
        rango = rangos.get("controladores", {}).get(ctrl, {}) if isinstance(rangos, dict) else {}
        texto_rango = ", ".join(
            f"{k} de {v['min']:g} a {v['max']:g}" + (" en escala log" if v.get("escala") == "log" else "")
            for k, v in sorted(rango.items())
            if isinstance(v, dict) and "min" in v and "max" in v) or "ver configs/rangos_sintonia.json"
        # Solo los parámetros que la búsqueda movió. El resto son constantes de
        # la formulación y ya están en la tabla de parámetros congelados.
        moviles = sorted(rango) if rango else sorted(gan["parametros"])
        adoptada = ", ".join(f"{k} {gan['parametros'][k]:.6g}" for k in moviles
                             if isinstance(gan["parametros"].get(k), (int, float)))
        razon = gan.get("razon_esfuerzo")
        t.fila(NOMBRE_CTRL[ctrl], texto_rango, len(corridas), bloque["n_candidatas"], descartadas,
               adoptada, f"{gan['rmse_e_y_m']:.4f}", "" if razon is None else f"{razon:.2f}")
    return t


# ---------------------------------------------------------------- tabla 6
def tabla_repetibilidad():
    """Repetibilidad del piloto y umbral de mejora práctica que de ahí sale."""
    rep = cargar_json(REPETIBILIDAD)
    if rep is None:
        return saltar(6, f"no existe {relativa(REPETIBILIDAD)}")
    t = Tabla(6, "repetibilidad_piloto",
              "Repetibilidad medida en la fase piloto y umbral de mejora práctica adoptado "
              "para cada comparación, declarado antes de la campaña.",
              ["Controlador", "n", "RMSE medio en m", "σ en m", "IC 95 por ciento de σ en m",
               "2σ en m", "10 por ciento del RMSE en m", "Umbral adoptado en m", "Término que manda"])
    for ctrl in ORDEN_CTRL:
        r = rep["por_controlador"][ctrl]
        umbral = rep["umbrales"].get(ctrl)
        ic = r["ic95_sigma_m"]
        t.fila(NOMBRE_CTRL[ctrl], r["n"], f"{r['rmse_medio_m']:.4f}", f"{r['sigma_m']:.5f}",
               f"de {ic[0]:.5f} a {ic[1]:.5f}", f"{r['dos_sigma_m']:.5f}",
               f"{r['diez_por_ciento_rmse_m']:.5f}",
               "" if umbral is None else f"{umbral['umbral_m']:.5f}",
               "" if umbral is None else umbral["manda"])
    return t


# ---------------------------------------------------------------- tabla 7
def tabla_verificacion():
    """Cumplimiento de la plataforma sobre las corridas de fase verificacion."""
    carpetas = sorted(CORRIDAS.glob("verificacion_*")) if CORRIDAS.exists() else []
    manifiestos = []
    for carpeta in carpetas:
        datos = cargar_json(carpeta / "manifiesto.json")
        if datos is not None:
            manifiestos.append(datos)
    if not manifiestos:
        return saltar(7, "no hay corridas de fase verificacion con manifiesto")

    t = Tabla(7, "verificacion_plataforma",
              "Verificación de la plataforma. Valores agregados sobre las corridas de fase "
              "verificacion, con el criterio fijado antes de medir.",
              ["Punto", "Criterio", "Valor observado", "Corridas que cumplen"])

    def agregar(nombre, criterio, clave, formato, prueba, escala=1.0):
        vals = [m["resumen"].get(clave) for m in manifiestos]
        vals = [v * escala for v in vals if v is not None]
        if not vals:
            t.fila(nombre, criterio, "sin dato", f"0 de {len(manifiestos)}")
            return
        cumplen = sum(1 for v in vals if prueba(v))
        t.fila(nombre, criterio,
               f"mediana {format(est.median(vals), formato)}, de {format(min(vals), formato)} "
               f"a {format(max(vals), formato)}",
               f"{cumplen} de {len(vals)}")

    modelos = {m["assetto_corsa"].get("carModel") for m in manifiestos}
    versiones = {m["assetto_corsa"].get("acVersion") for m in manifiestos}
    pistas = {m["assetto_corsa"].get("track") for m in manifiestos}
    t.fila("Vehículo", "igual en todas las corridas", ", ".join(sorted(x for x in modelos if x)),
           f"{len(manifiestos)} de {len(manifiestos)}" if len(modelos) == 1 else "no homogéneo")
    t.fila("Pista", "igual en todas las corridas", ", ".join(sorted(x for x in pistas if x)),
           f"{len(manifiestos)} de {len(manifiestos)}" if len(pistas) == 1 else "no homogéneo")
    t.fila("Versión de Assetto Corsa", "igual en todas las corridas",
           ", ".join(sorted(x for x in versiones if x)),
           f"{len(manifiestos)} de {len(manifiestos)}" if len(versiones) == 1 else "no homogéneo")
    ampliada = sum(1 for m in manifiestos if m["meta"].get("fisica_ampliada"))
    t.fila("Física ampliada", "disponible, necesaria para medir los ejes con los contactos",
           f"{ampliada} de {len(manifiestos)} corridas", f"{ampliada} de {len(manifiestos)}")
    agregar("Batalla medida", "estable entre corridas, en m", "L_medida_mediana_m", ".3f", lambda v: v > 0)
    agregar("Ciclos con contactos válidos", "cercano a 100 por ciento", "pct_contactos_ok", ".1f",
            lambda v: v > 95.0)
    agregar("Periodo del ciclo", "cercano a 50 ms", "periodo_ms_media", ".2f", lambda v: 45.0 <= v <= 55.0)
    agregar("Tiempo de cómputo, p95", "por debajo del periodo de 50 ms", "tc_ms_p95", ".2f",
            lambda v: v < 50.0)
    agregar("Ciclos bajo 50 ms", "al menos 99 por ciento, decisión 1.4", "pct_tc_mayor_50ms", ".2f",
            lambda v: v <= 1.0)
    agregar("Saturación de magnitud", "informativo, en porcentaje", "pct_sat_magnitud", ".2f",
            lambda v: True)
    agregar("Saturación de tasa", "informativo, en porcentaje", "pct_sat_tasa", ".2f", lambda v: True)
    agregar("Respaldo del controlador", "informativo, en porcentaje", "pct_respaldo_controlador", ".2f",
            lambda v: True)
    return t


# ---------------------------------------------------------------- tabla 8
def tabla_campana():
    """Desempeño por celda del mapa, tres perfiles por tres regiones."""
    mapa = cargar_json(MAPA_CAMPANA)
    if mapa is None:
        return saltar(8, f"no existe {relativa(MAPA_CAMPANA)}, la campaña no ha corrido")
    t = Tabla(8, "desempeno_por_celda",
              "Desempeño por celda del mapa de operación. RMSE medio del error lateral y RMS de "
              "la tasa de dirección, con el número de sesiones válidas.",
              ["Perfil", "Región", "Sesiones", "Controlador", "RMSE medio en m", "RMSE mediano en m",
               "RMS de la tasa en rad/s", "Vueltas fallidas", "Máximo de ciclos sobre 50 ms en porcentaje"])
    for perfil in ORDEN_PERFIL:
        for region in ORDEN_REGION:
            celda = next((c for c in mapa["celdas"] if c["perfil"] == perfil and c["region"] == region), None)
            if celda is None:
                continue
            for ctrl in ORDEN_CTRL:
                r = celda["por_controlador"].get(ctrl)
                if r is None:
                    continue
                t.fila(perfil, region, celda.get("sesiones_completas", celda.get("sesiones")),
                       NOMBRE_CTRL[ctrl], f"{r['rmse_medio_m']:.4f}", f"{r['rmse_mediana_m']:.4f}",
                       f"{r['rms_tasa_media_rad_s']:.5f}", r.get("vueltas_fallidas", ""),
                       f"{r.get('pct_tc_mayor_50ms_max', 0.0):.2f}")
    return t


# ---------------------------------------------------------------- tabla 9
def tabla_contrastes():
    """Contraste del MPC frente a cada geométrico, con umbral, intervalo y corrección."""
    mapa = cargar_json(MAPA_CAMPANA)
    if mapa is None:
        return saltar(9, f"no existe {relativa(MAPA_CAMPANA)}, la campaña no ha corrido")
    t = Tabla(9, "contrastes_por_celda",
              "Contraste del MPC cinemático frente a cada controlador geométrico. La reducción "
              "positiva favorece al MPC. El veredicto exige superar el umbral, que el intervalo "
              "no lo cruce y valor p corregido por Holm menor que 0.05.",
              ["Perfil", "Región", "Comparación", "Reducción media en m", "IC 95 por ciento en m",
               "Umbral en m", "p de Wilcoxon", "p de Holm", "Razón de esfuerzo", "Veredicto"])
    for perfil in ORDEN_PERFIL:
        for region in ORDEN_REGION:
            celda = next((c for c in mapa["celdas"] if c["perfil"] == perfil and c["region"] == region), None)
            if celda is None:
                continue
            for ctrl in ("pure_pursuit", "stanley"):
                comp = celda["comparaciones"].get(ctrl)
                if comp is None:
                    continue
                ic = comp.get("ic95_bootstrap_m", [None, None])
                razon = comp.get("razon_esfuerzo_mpc_sobre_geometrico")
                t.fila(perfil, region, f"MPC frente a {NOMBRE_CTRL[ctrl]}",
                       f"{comp['reduccion_media_m']:+.4f}",
                       "" if ic[0] is None else f"de {ic[0]:+.4f} a {ic[1]:+.4f}",
                       f"{comp['umbral_m']:.5f}",
                       f"{comp.get('wilcoxon_p', float('nan')):.4f}",
                       f"{comp.get('p_holm', float('nan')):.4f}",
                       "" if razon is None else f"{razon:.2f}",
                       comp.get("etiqueta", ""))
    return t


TABLAS = {4: tabla_parametros, 5: tabla_sintonia, 6: tabla_repetibilidad,
          7: tabla_verificacion, 8: tabla_campana, 9: tabla_contrastes}


def main():
    global SALIDA, MAPA_CAMPANA
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--solo", nargs="*", type=int, help="números de tabla a generar")
    ap.add_argument("--mapa", default=str(MAPA_CAMPANA),
                    help="archivo de mapa, para ensayar el código con datos que no son de la campaña")
    ap.add_argument("--salida", default=str(SALIDA), help="carpeta de salida")
    args = ap.parse_args()
    MAPA_CAMPANA = Path(args.mapa)
    SALIDA = Path(args.salida)
    if MAPA_CAMPANA != RAIZ_P00 / "resultados_campana" / "mapa_campana.json":
        print(f"AVISO. Mapa tomado de {MAPA_CAMPANA}, que no es el de la campaña. "
              "Estas tablas son un ensayo del código y no van al manuscrito.")

    # La numeración empieza en 4 porque las tablas 1 a 3 del manuscrito son de
    # diseño, no de resultados, y no se generan desde datos.
    pedidas = args.solo or sorted(TABLAS)
    print(f"Tablas de P00 en {relativa(SALIDA)}")
    hechas = []
    for numero in pedidas:
        if numero not in TABLAS:
            print(f"  Tabla {numero} no existe")
            continue
        tabla = TABLAS[numero]()
        if tabla is not None:
            tabla.escribir()
            hechas.append(tabla)
    if hechas:
        ruta_md = SALIDA / "tablas_p00.md"
        with open(ruta_md, "w", encoding="utf-8") as f:
            f.write("# Tablas de resultados de P00\n\n"
                    "Generadas por `scripts/17_p00_tablas.py`. No editar a mano, se regeneran.\n\n")
            f.write("\n\n".join(t.markdown() for t in hechas))
        print(f"  Markdown combinado en {relativa(ruta_md)}")
    print(f"{len(hechas)} de {len(pedidas)} tablas generadas")


if __name__ == "__main__":
    main()
