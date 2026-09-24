"""
Empaqueta el código fuente del software en un zip para el registro ante la DNDA.

Código auxiliar nuevo, no modifica ningún archivo existente. Solo lee y copia.

Incluye el código de P00_control_lateral, las configuraciones necesarias para
ejecutarlo, la trazada de Monza y dos scripts previos de los mismos autores que
el software ejecuta, mpc_monza_Completo_barrido.py y 08_graficas_corrida.py.
Conserva la estructura de carpetas del repositorio, porque el código calcula
sus rutas a partir de la ubicación de cada archivo.

Excluye los datos y resultados del estudio P00, que no son el software. Los
planes generados, las configuraciones por corrida de sintonía y piloto, las
carpetas de resultados, la documentación y los cachés de Python.

Junto al zip escribe un listado con la huella SHA256 y el número de líneas de
cada archivo incluido.

Uso desde la raíz del repositorio.
    python P00_control_lateral/documentacion/empaquetar_codigo.py
"""
import hashlib
import json
import time
import zipfile
from pathlib import Path

DIR = Path(__file__).resolve().parent
RAIZ_P00 = DIR.parent
RAIZ_REPO = RAIZ_P00.parent
SALIDA = DIR / "entrega"

EXCLUIR_CARPETAS = {"__pycache__", "documentacion", "piloto_fase5", "resultados_campana", "sintonia_fase4",
                    "sintonia_pid"}
EXCLUIR_EN_CONFIGS = {"plan_campana.json", "plan_piloto.json", "plan_sintonia.json", "sintonia", "piloto"}
# Obra previa de los mismos autores que el software usa desde su ruta original. La trazada la lee la
# plataforma, el script de barrido lo ejecuta la pestaña MPC completo y su huella la compara procedencia.py,
# y el guion de gráficas lo ejecuta el botón Generar gráficas.
EXTRAS = [RAIZ_REPO / "Model Predictive Control" / "Python" / "monza_fast_lane.csv",
          RAIZ_REPO / "Model Predictive Control" / "Python" / "mpc_monza_Completo_barrido.py",
          RAIZ_REPO / "scripts" / "08_graficas_corrida.py"]


def incluir(ruta):
    rel = ruta.relative_to(RAIZ_P00)
    if any(p in EXCLUIR_CARPETAS for p in rel.parts):
        return False
    if rel.parts[0] == "configs" and len(rel.parts) > 1 and rel.parts[1] in EXCLUIR_EN_CONFIGS:
        return False
    return ruta.is_file()


def sha256(ruta):
    return hashlib.sha256(ruta.read_bytes()).hexdigest()


def lineas(ruta):
    if ruta.suffix not in (".py", ".json", ".ini", ".md", ".txt", ".csv"):
        return None
    return len(ruta.read_text(encoding="utf-8", errors="replace").splitlines())


def main():
    datos = json.loads((DIR / "datos_portada.json").read_text(encoding="utf-8"))
    nombre = datos.get("NOMBRE", "")
    version = datos.get("VERSION", "")
    base = "PENDIENTE_NOMBRE" if "PENDIENTE" in nombre else "".join(c if c.isalnum() else "_" for c in nombre)
    if "PENDIENTE" not in version:
        base += "_v" + version
    archivos = sorted(r for r in RAIZ_P00.rglob("*") if incluir(r)) + EXTRAS
    SALIDA.mkdir(exist_ok=True)
    destino = SALIDA / f"Codigo_fuente_{base}.zip"
    filas = []
    with zipfile.ZipFile(destino, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for r in archivos:
            nombre_zip = r.relative_to(RAIZ_REPO).as_posix()
            z.write(r, nombre_zip)
            filas.append((nombre_zip, sha256(r), lineas(r)))
    total_py = sum(n for a, _, n in filas if a.endswith(".py") and n)
    propio_py = sum(n for a, _, n in filas if a.endswith(".py") and a.startswith("P00_control_lateral/") and n)
    listado = [f"Código fuente de {nombre} {version}",
               f"Generado el {time.strftime('%Y-%m-%d %H:%M')} por documentacion/empaquetar_codigo.py",
               f"Archivos {len(filas)}, archivos Python {sum(1 for a, _, _ in filas if a.endswith('.py'))}, "
               f"líneas de Python {total_py}",
               f"Líneas de Python de P00_control_lateral {propio_py}, el resto es obra previa de los mismos autores",
               "",
               "archivo | sha256 | lineas"]
    listado += [f"{a} | {h} | {n if n is not None else ''}" for a, h, n in filas]
    (SALIDA / f"Listado_codigo_fuente_{base}.txt").write_text("\n".join(listado) + "\n", encoding="utf-8")
    print(f"{destino.name}, {len(filas)} archivos, {total_py} líneas de Python, "
          f"{destino.stat().st_size / 1024:.0f} KB")


if __name__ == "__main__":
    main()
