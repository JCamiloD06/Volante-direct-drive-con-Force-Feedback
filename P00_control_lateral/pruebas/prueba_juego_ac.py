"""
Prueba de la preparación de Assetto Corsa sin abrir el juego.

Comprueba la configuración, el escalado de botones, la escritura de la
plantilla con respaldo, la restauración, el criterio de salida y que el
bucle de corrida acepta la información de sesión. Todo en carpetas
temporales, sin tocar la carpeta cfg real del juego.

Uso desde la raíz del repositorio.
    python P00_control_lateral/pruebas/prueba_juego_ac.py
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from plataforma import juego_ac, procedencia  # noqa: E402

RESULTADOS = []


def verificar(nombre, condicion, detalle=""):
    RESULTADOS.append((nombre, bool(condicion)))
    print(f"[{'OK' if condicion else 'FALLA'}] {nombre} {detalle}")


def main():
    cfg = juego_ac.cargar_config()
    verificar("Configuración cargada", cfg["esperado"]["track"] == "monza")
    faltan = [n for n in juego_ac.ARCHIVOS_PLANTILLA if not (cfg["_dir_plantilla"] / n).exists()]
    verificar("Plantilla completa", not faltan, str(faltan))
    race = (cfg["_dir_plantilla"] / "race.ini").read_text(encoding="utf-8", errors="replace")
    verificar("Plantilla con Monza, Giulietta y Hotlap",
              "TRACK=monza" in race and "MODEL=alfa_romeo_giulietta_qv" in race and "TYPE=4" in race)
    controles = (cfg["_dir_plantilla"] / "controls.ini").read_text(encoding="utf-8", errors="replace")
    verificar("Plantilla con vJoy como volante", "CON0=vJoy Device" in controles)

    # Escalado con la medida de la prueba del 2026-09-16, ventana de 1024 por 768 vista como 819 por 614.
    ref = cfg["ventana"]["tamano_referencia"]
    x, y = juego_ac.escalar_boton(cfg["ventana"]["botones"]["conducir"], ref, (819, 614))
    verificar("Escalado del botón conducir", (x, y) == (39, 139), f"{(x, y)}")
    verificar("Escalado identidad", juego_ac.escalar_boton((49, 174), ref, tuple(ref)) == (49, 174))

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        dir_cfg = tmp / "cfg"
        dir_cfg.mkdir()
        dir_resp = tmp / "respaldos"
        for n in juego_ac.ARCHIVOS_PLANTILLA:
            (dir_cfg / n).write_text(f"original {n}\n", encoding="utf-8")
        (dir_cfg / "otro.ini").write_text("no tocar\n", encoding="utf-8")

        r1 = juego_ac.aplicar_plantilla(cfg["_dir_plantilla"], dir_cfg, dir_resp, marca="m1")
        verificar("Primera aplicación reescribe los cuatro archivos", len(r1["reescritos"]) == 4)
        verificar("Respaldo creado con la carpeta completa",
                  r1["respaldo"] is not None and (Path(r1["respaldo"]) / "otro.ini").exists()
                  and (Path(r1["respaldo"]) / "race.ini").read_text(encoding="utf-8") == "original race.ini\n")
        verificar("Archivo ajeno a la plantilla intacto", (dir_cfg / "otro.ini").read_text(encoding="utf-8") == "no tocar\n")
        verificar("Carpeta del juego igual a la plantilla", all(c["igual"] for c in r1["archivos"].values()))

        r2 = juego_ac.aplicar_plantilla(cfg["_dir_plantilla"], dir_cfg, dir_resp, marca="m2")
        verificar("Segunda aplicación no reescribe ni respalda", r2["reescritos"] == [] and r2["respaldo"] is None)

        (dir_cfg / "race.ini").write_text("cambiado por content manager\n", encoding="utf-8")
        r3 = juego_ac.aplicar_plantilla(cfg["_dir_plantilla"], dir_cfg, dir_resp, marca="m1")
        verificar("Cambio posterior detectado y respaldo sin pisar el anterior",
                  r3["reescritos"] == ["race.ini"] and Path(r3["respaldo"]).name == "m1_1"
                  and (dir_resp / "m1").exists())

        ultimo = juego_ac.ultimo_respaldo(dir_resp)
        verificar("Último respaldo", ultimo is not None and ultimo.name == "m1_1", str(ultimo))
        restaurados = juego_ac.restaurar_respaldo(dir_resp / "m1", dir_cfg)
        verificar("Restauración de los archivos de la plantilla",
                  len(restaurados) == 4 and (dir_cfg / "race.ini").read_text(encoding="utf-8") == "original race.ini\n")

    verificar("Salida en la posición de hotlap", juego_ac.cerca_de_salida(0.8564, 0.0, cfg))
    verificar("Fuera de la salida por posición", not juego_ac.cerca_de_salida(0.30, 0.0, cfg))
    verificar("Fuera de la salida por velocidad", not juego_ac.cerca_de_salida(0.8564, 20.0, cfg))
    verificar("Salida cerca del salto de 1 a 0",
              juego_ac.cerca_de_salida(0.001, 0.0, dict(cfg, posicion_salida=0.999)))

    ayuda = subprocess.run([sys.executable, str(RAIZ_P00 / "ejecutar_corrida.py"), "--help"],
                           capture_output=True, text=True, encoding="utf-8", errors="replace").stdout
    verificar("El bucle de corrida acepta la información de sesión", "--info-sesion-ac" in ayuda)
    verificar("Huella del módulo nuevo en el manifiesto", "plataforma/juego_ac.py" in procedencia.huellas_codigo_p00())

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} correctas")
    sys.exit(1 if fallas else 0)


if __name__ == "__main__":
    main()
