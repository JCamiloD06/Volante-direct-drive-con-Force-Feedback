"""
Apertura y preparación de Assetto Corsa para una corrida de P00.

Herramienta del lanzador, no es aporte del artículo. Resuelve tres cosas
verificadas en la prueba del 2026-09-16.

Primero, abrir el juego. acs.exe solo abre directo si la carpeta del juego
tiene steam_appid.txt. Sin ese archivo Steam abre el lanzador oficial, que
reescribe la carpeta cfg.

Segundo, fijar la sesión. Antes de abrir se escribe la plantilla de
configs/sesion_ac en la carpeta cfg del juego. Si algún archivo difiere, se
respalda antes la carpeta cfg completa. Las huellas de la plantilla quedan en
el manifiesto de la corrida.

Tercero, entregar el carro. Al cargar, el juego queda en el menú de boxes con
freno en 1.0 sin leer vJoy. Se pulsa el botón del volante y se comprueba que
el freno enviado por vJoy aparece en la memoria compartida.

El reinicio de sesión cierra el juego con el mensaje de cierre de ventana y
lo vuelve a abrir con la plantilla. Se descartó el botón Reiniciar sesión del
menú de pausa porque el juego resalta la opción pero ignora el clic
programado, verificado el 2026-09-16. Reabrir tarda del orden de 11 s más y
deja cada vuelta con un proceso nuevo.
"""
import ctypes
import gc
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from plataforma import procedencia

RUTA_CONFIG = procedencia.RAIZ_P00 / "configs" / "juego_ac.json"
ARCHIVOS_PLANTILLA = ("race.ini", "assists.ini", "controls.ini", "video.ini")
# Custom Shaders Patch se carga solo al abrir acs.exe, a través de dwrite.dll,
# y arrastra con él Sol y Pure si están instalados. No basta con no abrir
# Content Manager. Se registra su presencia en cada preparación, porque cambia
# el clima y la temperatura de pista que fija la plantilla, y con ellas el
# agarre. Añadido el 2026-09-23.
ARCHIVOS_CSP = ("dwrite.dll", "extension")


def buscar_instalaciones():
    """
    Busca carpetas de Assetto Corsa en las bibliotecas de Steam. Sirve para
    decirle al usuario qué poner en ruta_ac cuando el juego no está donde la
    configuración supone. No modifica nada.
    """
    candidatas, bibliotecas = [], []
    try:
        import winreg
        for raiz, clave, valor in ((winreg.HKEY_CURRENT_USER, r"Software\Valve\Steam", "SteamPath"),
                                   (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\WOW6432Node\Valve\Steam", "InstallPath")):
            try:
                with winreg.OpenKey(raiz, clave) as k:
                    bibliotecas.append(Path(winreg.QueryValueEx(k, valor)[0]))
            except OSError:
                continue
    except ImportError:
        pass
    vdf = [b / "steamapps" / "libraryfolders.vdf" for b in list(bibliotecas)]
    for ruta in vdf:
        try:
            texto = ruta.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for linea in texto.splitlines():
            partes = linea.split('"')
            if len(partes) >= 4 and partes[1] == "path":
                bibliotecas.append(Path(partes[3].replace("\\\\", "\\")))
    for biblioteca in bibliotecas:
        candidata = biblioteca / "steamapps" / "common" / "assettocorsa"
        if (candidata / "acs.exe").exists() and candidata not in candidatas:
            candidatas.append(candidata)
    return candidatas


def estado_mods(ruta_ac):
    """Presencia de Custom Shaders Patch en la carpeta del juego, para el manifiesto."""
    ruta = Path(ruta_ac)
    presentes = {nombre: (ruta / nombre).exists() for nombre in ARCHIVOS_CSP}
    return {"archivos": presentes, "csp_activo": any(presentes.values()),
            "sha256_dwrite": procedencia.sha256(ruta / "dwrite.dll"),
            "_nota": "dwrite.dll carga Custom Shaders Patch al abrir acs.exe, y con él Sol y Pure "
                     "si están instalados. Renombrarlo lo desactiva sin desinstalar nada."}


class ErrorJuego(Exception):
    pass


# ---------------------------------------------------------------- configuración y plantilla

def cargar_config(ruta=RUTA_CONFIG):
    with open(ruta, encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["_ruta_config"] = str(ruta)
    cfg["_dir_cfg_juego"] = Path(os.path.expanduser(cfg["directorio_cfg_juego"]))
    cfg["_dir_plantilla"] = procedencia.RAIZ_P00 / cfg["plantilla"]
    cfg["_dir_respaldos"] = procedencia.RAIZ_REPO / cfg["directorio_respaldos"]
    return cfg


def escalar_boton(punto_ref, tam_ref, tam_cliente):
    """Lleva un punto de la ventana de referencia al tamaño real del área cliente."""
    return (int(round(punto_ref[0] * tam_cliente[0] / tam_ref[0])),
            int(round(punto_ref[1] * tam_cliente[1] / tam_ref[1])))


def comparar_plantilla(dir_plantilla, dir_cfg):
    out = {}
    for nombre in ARCHIVOS_PLANTILLA:
        h_p = procedencia.sha256(Path(dir_plantilla) / nombre)
        h_j = procedencia.sha256(Path(dir_cfg) / nombre)
        out[nombre] = {"sha256_plantilla": h_p, "sha256_juego": h_j, "igual": h_p is not None and h_p == h_j}
    return out


def aplicar_plantilla(dir_plantilla, dir_cfg, dir_respaldos, marca=None):
    """
    Escribe la plantilla en la carpeta cfg del juego. Si algún archivo
    difiere, respalda antes la carpeta cfg completa. Nunca borra respaldos.
    """
    dir_plantilla, dir_cfg, dir_respaldos = Path(dir_plantilla), Path(dir_cfg), Path(dir_respaldos)
    faltan = [n for n in ARCHIVOS_PLANTILLA if not (dir_plantilla / n).exists()]
    if faltan:
        raise ErrorJuego(f"faltan archivos de la plantilla {faltan} en {dir_plantilla}")
    if not dir_cfg.exists():
        raise ErrorJuego(f"no existe la carpeta de configuración del juego {dir_cfg}")
    antes = comparar_plantilla(dir_plantilla, dir_cfg)
    distintos = [n for n, c in antes.items() if not c["igual"]]
    respaldo = None
    if distintos:
        marca = marca or time.strftime("%Y%m%d_%H%M%S")
        respaldo = dir_respaldos / marca
        k = 1
        while respaldo.exists():
            respaldo = dir_respaldos / f"{marca}_{k}"
            k += 1
        shutil.copytree(dir_cfg, respaldo)
        for n in distintos:
            shutil.copy2(dir_plantilla / n, dir_cfg / n)
    despues = comparar_plantilla(dir_plantilla, dir_cfg)
    if not all(c["igual"] for c in despues.values()):
        raise ErrorJuego("la plantilla no quedó escrita igual en la carpeta del juego")
    return {"archivos": despues, "reescritos": distintos, "respaldo": str(respaldo) if respaldo else None}


def ultimo_respaldo(dir_respaldos):
    dir_respaldos = Path(dir_respaldos)
    if not dir_respaldos.exists():
        return None
    carpetas = sorted(p for p in dir_respaldos.iterdir() if p.is_dir())
    return carpetas[-1] if carpetas else None


def restaurar_respaldo(respaldo, dir_cfg):
    """Devuelve a la carpeta del juego solo los archivos que maneja la plantilla."""
    restaurados = []
    for n in ARCHIVOS_PLANTILLA:
        origen = Path(respaldo) / n
        if origen.exists():
            shutil.copy2(origen, Path(dir_cfg) / n)
            restaurados.append(n)
    return restaurados


def cerca_de_salida(posicion, v_kmh, cfg):
    d = abs(posicion - cfg["posicion_salida"])
    d = min(d, 1.0 - d)
    return d <= cfg["tolerancia_posicion_salida"] and v_kmh <= cfg["velocidad_max_salida_kmh"]


# ---------------------------------------------------------------- Windows

_k32 = ctypes.WinDLL("kernel32", use_last_error=True) if os.name == "nt" else None
_u32 = ctypes.WinDLL("user32", use_last_error=True) if os.name == "nt" else None
if _k32 is not None:
    _k32.OpenFileMappingW.restype = ctypes.c_void_p
    _k32.CloseHandle.argtypes = [ctypes.c_void_p]


def mapa_existe(nombre):
    """Comprueba la memoria compartida sin crearla. Crearla antes que el juego puede dejarla con otro tamaño."""
    h = _k32.OpenFileMappingW(0x0004, False, nombre)
    if h:
        _k32.CloseHandle(h)
        return True
    return False


def juego_abierto():
    out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq acs.exe", "/NH"], capture_output=True, text=True,
                         creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout
    return "acs.exe" in out.lower()


def comprobar_instalacion(cfg):
    ruta = Path(cfg["ruta_ac"])
    if not (ruta / "acs.exe").exists():
        encontradas = buscar_instalaciones()
        pista = (" Encontré el juego en " + ", ".join(str(c) for c in encontradas) + "."
                 if encontradas else " No encontré ninguna instalación en las bibliotecas de Steam.")
        raise ErrorJuego(f"no se encuentra acs.exe en {ruta}. Corrige la clave ruta_ac en "
                         f"{cfg.get('_ruta_config', RUTA_CONFIG)} con la carpeta donde está instalado "
                         f"Assetto Corsa, la que contiene acs.exe.{pista}")
    appid = ruta / "steam_appid.txt"
    if not appid.exists() or appid.read_text(encoding="utf-8", errors="replace").strip() != cfg["steam_appid"]:
        raise ErrorJuego(f"falta {appid} con el valor {cfg['steam_appid']}. Sin él Steam abre el lanzador oficial.")


def lanzar(cfg):
    comprobar_instalacion(cfg)
    ruta = Path(cfg["ruta_ac"])
    return subprocess.Popen([str(ruta / "acs.exe")], cwd=str(ruta))


class VentanaAC:
    def __init__(self, cfg):
        self.cfg = cfg["ventana"]
        self.hwnd = _u32.FindWindowW(None, self.cfg["titulo"])
        if not self.hwnd:
            raise ErrorJuego("no se encuentra la ventana de Assetto Corsa")

    def tamano_cliente(self):
        from ctypes import wintypes
        rc = wintypes.RECT()
        _u32.GetClientRect(self.hwnd, ctypes.byref(rc))
        return rc.right, rc.bottom

    def al_frente(self):
        """
        Devuelve si quedó al frente, si ya lo estaba y si hizo falta pulsar Alt.
        Alt solo se pulsa si SetForegroundWindow no bastó. Tocar Alt sobre la
        ventana ya activa puede dejarla en modo menú y hacer que el juego no
        reciba el clic siguiente, sospecha del 2026-09-16 NO CONFIRMADA.
        """
        ya = _u32.GetForegroundWindow() == self.hwnd
        alt = False
        if not ya:
            _u32.ShowWindow(self.hwnd, 9)
            _u32.SetForegroundWindow(self.hwnd)
            time.sleep(0.3)
            if _u32.GetForegroundWindow() != self.hwnd:
                # Pulsar y soltar Alt permite que un proceso en segundo plano cambie la ventana activa.
                alt = True
                _u32.keybd_event(0x12, 0, 0, 0)
                _u32.keybd_event(0x12, 0, 0x0002, 0)
                _u32.SetForegroundWindow(self.hwnd)
            time.sleep(0.5)
        return _u32.GetForegroundWindow() == self.hwnd, ya, alt

    def clic(self, nombre_boton):
        from ctypes import wintypes
        x, y = escalar_boton(self.cfg["botones"][nombre_boton], self.cfg["tamano_referencia"], self.tamano_cliente())
        pt = wintypes.POINT(x, y)
        _u32.ClientToScreen(self.hwnd, ctypes.byref(pt))
        al_frente, ya_al_frente, alt = self.al_frente()
        if not al_frente:
            raise ErrorJuego("no se pudo traer al frente la ventana de Assetto Corsa")
        # El juego ignora el clic si el cursor ya estaba quieto sobre el botón,
        # verificado el 2026-09-16. Se lleva el cursor desde el centro en pasos.
        w, h = self.tamano_cliente()
        centro = wintypes.POINT(w // 2, h // 2)
        _u32.ClientToScreen(self.hwnd, ctypes.byref(centro))
        _u32.SetCursorPos(centro.x, centro.y)
        time.sleep(0.3)
        for k in range(1, 11):
            _u32.SetCursorPos(int(centro.x + (pt.x - centro.x) * k / 10), int(centro.y + (pt.y - centro.y) * k / 10))
            time.sleep(0.03)
        time.sleep(0.3)
        _u32.mouse_event(0x0002, 0, 0, 0, 0)
        time.sleep(0.08)
        _u32.mouse_event(0x0004, 0, 0, 0, 0)
        return {"boton": nombre_boton, "cliente": [x, y], "pantalla": [pt.x, pt.y],
                "ya_al_frente": ya_al_frente, "alt": alt}

    def cerrar(self):
        _u32.PostMessageW(self.hwnd, 0x0010, 0, 0)


# ---------------------------------------------------------------- memoria compartida y control

def _leer(mem):
    ph = mem.leer_fisica()
    gr = mem.leer_graficos()
    return {"status": int(gr.status), "packet": int(ph.packetId), "posicion": float(gr.normalizedCarPosition),
            "v_kmh": float(ph.speedKmh), "freno": float(ph.brake), "volante": float(ph.steerAngle)}


def esperar_en_vivo(cfg, tiempo_max, log):
    from plataforma.memoria_ac import MemoriaAC
    t0 = time.perf_counter()
    mem = None
    while time.perf_counter() - t0 < tiempo_max:
        if mem is None and mapa_existe("acpmf_graphics") and mapa_existe("acpmf_physics"):
            time.sleep(1.0)
            mem = MemoriaAC()
            log("Memoria compartida disponible")
        if mem is not None:
            a = _leer(mem)
            time.sleep(0.2)
            b = _leer(mem)
            if b["status"] == 2 and b["packet"] != a["packet"]:
                estatica = mem.leer_estatica()
                esp = cfg["esperado"]
                if estatica["track"] != esp["track"] or estatica["carModel"] != esp["carModel"]:
                    mem.cerrar()
                    raise ErrorJuego(f"sesión con pista {estatica['track']} y carro {estatica['carModel']}, "
                                     f"se esperaba {esp['track']} y {esp['carModel']}")
                return mem, estatica, round(time.perf_counter() - t0, 2)
        time.sleep(0.2)
    if mem is not None:
        mem.cerrar()
    raise ErrorJuego(f"el juego no llegó a sesión en vivo en {tiempo_max:.0f} s")


def probar_control(mem, cfg):
    """
    Envía freno por vJoy y comprueba que aparece en la memoria compartida.
    Deja el freno aplicado y libera el dispositivo para el proceso de la corrida.
    """
    import pyvjoy
    p = cfg["prueba_control"]
    dev = pyvjoy.VJoyDevice(int(cfg["id_vjoy"]))
    try:
        dev.data.wAxisX = 16384
        dev.data.wAxisY = 0
        dev.data.wAxisZRot = int(p["freno"] * 32767)
        dev.update()
        t0 = time.perf_counter()
        lectura = _leer(mem)
        while time.perf_counter() - t0 < p["tiempo_max_s"]:
            lectura = _leer(mem)
            if abs(lectura["freno"] - p["freno"]) <= p["tolerancia"]:
                return True, lectura
            time.sleep(0.1)
        return False, lectura
    finally:
        del dev
        gc.collect()


def cerrar_juego(cfg, log):
    """Cierra el juego como la X de la ventana y espera a que termine el proceso."""
    t0 = time.perf_counter()
    VentanaAC(cfg).cerrar()
    log("Cerrando Assetto Corsa")
    while time.perf_counter() - t0 < cfg["tiempo_max_cierre_s"]:
        if not juego_abierto():
            time.sleep(1.0)
            return round(time.perf_counter() - t0, 2)
        time.sleep(0.5)
    raise ErrorJuego(f"el juego no se cerró en {cfg['tiempo_max_cierre_s']:.0f} s")


def preparar(cfg, reiniciar=False, log=print):
    info = {}
    try:
        return _preparar(cfg, reiniciar, log, info)
    except Exception as e:
        e.info = info
        raise


def _preparar(cfg, reiniciar, log, info):
    """
    Deja Assetto Corsa en vivo, con Monza y el Giulietta, el carro en la
    salida y vJoy con control. Devuelve la información para el manifiesto.
    """
    info.update({"config": cfg["_ruta_config"], "inicio": time.strftime("%Y-%m-%dT%H:%M:%S"),
                 "reinicio_pedido": bool(reiniciar), "pasos": [],
                 "mods": estado_mods(cfg["ruta_ac"])})
    if info["mods"]["csp_activo"]:
        log("AVISO. Custom Shaders Patch está presente en la carpeta del juego y se carga con acs.exe. "
            "Puede sobrescribir el clima y la temperatura de pista de la plantilla, y con ellos el agarre.")
    abierto = juego_abierto()
    if abierto and reiniciar:
        info["cierre_s"] = cerrar_juego(cfg, log)
        log(f"Juego cerrado en {info['cierre_s']} s")
        abierto = False
    info["abierto_por_lanzador"] = not abierto
    if not abierto:
        comprobar_instalacion(cfg)
        info["plantilla"] = aplicar_plantilla(cfg["_dir_plantilla"], cfg["_dir_cfg_juego"], cfg["_dir_respaldos"])
        if info["plantilla"]["reescritos"]:
            log(f"Plantilla escrita, archivos cambiados {info['plantilla']['reescritos']}, "
                f"respaldo en {info['plantilla']['respaldo']}")
        else:
            log("La carpeta del juego ya coincide con la plantilla")
        log("Abriendo Assetto Corsa")
        lanzar(cfg)
        mem, estatica, t_carga = esperar_en_vivo(cfg, cfg["tiempo_max_carga_s"], log)
        info["carga_s"] = t_carga
        log(f"Sesión en vivo en {t_carga} s")
    else:
        log("Assetto Corsa ya está abierto")
        info["plantilla"] = {"archivos": comparar_plantilla(cfg["_dir_plantilla"], cfg["_dir_cfg_juego"]),
                             "reescritos": [], "respaldo": None,
                             "nota": "juego abierto antes, la sesión cargada puede no venir de la plantilla"}
        mem, estatica, _ = esperar_en_vivo(cfg, 10.0, log)
    info["assetto_corsa"] = estatica
    try:
        ventana = VentanaAC(cfg)
        info["tamano_cliente"] = list(ventana.tamano_cliente())
        estado = _leer(mem)
        if not cerca_de_salida(estado["posicion"], estado["v_kmh"], cfg):
            raise ErrorJuego(f"el carro no está en la salida, posición {estado['posicion']:.4f} y "
                             f"{estado['v_kmh']:.1f} km/h. Usa Reiniciar sesión.")
        # El menú puede tardar en aceptar clics aunque la sesión ya esté en vivo,
        # visto al reabrir el 2026-09-16. Solo se pulsa si vJoy aún no controla el carro.
        ok, lectura = probar_control(mem, cfg)
        intentos = cfg["clic_conducir"]
        time.sleep(0.0 if ok else intentos["espera_inicial_s"])
        for k in range(intentos["maximo"]):
            if ok:
                break
            paso = ventana.clic("conducir")
            info["pasos"].append(paso)
            log(f"Clic en el volante del menú, intento {k + 1}, ventana ya al frente "
                f"{'sí' if paso['ya_al_frente'] else 'no'}, Alt {'sí' if paso['alt'] else 'no'}")
            time.sleep(intentos["espera_s"])
            ok, lectura = probar_control(mem, cfg)
        info["clic_automatico_ok"] = ok
        if not ok:
            # Respaldo. El clic programado no siempre entra al reabrir el juego,
            # visto el 2026-09-16. Se espera a que el investigador pulse el volante.
            log("PULSA TÚ EL VOLANTE del menú del juego, el lanzador espera a que vJoy tenga el control")
            t0 = time.perf_counter()
            while not ok and time.perf_counter() - t0 < intentos["espera_manual_s"]:
                time.sleep(1.0)
                ok, lectura = probar_control(mem, cfg)
            info["clic_manual_s"] = round(time.perf_counter() - t0, 1) if ok else None
        info["control"] = {"freno_enviado": cfg["prueba_control"]["freno"], "lectura": lectura, "ok": ok}
        if not ok:
            raise ErrorJuego(f"vJoy no controla el carro, freno leído {lectura['freno']:.2f}")
        log(f"vJoy controla el carro, freno leído {lectura['freno']:.2f}")
        info["estado_final"] = _leer(mem)
        if not cerca_de_salida(info["estado_final"]["posicion"], info["estado_final"]["v_kmh"], cfg):
            raise ErrorJuego("el carro se movió de la salida durante la preparación")
    finally:
        mem.cerrar()
    info["fin"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    return info
