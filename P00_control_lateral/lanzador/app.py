"""
Ventana del lanzador de pruebas en Assetto Corsa.

Dos pestañas principales. Prueba de controladores, con las corridas, el plan
de campaña y la verificación de P00. MPC completo, que ejecuta sin modificarlo
ni pasarle argumentos Model Predictive Control/Python/mpc_monza_Completo_barrido.py.
Sin argumentos ese script es mpc_monza_Completo.py con la salvaguarda anti
bloqueo del MPC longitudinal, decisión del investigador del 2026-09-16 después
de que el original se quedara sin acelerar en pista.

Herramienta para ejecutar el experimento del aim, no es aporte del artículo.
La ventana no controla el vehículo. Arma el comando, ejecuta
ejecutar_corrida.py como proceso aparte y muestra su salida, de modo que el
ciclo de control de 50 ms no comparte proceso con la interfaz gráfica.

Iniciar corrida abre y prepara Assetto Corsa con plataforma/juego_ac.py, en
un hilo aparte para no congelar la ventana, y solo entonces lanza la corrida.
La preparación es siempre obligatoria, para que la sesión del juego quede
registrada en el manifiesto de la corrida. Si el juego ya está abierto y el
carro no está en la salida, la ventana ofrece reiniciar la sesión y repetir
el intento.
"""
import csv
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

RAIZ_P00 = Path(__file__).resolve().parents[1]
if str(RAIZ_P00) not in sys.path:
    sys.path.insert(0, str(RAIZ_P00))

from lanzador import corridas as mod_corridas  # noqa: E402
from lanzador import plan as mod_plan
from lanzador import plan_piloto as mod_plan_pil
from lanzador import plan_sintonia as mod_plan_sint  # noqa: E402
from lanzador import proceso_consola  # noqa: E402
from plataforma import procedencia  # noqa: E402

SCRIPT = RAIZ_P00 / "ejecutar_corrida.py"
CONFIG_BASE = RAIZ_P00 / "configs" / "base.json"
FASES = ("verificacion", "sintonia", "piloto", "campana", "prueba")
PREFIJO_GUARDADA = "CORRIDA_GUARDADA|"
SEMILLA_SUGERIDA = 20260915
FASES_CON_PLAN = ("campana", "sintonia", "piloto")
SCRIPT_MPC = procedencia.RAIZ_REPO / "Model Predictive Control" / "Python" / "mpc_monza_Completo_barrido.py"
SCRIPT_GRAFICAS = procedencia.RAIZ_REPO / "scripts" / "08_graficas_corrida.py"
PREFIJO_ANTI_BLOQUEO = "[ANTI-BLOQUEO]"
PREFIJO_MPC_GUARDADA = "Corrida guardada en:"


class Lanzador(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Lanzador de pruebas en Assetto Corsa")
        self.geometry("1180x860")
        self.proceso = None
        self.cola = queue.Queue()
        self.archivo_detener = None
        self.carpeta_ultima = None
        self.etiqueta_repeticion = ""
        self.preparando = False
        self.archivo_sesion_ac = None
        # Última preparación pedida, para saber si el intento ya reinició la
        # sesión y no ofrecer dos veces lo mismo cuando falla.
        self.reinicio_pedido = False
        self.proceso_mpc = None
        self.carpeta_mpc = None
        self.preparacion_mpc = None
        self.anti_bloqueos_mpc = 0
        # Tanda de sintonía. Encadena las corridas del plan de la fase 4, con
        # reinicio de sesión entre una y otra, y se detiene sola si una vuelta
        # no se completa, para no encadenar vueltas sobre un coche atravesado.
        self.tanda = None

        self.var_ctrl = tk.StringVar(value=mod_plan.CONTROLADORES[1])
        self.var_perfil = tk.StringVar(value="conservador")
        self.var_fase = tk.StringVar(value="verificacion")
        self.var_sesion = tk.IntVar(value=0)
        self.var_etiqueta = tk.StringVar(value="")
        self.var_tmax = tk.StringVar(value="")
        self.var_config = tk.StringVar(value=str(CONFIG_BASE))
        self.var_sin_verif = tk.BooleanVar(value=False)
        self.var_id_plan = tk.StringVar(value="")
        self.var_estado = tk.StringVar(value="Listo")
        self.var_estado_ac = tk.StringVar(value="Juego sin preparar en esta ventana")
        self.var_mpc_auto = tk.BooleanVar(value=True)
        self.var_estado_mpc = tk.StringVar(value="MPC detenido")
        self.var_info_sint = tk.StringVar(value="")
        self.var_estado_tanda = tk.StringVar(value="Tanda detenida")
        self.var_sesiones_seguidas = tk.IntVar(value=1)
        self.var_info_pil = tk.StringVar(value="")

        self.nb_principal = ttk.Notebook(self)
        self.nb_principal.pack(fill="both", expand=True, padx=8, pady=(8, 8))
        tab_p00 = ttk.Frame(self.nb_principal)
        self.nb_principal.add(tab_p00, text="Prueba de controladores")
        tab_mpc = ttk.Frame(self.nb_principal)
        self.nb_principal.add(tab_mpc, text="MPC completo")

        self.nb = ttk.Notebook(tab_p00)
        self.nb.pack(fill="both", expand=True, padx=4, pady=4)
        self._tab_corrida()
        self._tab_plan()
        self._tab_sintonia()
        self._tab_piloto()
        self._tab_verificacion()
        self._tab_mpc(tab_mpc)
        # Barra de estado, común a las dos pestañas. A la derecha el estado del
        # juego, que antes vivía en el marco superior ya retirado.
        barra_estado = ttk.Frame(self)
        barra_estado.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Label(barra_estado, textvariable=self.var_estado, anchor="w").pack(side="left")
        ttk.Label(barra_estado, textvariable=self.var_estado_ac, foreground="#444", anchor="e").pack(side="right")

        for v in (self.var_ctrl, self.var_perfil, self.var_fase, self.var_sesion, self.var_etiqueta,
                  self.var_tmax, self.var_config, self.var_sin_verif, self.var_id_plan):
            v.trace_add("write", lambda *_: self._actualizar_vista())
        self._actualizar_vista()
        self.refrescar_plan()
        self.refrescar_sintonia()
        self.refrescar_piloto()
        self.refrescar_verificacion()
        self.protocol("WM_DELETE_WINDOW", self._cerrar)
        self.after(100, self._leer_cola)

    # ------------------------------------------------------------------ pestaña corrida
    def _tab_corrida(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Corrida")
        cfg = ttk.LabelFrame(tab, text="Configuración de la corrida")
        cfg.pack(fill="x", padx=6, pady=6)

        ttk.Label(cfg, text="Fase").grid(row=0, column=0, sticky="w", padx=4, pady=3)
        ttk.Combobox(cfg, textvariable=self.var_fase, values=FASES, state="readonly", width=16).grid(row=0, column=1, sticky="w")
        ttk.Label(cfg, text="Controlador").grid(row=0, column=2, sticky="w", padx=4)
        self.cb_ctrl = ttk.Combobox(cfg, textvariable=self.var_ctrl, values=mod_plan.CONTROLADORES, state="readonly", width=18)
        self.cb_ctrl.grid(row=0, column=3, sticky="w")
        ttk.Label(cfg, text="Perfil").grid(row=0, column=4, sticky="w", padx=4)
        self.cb_perfil = ttk.Combobox(cfg, textvariable=self.var_perfil, values=mod_plan.PERFILES, state="readonly", width=14)
        self.cb_perfil.grid(row=0, column=5, sticky="w")

        ttk.Label(cfg, text="Sesión").grid(row=1, column=0, sticky="w", padx=4, pady=3)
        self.sp_sesion = ttk.Spinbox(cfg, from_=0, to=99, textvariable=self.var_sesion, width=6)
        self.sp_sesion.grid(row=1, column=1, sticky="w")
        ttk.Label(cfg, text="Etiqueta").grid(row=1, column=2, sticky="w", padx=4)
        ttk.Entry(cfg, textvariable=self.var_etiqueta, width=30).grid(row=1, column=3, columnspan=2, sticky="w")
        ttk.Label(cfg, text="Id del plan").grid(row=1, column=5, sticky="w")
        ttk.Label(cfg, textvariable=self.var_id_plan, width=12).grid(row=1, column=6, sticky="w")

        ttk.Label(cfg, text="Tiempo máximo s").grid(row=2, column=0, sticky="w", padx=4, pady=3)
        ttk.Entry(cfg, textvariable=self.var_tmax, width=8).grid(row=2, column=1, sticky="w")
        ttk.Label(cfg, text="Configuración").grid(row=2, column=2, sticky="w", padx=4)
        ttk.Entry(cfg, textvariable=self.var_config, width=60).grid(row=2, column=3, columnspan=3, sticky="we")
        ttk.Button(cfg, text="Elegir", command=self._elegir_config).grid(row=2, column=6, sticky="w", padx=4)
        ttk.Checkbutton(cfg, text="No verificar vehículo, solo pruebas", variable=self.var_sin_verif).grid(
            row=3, column=0, columnspan=3, sticky="w", padx=4, pady=3)

        botones = ttk.Frame(tab)
        botones.pack(fill="x", padx=6)
        self.bt_iniciar = ttk.Button(botones, text="Iniciar corrida", command=self.iniciar)
        self.bt_iniciar.pack(side="left")
        self.bt_detener = ttk.Button(botones, text="Detener y guardar", command=self.detener, state="disabled")
        self.bt_detener.pack(side="left", padx=6)
        ttk.Label(botones, text="Iniciar corrida abre el juego, pulsa el volante del menú y arranca la vuelta",
                  foreground="#444").pack(side="left", padx=12)
        # Se conserva porque devuelve la carpeta del juego al estado previo a la
        # plantilla, y esa acción toca la instalación del investigador.
        self.bt_restaurar = ttk.Button(botones, text="Restaurar configuración del juego",
                                       command=self.restaurar_cfg_juego)
        self.bt_restaurar.pack(side="right")
        self._resto_tab_corrida(tab)

    def _resto_tab_corrida(self, tab):
        self.var_comando = tk.StringVar()
        ttk.Label(tab, textvariable=self.var_comando, foreground="#444", wraplength=1100, justify="left").pack(
            fill="x", padx=6, pady=4)

        panel = ttk.Panedwindow(tab, orient="vertical") if hasattr(ttk, "Panedwindow") else ttk.PanedWindow(tab, orient="vertical")
        panel.pack(fill="both", expand=True, padx=6, pady=6)
        marco_log = ttk.LabelFrame(panel, text="Salida en vivo")
        self.txt_log = tk.Text(marco_log, height=16, font=("Consolas", 9), wrap="none")
        sb = ttk.Scrollbar(marco_log, command=self.txt_log.yview)
        self.txt_log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt_log.pack(fill="both", expand=True)
        marco_res = ttk.LabelFrame(panel, text="Resumen de la última corrida")
        self.txt_res = tk.Text(marco_res, height=10, font=("Consolas", 9), wrap="word")
        self.txt_res.pack(fill="both", expand=True)
        panel.add(marco_log, weight=3)
        panel.add(marco_res, weight=2)

    def _elegir_config(self):
        ruta = filedialog.askopenfilename(initialdir=str(RAIZ_P00 / "configs"), filetypes=[("JSON", "*.json")])
        if ruta:
            self.var_config.set(ruta)

    def _actualizar_vista(self):
        campana = self.var_fase.get() == "campana"
        estado = "disabled" if campana else "readonly"
        self.cb_ctrl.configure(state=estado)
        self.cb_perfil.configure(state=estado)
        self.sp_sesion.configure(state="disabled" if campana else "normal")
        if self.var_fase.get() not in FASES_CON_PLAN and self.var_id_plan.get():
            self.var_id_plan.set("")
        try:
            args = self._argumentos(validar=False)
            self.var_comando.set("Comando: " + " ".join(args[1:]))
        except Exception as e:
            self.var_comando.set(f"Comando incompleto: {e}")

    def _argumentos(self, validar=True, archivo_detener=None, archivo_sesion_ac=None):
        args = [proceso_consola.python_consola(), "-u", str(SCRIPT),
                "--controlador", self.var_ctrl.get(), "--perfil", self.var_perfil.get(),
                "--sesion", str(int(self.var_sesion.get())), "--fase", self.var_fase.get(),
                "--config", self.var_config.get()]
        etiqueta = " ".join(x for x in (self.var_etiqueta.get().strip(), self.etiqueta_repeticion) if x)
        if etiqueta:
            args += ["--etiqueta", etiqueta]
        if self.var_id_plan.get():
            args += ["--id-plan", self.var_id_plan.get()]
        tmax = self.var_tmax.get().strip()
        if tmax:
            float(tmax)
            args += ["--tiempo-max-s", tmax]
        if self.var_sin_verif.get():
            args.append("--sin-verificar-vehiculo")
        if archivo_detener:
            args += ["--archivo-detener", archivo_detener]
        if archivo_sesion_ac:
            args += ["--info-sesion-ac", archivo_sesion_ac]
        # Las fases con plan exigen identificador. En campana viene de la pestaña
        # Plan de campaña y en sintonia de la Tanda de sintonía. Sin esto una
        # corrida suelta queda con fase de plan y sin fila a la que pertenecer,
        # y el análisis la ignora sin que se note.
        if validar and self.var_fase.get() in FASES_CON_PLAN and not self.var_id_plan.get():
            origen = {"campana": "la pestaña Plan de campaña",
                      "sintonia": "la pestaña Tanda de sintonía",
                      "piloto": "la pestaña Piloto"}[self.var_fase.get()]
            raise ValueError(f"en fase {self.var_fase.get()} indica el identificador de plan, "
                             f"normalmente se elige desde {origen}")
        return args

    def _ocupado(self):
        return self.proceso is not None or self.proceso_mpc is not None or self.preparando

    def iniciar(self):
        if self._ocupado():
            return
        try:
            self._argumentos(validar=True)
        except ValueError as e:
            messagebox.showerror("No se puede iniciar", str(e))
            return
        if not Path(self.var_config.get()).exists():
            messagebox.showerror("No se puede iniciar", "no existe el archivo de configuración")
            return
        fase = self.var_fase.get()
        if fase != "campana" and 1 <= int(self.var_sesion.get()) <= 10:
            if not messagebox.askyesno("Sesión reservada",
                                       "Las sesiones 1 a 10 son de la campaña. ¿Usar esa sesión en otra fase?"):
                return
        if self.var_sin_verif.get() and fase in ("campana", "sintonia", "piloto"):
            messagebox.showerror("No se puede iniciar", "no verificar el vehículo solo se permite en verificación o prueba")
            return
        # La preparación es siempre obligatoria. Así la sesión del juego queda
        # registrada en el manifiesto de toda corrida, sea de la fase que sea.
        # Sin reinicio, que cierra y reabre el juego, porque eso solo hace falta
        # si el carro no quedó en la salida, y en ese caso se ofrece al fallar.
        self.txt_log.delete("1.0", "end")
        self._iniciar_preparacion(reiniciar=False, destino="p00")

    def _lanzar_corrida(self, archivo_sesion_ac):
        fase = self.var_fase.get()
        self.archivo_sesion_ac = archivo_sesion_ac
        self.archivo_detener = os.path.join(tempfile.gettempdir(), f"p00_detener_{os.getpid()}_{int(time.time())}.flag")
        args = self._argumentos(validar=True, archivo_detener=self.archivo_detener,
                                archivo_sesion_ac=archivo_sesion_ac)
        entorno = dict(os.environ, PYTHONIOENCODING="utf-8")
        # Sin ventana de consola, para cuando el lanzador se abre desde el acceso directo sin consola.
        banderas = (subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW) if os.name == "nt" else 0
        if archivo_sesion_ac is None:
            self.txt_log.delete("1.0", "end")
        self._log("$ " + " ".join(args[1:]))
        self.proceso = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                        encoding="utf-8", errors="replace", cwd=str(procedencia.RAIZ_REPO),
                                        env=entorno, creationflags=banderas)
        self.carpeta_ultima = None
        threading.Thread(target=self._lector, args=(self.proceso,), daemon=True).start()
        self._botones_ocupados(True)
        self.bt_detener.configure(state="normal")
        self.var_estado.set(f"Corrida en curso, {self.var_ctrl.get()} {self.var_perfil.get()} fase {fase}")

    # ------------------------------------------------------------------ preparación del juego
    def _botones_ocupados(self, ocupado):
        estado = "disabled" if ocupado else "normal"
        for b in (self.bt_iniciar, self.bt_restaurar,
                  self.bt_mpc_iniciar, self.bt_tanda, self.bt_tanda_pil, self.bt_tanda_camp):
            b.configure(state=estado)

    def _log_destino(self, destino, texto):
        if destino == "mpc" or (destino is None and self.nb_principal.index("current") == 1):
            self._log_mpc(texto)
        else:
            self._log(texto)

    def _iniciar_preparacion(self, reiniciar, destino):
        self.reinicio_pedido = bool(reiniciar)
        self.preparando = True
        self._botones_ocupados(True)
        self.var_estado.set("Preparando Assetto Corsa")
        self.var_estado_ac.set("Preparando, no toques el teclado ni el ratón")
        threading.Thread(target=self._hilo_preparacion, args=(reiniciar, destino), daemon=True).start()

    def _hilo_preparacion(self, reiniciar, destino):
        # Cada preparación queda guardada en data, también si falla o si no lanza corrida.
        registro = []
        info = {}

        def log(t):
            registro.append(f"{time.strftime('%H:%M:%S')} {t}")
            self.cola.put(("ac_log", destino, t))

        ruta = None
        try:
            from plataforma import juego_ac
            cfg_juego = juego_ac.cargar_config()
            carpeta = procedencia.RAIZ_REPO / cfg_juego["directorio_preparaciones"]
            carpeta.mkdir(parents=True, exist_ok=True)
            ruta = carpeta / f"preparacion_{time.strftime('%Y%m%d_%H%M%S')}.json"
            info = juego_ac.preparar(cfg_juego, reiniciar=reiniciar, log=log)
            evento = ("ac_ok", destino, str(ruta))
        except Exception as e:
            info = getattr(e, "info", info)
            info["error"] = f"{type(e).__name__}: {e}"
            evento = ("ac_error", destino, info["error"])
        if ruta is not None:
            info["destino"] = destino
            info["registro"] = registro
            with open(ruta, "w", encoding="utf-8") as f:
                json.dump(info, f, ensure_ascii=False, indent=1, default=str)
        self.cola.put(evento)

    def _evento_preparacion(self, evento):
        tipo, destino = evento[0], evento[1]
        if tipo == "ac_log":
            self._log_destino(destino, "[juego] " + evento[2])
            if evento[2].startswith("PULSA"):
                self.var_estado_ac.set("Pulsa tú el volante en el menú del juego")
            return
        self.preparando = False
        self._botones_ocupados(False)
        if tipo == "ac_ok":
            ruta = evento[2]
            self.var_estado_ac.set(f"Juego listo, {time.strftime('%H:%M:%S')}")
            self._log_destino(destino, f"[juego] Preparación completa, guardada en {ruta}")
            if destino == "p00":
                self._lanzar_corrida(ruta)
            elif destino == "mpc":
                self._lanzar_mpc(ruta)
            else:
                self.var_estado.set("Juego listo")
        else:
            mensaje = evento[2]
            self._log_destino(destino, "[juego] ERROR " + mensaje)
            self.var_estado_ac.set("La preparación falló, revisa la salida")
            self.var_estado.set("Corrida no iniciada" if destino else "Preparación fallida")
            if self.tanda is not None:
                # La tanda ya reinicia la sesión antes de cada vuelta, de modo
                # que un fallo aquí no se arregla reiniciando otra vez.
                self._detener_tanda("la preparación del juego falló")
                messagebox.showerror("Assetto Corsa", mensaje)
                return
            if destino and not self.reinicio_pedido and messagebox.askyesno(
                    "Assetto Corsa",
                    f"{mensaje}\n\n¿Cerrar y reabrir el juego para dejar el carro en la salida "
                    "y volver a intentarlo?"):
                self._iniciar_preparacion(reiniciar=True, destino=destino)
                return
            messagebox.showerror("Assetto Corsa", mensaje)

    def restaurar_cfg_juego(self):
        from plataforma import juego_ac
        cfg_juego = juego_ac.cargar_config()
        if juego_ac.juego_abierto():
            messagebox.showerror("Restaurar", "Cierra Assetto Corsa antes de restaurar la configuración.")
            return
        respaldo = juego_ac.ultimo_respaldo(cfg_juego["_dir_respaldos"])
        if respaldo is None:
            messagebox.showinfo("Restaurar", "No hay respaldos. La plantilla nunca cambió la configuración del juego.")
            return
        if not messagebox.askyesno("Restaurar", f"Se copiarán {', '.join(juego_ac.ARCHIVOS_PLANTILLA)} desde el "
                                                f"respaldo {respaldo.name} a la carpeta del juego. ¿Continuar?"):
            return
        restaurados = juego_ac.restaurar_respaldo(respaldo, cfg_juego["_dir_cfg_juego"])
        self._log_destino(None, f"[juego] Restaurados {restaurados} desde {respaldo}")
        self.var_estado_ac.set(f"Configuración restaurada desde {respaldo.name}")

    def _lector(self, proceso):
        for linea in proceso.stdout:
            self.cola.put(linea.rstrip("\n"))
        self.cola.put(None)

    def detener(self):
        if self.proceso is None or self.archivo_detener is None:
            return
        open(self.archivo_detener, "w").close()
        self._log("Solicitud de detención enviada, la corrida guarda el registro al salir.")
        self.var_estado.set("Deteniendo")
        self.after(8000, self._verificar_detencion)

    def _verificar_detencion(self):
        if self.proceso is not None and self.proceso.poll() is None:
            if messagebox.askyesno("La corrida no responde",
                                   "La corrida no terminó. ¿Forzar el cierre? El registro de esta corrida se perderá."):
                self.proceso.terminate()

    def _log(self, texto):
        self.txt_log.insert("end", texto + "\n")
        self.txt_log.see("end")
        # El texto en pantalla se borra al empezar cada corrida, así que la
        # tanda guarda su registro completo en un archivo. Sin esto se perdía
        # la evidencia de una corrida fallida al arrancar la siguiente.
        archivo = (self.tanda or {}).get("archivo_log")
        if archivo:
            try:
                with open(archivo, "a", encoding="utf-8") as f:
                    f.write(f"{time.strftime('%H:%M:%S')} {texto}\n")
            except OSError:
                pass

    def _leer_cola(self):
        try:
            while True:
                linea = self.cola.get_nowait()
                if isinstance(linea, tuple):
                    if linea[0].startswith("graficas_"):
                        self._evento_graficas(linea)
                    elif linea[0].startswith("mpc_"):
                        self._evento_mpc(linea)
                    else:
                        self._evento_preparacion(linea)
                    continue
                if linea is None:
                    self._fin_proceso()
                    break
                if linea.startswith(PREFIJO_GUARDADA):
                    self.carpeta_ultima = linea[len(PREFIJO_GUARDADA):].strip()
                else:
                    self._log(linea)
        except queue.Empty:
            pass
        self.after(100, self._leer_cola)

    def _fin_proceso(self):
        if self.proceso is None:
            return
        codigo = self.proceso.wait()
        self.proceso = None
        if self.archivo_detener and os.path.exists(self.archivo_detener):
            os.remove(self.archivo_detener)
        self.archivo_detener = None
        self.archivo_sesion_ac = None
        self.etiqueta_repeticion = ""
        self._botones_ocupados(False)
        self.bt_detener.configure(state="disabled")
        self._log(f"Proceso terminado con código {codigo}.")
        self.txt_res.delete("1.0", "end")
        if self.carpeta_ultima:
            try:
                self.txt_res.insert("end", mod_corridas.resumen_legible(self.carpeta_ultima))
            except Exception as e:
                self.txt_res.insert("end", f"No se pudo leer el resumen, {e}")
            self.var_estado.set("Corrida guardada")
        else:
            self.txt_res.insert("end", "La corrida no guardó registro. Revisa la salida en vivo.")
            self.var_estado.set("Corrida sin registro")
        self.refrescar_plan()
        self.refrescar_sintonia()
        self.refrescar_piloto()
        self.refrescar_verificacion()
        if self.tanda is not None:
            self._paso_tanda_terminado()

    # ------------------------------------------------------------------ pestaña plan
    def _tab_plan(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Plan de campaña")
        barra = ttk.Frame(tab)
        barra.pack(fill="x", padx=6, pady=6)
        self.var_info_plan = tk.StringVar()
        ttk.Label(barra, textvariable=self.var_info_plan).pack(side="left")
        ttk.Button(barra, text="Usar seleccionada", command=self.usar_seleccionada).pack(side="right")
        # Sesiones encadenadas. Uno es el diseño previsto, con las sesiones
        # separadas en el tiempo. Más de uno sirve para correr sin vigilancia y
        # queda registrado en el log de la tanda, porque debilita el remuestreo.
        ttk.Spinbox(barra, from_=1, to=10, textvariable=self.var_sesiones_seguidas,
                    width=4).pack(side="right", padx=(0, 4))
        ttk.Label(barra, text="Sesiones seguidas").pack(side="right", padx=(8, 2))
        self.bt_tanda_camp = ttk.Button(barra, text="Correr la sesión",
                                        command=self.iniciar_tanda_campana)
        self.bt_tanda_camp.pack(side="right", padx=4)
        ttk.Button(barra, text="Usar siguiente", command=self.usar_siguiente).pack(side="right", padx=4)
        ttk.Button(barra, text="Actualizar", command=self.refrescar_plan).pack(side="right", padx=4)
        self.bt_generar = ttk.Button(barra, text="Generar plan", command=self.generar_plan)
        self.bt_generar.pack(side="right", padx=4)

        columnas = ("id_plan", "sesion", "posicion", "controlador", "perfil", "estado", "intentos", "carpeta")
        self.tree_plan = ttk.Treeview(tab, columns=columnas, show="headings", height=24)
        anchos = (80, 60, 70, 130, 110, 150, 70, 380)
        for c, a in zip(columnas, anchos):
            self.tree_plan.heading(c, text=c)
            self.tree_plan.column(c, width=a, anchor="w")
        self.tree_plan.tag_configure("completada", background="#e3f4e3")
        self.tree_plan.tag_configure("vuelta no completada", background="#fbe9d0")
        self.tree_plan.tag_configure("siguiente", background="#dbe8fb")
        sb = ttk.Scrollbar(tab, command=self.tree_plan.yview)
        self.tree_plan.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y", padx=(0, 6))
        self.tree_plan.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.filas_plan = {}
        self.siguiente = None

    def _cfg(self):
        return mod_corridas.cargar_config(self.var_config.get())

    def refrescar_plan(self):
        plan = mod_plan.cargar_plan()
        self.tree_plan.delete(*self.tree_plan.get_children())
        self.filas_plan = {}
        self.siguiente = None
        if plan is None:
            self.var_info_plan.set("Sin plan de campaña. Genéralo antes de la primera sesión de evaluación.")
            self.bt_generar.configure(state="normal")
            return
        self.bt_generar.configure(state="disabled")
        try:
            lista = mod_corridas.escanear(mod_corridas.directorio_corridas(self._cfg()))
        except Exception:
            lista = []
        filas, self.siguiente = mod_corridas.estado_plan(plan, lista)
        hechas = sum(1 for f in filas if f["estado"] != "pendiente")
        self.var_info_plan.set(f"Plan con semilla {plan['semilla']}, generado {plan['generado']}. "
                               f"{hechas} de {len(filas)} corridas hechas. Siguiente {self.siguiente or 'ninguna'}.")
        for f in filas:
            tag = "siguiente" if f["id_plan"] == self.siguiente else f["estado"]
            self.tree_plan.insert("", "end", iid=f["id_plan"], tags=(tag,),
                                  values=tuple(f[c] for c in ("id_plan", "sesion", "posicion", "controlador",
                                                               "perfil", "estado", "intentos", "carpeta")))
            self.filas_plan[f["id_plan"]] = f

    def generar_plan(self):
        if mod_plan.cargar_plan() is not None:
            messagebox.showinfo("Plan existente", "Ya existe un plan y no se sobrescribe.")
            return
        semilla = simpledialog.askinteger("Semilla del plan", "Semilla para el orden aleatorio de las 10 sesiones",
                                          initialvalue=SEMILLA_SUGERIDA, parent=self)
        if semilla is None:
            return
        if not messagebox.askyesno("Confirmar", f"Se generará y guardará el plan con semilla {semilla}. "
                                                "No podrá cambiarse después. ¿Continuar?"):
            return
        mod_plan.guardar_plan_nuevo(mod_plan.generar_plan(semilla))
        self.refrescar_plan()

    def usar_siguiente(self):
        if self.siguiente is None:
            messagebox.showinfo("Plan", "No hay corridas pendientes o no hay plan.")
            return
        self._cargar_en_corrida(self.filas_plan[self.siguiente])

    def usar_seleccionada(self):
        sel = self.tree_plan.selection()
        if not sel:
            messagebox.showinfo("Plan", "Selecciona una corrida de la tabla.")
            return
        fila = self.filas_plan[sel[0]]
        if fila["estado"] != "pendiente":
            if not messagebox.askyesno(
                    "Repetir corrida",
                    f"{fila['id_plan']} ya tiene {fila['intentos']} intento(s), estado {fila['estado']}. "
                    "Según la decisión 5.2 ninguna vuelta se repite ni se descarta en silencio. "
                    "Si repites, la corrida quedará etiquetada como repetición. ¿Continuar?"):
                return
            self._cargar_en_corrida(fila, repeticion=True)
            return
        if fila["id_plan"] != self.siguiente:
            if not messagebox.askyesno("Fuera de orden",
                                       f"La siguiente según el plan es {self.siguiente}. ¿Usar {fila['id_plan']} de todos modos?"):
                return
        self._cargar_en_corrida(fila)

    def iniciar_tanda_campana(self):
        """
        Corre las vueltas pendientes de la campaña, en el orden del plan.

        Por omisión corre una sola sesión y se detiene, porque la sesión es la
        unidad de remuestreo del análisis y el diseño pide separarlas en el
        tiempo. El campo Sesiones seguidas permite encadenar varias para correr
        sin vigilancia. Cuántas se encadenaron queda en el registro de la tanda
        y en la marca de tiempo de cada corrida, de modo que el análisis puede
        distinguir después las sesiones consecutivas de las separadas.
        """
        if self._ocupado() or self.tanda is not None:
            return
        self.refrescar_plan()
        if not self.filas_plan:
            messagebox.showerror("Campaña", "No hay plan de campaña.")
            return
        pendientes = [f for f in self.filas_plan.values() if f["estado"] == "pendiente"]
        if not pendientes:
            messagebox.showinfo("Campaña", "Todas las corridas del plan ya están hechas.")
            return
        try:
            n_sesiones = max(1, min(10, int(self.var_sesiones_seguidas.get())))
        except (tk.TclError, ValueError):
            n_sesiones = 1
        sesiones = sorted({f["sesion"] for f in pendientes})[:n_sesiones]
        de_las_sesiones = sorted([f for f in pendientes if f["sesion"] in sesiones],
                                 key=lambda f: (f["sesion"], f["posicion"]))
        if str(self.var_config.get()) != str(CONFIG_BASE):
            if not messagebox.askyesno("Campaña", "La configuración no es base.json. La campaña usa los "
                                                  "parámetros congelados. ¿Usar base.json?"):
                return
            self.var_config.set(str(CONFIG_BASE))
        rotulo = (f"Sesión {sesiones[0]}" if len(sesiones) == 1
                  else f"Sesiones {sesiones[0]} a {sesiones[-1]}, {len(sesiones)} seguidas")
        aviso = ("" if len(sesiones) == 1 else
                 "Encadenar sesiones las deja sin separación en el tiempo, y el remuestreo del "
                 "análisis pierde parte de la variación entre jornadas. Queda registrado.\n\n")
        if not messagebox.askyesno(
                "Correr la campaña",
                f"{rotulo}, {len(de_las_sesiones)} vueltas pendientes, en el orden del plan.\n"
                "La tanda reinicia la sesión del juego antes de cada vuelta y se detiene al terminar.\n\n"
                f"{aviso}No uses el teclado ni el ratón mientras corre. ¿Empezar?"):
            return
        carpeta_log = procedencia.RAIZ_REPO / "data" / "raw" / "p00" / "tandas"
        carpeta_log.mkdir(parents=True, exist_ok=True)
        tramo = f"s{sesiones[0]:02d}" if len(sesiones) == 1 else f"s{sesiones[0]:02d}_a_s{sesiones[-1]:02d}"
        archivo_log = carpeta_log / f"campana_{tramo}_{time.strftime('%Y%m%d_%H%M%S')}.log"
        corridas = [dict(f, ruta_config=str(CONFIG_BASE)) for f in de_las_sesiones]
        self.tanda = {"tipo": "campana", "fase": "campana", "pendientes": corridas, "hechas": 0,
                      "fallidas": 0, "total": len(corridas), "parar": False,
                      "archivo_log": str(archivo_log), "sesiones": sesiones}
        self._log(f"Tanda de campaña iniciada, {rotulo.lower()}, {len(corridas)} vueltas, "
                  f"registro en {archivo_log}")
        if len(sesiones) > 1:
            self._log("AVISO. Sesiones encadenadas sin separación en el tiempo, "
                      f"{sesiones}. Queda declarado para el análisis.")
        self.bt_tanda_parar.configure(state="normal")
        self._siguiente_de_tanda()

    def _cargar_en_corrida(self, fila, repeticion=False):
        self.var_fase.set("campana")
        self.var_id_plan.set(fila["id_plan"])
        self.var_sesion.set(fila["sesion"])
        self.var_ctrl.set(fila["controlador"])
        self.var_perfil.set(fila["perfil"])
        self.etiqueta_repeticion = f"repeticion_confirmada_{fila['id_plan']}" if repeticion else ""
        self._actualizar_vista()
        self.nb.select(0)


    # ------------------------------------------------------------------ pestaña sintonía
    def _tab_sintonia(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Tanda de sintonía")
        barra = ttk.Frame(tab)
        barra.pack(fill="x", padx=6, pady=6)
        ttk.Label(barra, textvariable=self.var_info_sint).pack(side="left")
        self.bt_tanda = ttk.Button(barra, text="Iniciar tanda", command=self.iniciar_tanda)
        self.bt_tanda.pack(side="right", padx=4)
        self.bt_tanda_parar = ttk.Button(barra, text="Detener tanda", command=self.parar_tanda, state="disabled")
        self.bt_tanda_parar.pack(side="right", padx=4)
        ttk.Button(barra, text="Usar seleccionada", command=self.usar_seleccionada_sintonia).pack(side="right", padx=4)
        ttk.Button(barra, text="Actualizar", command=self.refrescar_sintonia).pack(side="right", padx=4)
        self.bt_gen_sint = ttk.Button(barra, text="Generar plan", command=self.generar_plan_sintonia)
        self.bt_gen_sint.pack(side="right", padx=4)

        ttk.Label(tab, textvariable=self.var_estado_tanda, anchor="w").pack(fill="x", padx=6)
        columnas = ("id_plan", "controlador", "parametros", "estado", "carpeta")
        self.tree_sint = ttk.Treeview(tab, columns=columnas, show="headings", height=22)
        for c, a in zip(columnas, (170, 130, 330, 150, 300)):
            self.tree_sint.heading(c, text=c)
            self.tree_sint.column(c, width=a, anchor="w")
        self.tree_sint.tag_configure("completada", background="#e3f4e3")
        self.tree_sint.tag_configure("vuelta no completada", background="#fbe9d0")
        self.tree_sint.tag_configure("siguiente", background="#dbe8fb")
        sb = ttk.Scrollbar(tab, command=self.tree_sint.yview)
        self.tree_sint.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y", padx=(0, 6))
        self.tree_sint.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.filas_sint = []

    def refrescar_sintonia(self):
        plan = mod_plan_sint.cargar_plan()
        self.tree_sint.delete(*self.tree_sint.get_children())
        self.filas_sint = []
        if plan is None:
            self.var_info_sint.set("Sin plan de sintonía. Escribe los rangos y pulsa Generar plan.")
            self.bt_gen_sint.configure(state="normal")
            return
        self.bt_gen_sint.configure(state="disabled")
        try:
            lista = mod_corridas.escanear(mod_corridas.directorio_corridas(self._cfg()))
        except Exception:
            lista = []
        hechas = {}
        for c in lista:
            if c["fase"] == "sintonia" and c["id_plan"]:
                hechas.setdefault(c["id_plan"], []).append(c)
        pendientes = 0
        primera_pendiente = True
        for c in plan["corridas"]:
            intentos = sorted(hechas.get(c["id_plan"], []), key=lambda x: x["inicio"])
            if not intentos:
                estado, carpeta = "pendiente", ""
                pendientes += 1
            elif any(x["vuelta_completada"] for x in intentos):
                estado, carpeta = "completada", intentos[-1]["carpeta"].name
            else:
                estado, carpeta = "vuelta no completada", intentos[-1]["carpeta"].name
            etiqueta = estado
            if estado == "pendiente" and primera_pendiente:
                etiqueta, primera_pendiente = "siguiente", False
            params = ", ".join(f"{k}={v:g}" for k, v in c["parametros"].items())
            self.tree_sint.insert("", "end", values=(c["id_plan"], c["controlador"], params, estado, carpeta),
                                  tags=(etiqueta,))
            self.filas_sint.append(dict(c, estado=estado))
        total = len(plan["corridas"])
        self.var_info_sint.set(f"Plan de sintonía, semilla {plan['semilla']}, perfil {plan['perfil']}, "
                               f"{total - pendientes} de {total} corridas hechas")

    def generar_plan_sintonia(self):
        if mod_plan_sint.cargar_plan() is not None:
            messagebox.showerror("Plan de sintonía", "El plan ya existe y no se sobrescribe.")
            return
        if not mod_plan_sint.RUTA_RANGOS.exists():
            messagebox.showerror("Plan de sintonía", f"Falta {mod_plan_sint.RUTA_RANGOS.name}. "
                                                     "Los rangos se escriben antes de generar el plan.")
            return
        semilla = simpledialog.askinteger("Generar plan de sintonía", "Semilla, queda registrada en el plan",
                                          parent=self, minvalue=0)
        if semilla is None:
            return
        rangos = mod_plan_sint.cargar_rangos()
        plan = mod_plan_sint.generar_plan(semilla, rangos)
        ruta = mod_plan_sint.guardar_plan_nuevo(plan)
        rutas = mod_plan_sint.escribir_configs(plan, CONFIG_BASE)
        messagebox.showinfo("Plan de sintonía", f"Plan guardado en {ruta}\n"
                                                f"{len(rutas)} configuraciones escritas en configs/sintonia")
        self.refrescar_sintonia()

    def usar_seleccionada_sintonia(self):
        """
        Carga una fila del plan en la pestaña Corrida, para repetirla a mano.
        Es la única vía de correr una vuelta en fase sintonia, porque esa fase
        exige identificador de plan y el campo no es editable.
        """
        sel = self.tree_sint.selection()
        if not sel:
            messagebox.showinfo("Tanda de sintonía", "Elige primero una fila del plan.")
            return
        fila = self.filas_sint[self.tree_sint.index(sel[0])]
        if fila["estado"] != "pendiente" and not messagebox.askyesno(
                "Repetir corrida", f"{fila['id_plan']} ya está como {fila['estado']}. "
                                   "Repetirla añade un intento y el análisis usa el último. ¿Continuar?"):
            return
        self.var_fase.set("sintonia")
        self.var_ctrl.set(fila["controlador"])
        self.var_perfil.set(fila["perfil"])
        self.var_sesion.set(int(fila["sesion"]))
        self.var_etiqueta.set("")
        self.var_config.set(str(mod_plan_sint.ruta_config(fila)))
        self.var_id_plan.set(fila["id_plan"])
        self.nb.select(0)
        self.var_estado.set(f"Corrida de sintonía cargada, {fila['id_plan']}")

    def iniciar_tanda(self):
        if self._ocupado() or self.tanda is not None:
            return
        plan = mod_plan_sint.cargar_plan()
        if plan is None:
            messagebox.showerror("Tanda", "No hay plan de sintonía.")
            return
        self.refrescar_sintonia()
        pendientes = [f for f in self.filas_sint if f["estado"] == "pendiente"]
        if not pendientes:
            messagebox.showinfo("Tanda", "Todas las corridas del plan ya están hechas.")
            return
        faltan = [f["id_plan"] for f in pendientes if not mod_plan_sint.ruta_config(f).exists()]
        if faltan:
            messagebox.showerror("Tanda", f"Faltan archivos de configuración, por ejemplo {faltan[0]}.json")
            return
        if not messagebox.askyesno("Tanda de sintonía",
                                   f"Se correrán {len(pendientes)} vueltas seguidas, reiniciando la sesión antes "
                                   "de cada una. La tanda se detiene sola si una vuelta no se completa.\n\n"
                                   "No uses el teclado ni el ratón mientras corre. ¿Empezar?"):
            return
        carpeta_log = procedencia.RAIZ_REPO / "data" / "raw" / "p00" / "tandas"
        carpeta_log.mkdir(parents=True, exist_ok=True)
        archivo_log = carpeta_log / f"tanda_{time.strftime('%Y%m%d_%H%M%S')}.log"
        self.tanda = {"tipo": "sintonia", "fase": "sintonia", "pendientes": pendientes, "hechas": 0,
                      "fallidas": 0, "total": len(pendientes), "parar": False,
                      "archivo_log": str(archivo_log)}
        self._log(f"Tanda iniciada, registro en {archivo_log}")
        self.nb.select(self.nb.index("current"))
        self.bt_tanda_parar.configure(state="normal")
        self._siguiente_de_tanda()

    def parar_tanda(self):
        if self.tanda is None:
            return
        self.tanda["parar"] = True
        self.var_estado_tanda.set("La tanda se detendrá al terminar la corrida en curso")
        self._log("Tanda, detención pedida. Termina la corrida en curso y no lanza la siguiente.")

    def _detener_tanda(self, motivo):
        hechas = self.tanda["hechas"] if self.tanda else 0
        total = self.tanda["total"] if self.tanda else 0
        fallidas = self.tanda.get("fallidas", 0) if self.tanda else 0
        self._log(f"Tanda detenida tras {hechas} de {total} corridas, {fallidas} con vuelta no "
                  f"completada. Motivo, {motivo}.")
        self.tanda = None
        self.bt_tanda_parar.configure(state="disabled")
        self.var_estado_tanda.set(f"Tanda detenida, {hechas} de {total} corridas. Motivo, {motivo}")

    def _siguiente_de_tanda(self):
        corrida = self.tanda["pendientes"].pop(0)
        self.var_fase.set(self.tanda["fase"])
        self.var_ctrl.set(corrida["controlador"])
        self.var_perfil.set(corrida["perfil"])
        self.var_sesion.set(int(corrida["sesion"]))
        self.var_etiqueta.set("")
        if corrida.get("ruta_config"):
            self.var_config.set(str(corrida["ruta_config"]))
        else:
            modulo = mod_plan_pil if self.tanda["tipo"] == "piloto" else mod_plan_sint
            self.var_config.set(str(modulo.ruta_config(corrida)))
        self.var_id_plan.set(corrida["id_plan"])
        self.tanda["actual"] = corrida
        n = self.tanda["hechas"] + 1
        fallidas = self.tanda["fallidas"]
        self.var_estado_tanda.set(f"Tanda, corrida {n} de {self.tanda['total']}, {corrida['id_plan']}"
                                  + (f", {fallidas} sin completar la vuelta" if fallidas else ""))
        self.txt_log.delete("1.0", "end")
        if self.tanda["tipo"] == "piloto":
            detalle = f"agarre {corrida['grip_usage_factor']:g}"
        elif self.tanda["tipo"] == "campana":
            detalle = f"sesión {corrida['sesion']}, posición {corrida.get('posicion', '')}"
        else:
            detalle = str(corrida.get("parametros"))
        self._log(f"Tanda, corrida {n} de {self.tanda['total']}, {corrida['id_plan']}, "
                  f"{corrida['controlador']} perfil {corrida['perfil']}, {detalle}")
        self._iniciar_preparacion(reiniciar=True, destino="p00")

    def _paso_tanda_terminado(self):
        actual = self.tanda.get("actual", {})
        idp = actual.get("id_plan", "la corrida")
        if not self.carpeta_ultima:
            # Sin carpeta guardada no hay resultado que interpretar. Es un fallo
            # del lanzador o de la corrida, no del controlador, y la tanda para.
            self._detener_tanda(f"{idp} no guardó registro")
            return
        completada = False
        notas = []
        try:
            man = mod_corridas.leer_manifiesto(self.carpeta_ultima)
            completada = bool((man.get("resumen") or {}).get("vuelta_completada"))
            notas = man.get("notas", []) or []
        except Exception as e:
            self._detener_tanda(f"no se pudo leer el manifiesto de {idp}, {e}")
            return
        if not completada:
            # Un candidato inestable deja la vuelta sin completar. Es el
            # resultado que le corresponde, se registra y la tanda sigue.
            self.tanda["fallidas"] += 1
            motivo = next((n for n in notas if "abandono" in n or "tiempo maximo" in n), "sin nota")
            self._log(f"Tanda, {idp} no completó la vuelta, {motivo}. Cuenta como resultado y sigue.")
        self.tanda["hechas"] += 1
        if self.tanda["tipo"] == "piloto" and actual.get("bloque") == "ajuste_perfil":
            if self._cerrar_ajuste_si_pasa():
                return
        if self.tanda["parar"]:
            self._detener_tanda("detención pedida")
            return
        if not self.tanda["pendientes"]:
            self._detener_tanda("plan terminado")
            return
        self.after(3000, self._siguiente_de_tanda)


    # ------------------------------------------------------------------ pestaña piloto
    def _tab_piloto(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Piloto")
        barra = ttk.Frame(tab)
        barra.pack(fill="x", padx=6, pady=6)
        ttk.Label(barra, textvariable=self.var_info_pil).pack(side="left")
        self.bt_tanda_pil = ttk.Button(barra, text="Iniciar tanda", command=self.iniciar_tanda_piloto)
        self.bt_tanda_pil.pack(side="right", padx=4)
        ttk.Button(barra, text="Repetir seleccionada", command=self.repetir_seleccionada_piloto).pack(
            side="right", padx=4)
        ttk.Button(barra, text="Actualizar", command=self.refrescar_piloto).pack(side="right", padx=4)
        self.bt_tlim = ttk.Button(barra, text="Añadir bloque tiempo límite",
                                  command=self.anadir_bloque_tiempo_limite)
        self.bt_tlim.pack(side="right", padx=4)
        self.bt_gen_pil = ttk.Button(barra, text="Generar plan", command=self.generar_plan_piloto)
        self.bt_gen_pil.pack(side="right", padx=4)

        columnas = ("id_plan", "bloque", "agarre", "controlador", "perfil", "estado", "carpeta")
        self.tree_pil = ttk.Treeview(tab, columns=columnas, show="headings", height=22)
        for c, a in zip(columnas, (200, 120, 70, 130, 100, 170, 260)):
            self.tree_pil.heading(c, text=c)
            self.tree_pil.column(c, width=a, anchor="w")
        self.tree_pil.tag_configure("completada", background="#e3f4e3")
        self.tree_pil.tag_configure("con ruedas fuera", background="#fbe9d0")
        self.tree_pil.tag_configure("vuelta no completada", background="#fbe9d0")
        self.tree_pil.tag_configure("siguiente", background="#dbe8fb")
        self.tree_pil.tag_configure("no necesario", background="#eeeeee", foreground="#666666")
        sb = ttk.Scrollbar(tab, command=self.tree_pil.yview)
        self.tree_pil.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y", padx=(0, 6))
        self.tree_pil.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.filas_pil = []

    def _resultados_piloto(self):
        """Resultado de cada corrida del piloto ya guardada, por identificador."""
        try:
            lista = mod_corridas.escanear(mod_corridas.directorio_corridas(self._cfg()))
        except Exception:
            return {}
        res = {}
        for c in lista:
            if c["fase"] != "piloto" or not c["id_plan"]:
                continue
            anterior = res.get(c["id_plan"])
            if anterior and anterior["inicio"] > c["inicio"]:
                continue
            datos = dict(c)
            datos["ciclos_ruedas_fuera"] = self._ruedas_fuera(c["carpeta"])
            res[c["id_plan"]] = datos
        return res

    @staticmethod
    def _ruedas_fuera(carpeta):
        """
        Ciclos con alguna rueda fuera. Lo trae el resumen desde el 2026-09-19.
        Las corridas anteriores se cuentan leyendo su telemetría.
        """
        try:
            man = mod_corridas.leer_manifiesto(carpeta)
            valor = (man.get("resumen") or {}).get("ciclos_ruedas_fuera")
            if valor is not None:
                return int(valor)
        except Exception:
            return 0
        try:
            with open(Path(carpeta) / "telemetria.csv", encoding="utf-8") as f:
                return sum(1 for r in csv.DictReader(f) if float(r["ruedas_fuera"]) > 0)
        except (OSError, ValueError, KeyError):
            return 0

    def refrescar_piloto(self):
        plan = mod_plan_pil.cargar_plan()
        self.tree_pil.delete(*self.tree_pil.get_children())
        self.filas_pil = []
        if plan is None:
            self.var_info_pil.set("Sin plan de piloto. Revisa parametros_piloto.json y pulsa Generar plan.")
            self.bt_gen_pil.configure(state="normal")
            return
        self.bt_gen_pil.configure(state="disabled")
        res = self._resultados_piloto()
        nivel = mod_plan_pil.nivel_aceptado(plan, res)
        fijado = plan.get("agarre_fijado")
        primera_pendiente = True
        hechas = 0
        for c in plan["corridas"]:
            r = res.get(c["id_plan"])
            if r is None and fijado and c["bloque"] == "ajuste_perfil" and c["nivel"] < fijado["valor"]:
                # Escalón de la escalera que no hizo falta. La escalera se
                # declara completa antes de correr, y el criterio se cumplió en
                # un nivel superior, así que estos no son trabajo pendiente.
                estado, carpeta = "no necesario, nivel superior aceptado", ""
            elif r is None:
                estado, carpeta = "pendiente", ""
            elif not r["vuelta_completada"]:
                estado, carpeta = "vuelta no completada", r["carpeta"].name
                hechas += 1
            elif r["ciclos_ruedas_fuera"] > 0:
                estado, carpeta = f"con ruedas fuera, {r['ciclos_ruedas_fuera']} ciclos", r["carpeta"].name
                hechas += 1
            else:
                estado, carpeta = "completada", r["carpeta"].name
                hechas += 1
            etiqueta = "con ruedas fuera" if estado.startswith("con ruedas") else estado
            if estado.startswith("no necesario"):
                etiqueta = "no necesario"
            if estado == "pendiente" and primera_pendiente:
                etiqueta, primera_pendiente = "siguiente", False
            agarre = f"{c['grip_usage_factor']:g}" if c["grip_usage_factor"] is not None else "sin fijar"
            self.tree_pil.insert("", "end", tags=(etiqueta,),
                                 values=(c["id_plan"], c["bloque"], agarre, c["controlador"],
                                         c["perfil"], estado, carpeta))
            self.filas_pil.append(dict(c, estado=estado))
        no_necesarias = sum(1 for f in self.filas_pil if f["estado"].startswith("no necesario"))
        total = len(plan["corridas"]) - no_necesarias
        ya_tlim = any(c["bloque"] == "tiempo_limite" for c in plan["corridas"])
        self.bt_tlim.configure(state="disabled" if (ya_tlim or not fijado) else "normal")
        texto = f"Plan de piloto, {hechas} de {total} corridas hechas"
        if no_necesarias:
            texto += f", {no_necesarias} escalones de agarre no necesarios"
        if fijado:
            texto += f", agarre fijado en {fijado['valor']:g}"
        elif nivel is not None:
            texto += f", nivel {nivel:g} aceptado pendiente de fijar"
        else:
            texto += ", ajuste del perfil en curso"
        self.var_info_pil.set(texto)

    def generar_plan_piloto(self):
        if mod_plan_pil.cargar_plan() is not None:
            messagebox.showerror("Piloto", "El plan ya existe y no se sobrescribe.")
            return
        par = mod_plan_pil.cargar_parametros()
        rep = par["repetibilidad"]
        aj = par["ajuste_perfil"]
        if not messagebox.askyesno(
                "Generar plan de piloto",
                f"Ajuste del perfil, {len(aj['niveles'])} niveles de {aj['parametro']} "
                f"desde {aj['niveles'][0]:g} hasta {aj['niveles'][-1]:g}, tres controladores cada uno.\n"
                f"Repetibilidad, {rep['repeticiones']} vueltas por controlador en perfil {rep['perfil']}.\n\n"
                "El plan no se podrá sobrescribir. ¿Generar?"):
            return
        plan = mod_plan_pil.generar_plan(par)
        ruta = mod_plan_pil.guardar_plan_nuevo(plan)
        escritas = set()
        for c in plan["corridas"]:
            if c["grip_usage_factor"] is not None:
                escritas.add(mod_plan_pil.escribir_config(c, CONFIG_BASE))
        messagebox.showinfo("Piloto", f"Plan guardado en {ruta}\n"
                                      f"{len(escritas)} configuraciones escritas en configs/piloto")
        self.refrescar_piloto()

    def anadir_bloque_tiempo_limite(self):
        plan = mod_plan_pil.cargar_plan()
        if plan is None:
            messagebox.showerror("Piloto", "No hay plan de piloto.")
            return
        try:
            # Del archivo actual, no de la copia que el plan guardó al generarse.
            tl = mod_plan_pil.cargar_parametros()["tiempo_limite"]
            if not messagebox.askyesno(
                    "Añadir bloque de tiempo límite",
                    f"Se añaden {len(tl['controladores'])} vueltas en perfil {tl['perfil']}, "
                    "una por controlador, con el agarre ya fijado. "
                    "El plan original no incluía ese perfil, que es el más lento, así que la regla "
                    "del tiempo límite no era aplicable con datos del piloto. ¿Añadir?"):
                return
            plan, nuevas = mod_plan_pil.anadir_bloque_tiempo_limite(plan)
            for c in nuevas:
                mod_plan_pil.escribir_config(c, CONFIG_BASE)
        except (ValueError, KeyError) as e:
            messagebox.showerror("Piloto", str(e))
            return
        self._log(f"Piloto, añadidas {len(nuevas)} corridas del bloque tiempo límite.")
        self.refrescar_piloto()

    def repetir_seleccionada_piloto(self):
        """
        Carga una corrida del piloto en la pestaña Corrida, para repetirla a
        mano. Sirve cuando una vuelta se perdió por un fallo de la plataforma y
        no por su resultado. El análisis se queda con el último intento.
        """
        sel = self.tree_pil.selection()
        if not sel:
            messagebox.showinfo("Piloto", "Elige primero una fila del plan.")
            return
        fila = self.filas_pil[self.tree_pil.index(sel[0])]
        if fila["grip_usage_factor"] is None:
            messagebox.showerror("Piloto", "Esa corrida no tiene agarre asignado todavía.")
            return
        if fila["estado"] not in ("pendiente",) and not messagebox.askyesno(
                "Repetir corrida", f"{fila['id_plan']} está como {fila['estado']}. "
                                   "Repetirla añade un intento y el análisis usa el último. ¿Continuar?"):
            return
        self.var_fase.set("piloto")
        self.var_ctrl.set(fila["controlador"])
        self.var_perfil.set(fila["perfil"])
        self.var_sesion.set(int(fila["sesion"]))
        self.var_etiqueta.set("")
        self.var_config.set(str(mod_plan_pil.ruta_config(fila)))
        self.var_id_plan.set(fila["id_plan"])
        self.nb.select(0)
        self.var_estado.set(f"Corrida de piloto cargada, {fila['id_plan']}")

    def iniciar_tanda_piloto(self):
        if self._ocupado() or self.tanda is not None:
            return
        plan = mod_plan_pil.cargar_plan()
        if plan is None:
            messagebox.showerror("Piloto", "No hay plan de piloto.")
            return
        self.refrescar_piloto()
        fijado = plan.get("agarre_fijado")
        # Los bloques se corren en orden. El ajuste primero, porque el agarre
        # entra en los tres perfiles, y el tiempo límite al final, porque usa
        # el agarre ya fijado.
        orden = ["ajuste_perfil", "repetibilidad", "tiempo_limite"] if fijado else ["ajuste_perfil"]
        bloque, pendientes = None, []
        for b in orden:
            candidatas = [f for f in self.filas_pil if f["bloque"] == b and f["estado"] == "pendiente"]
            if candidatas:
                bloque, pendientes = b, candidatas
                break
        if not pendientes:
            messagebox.showinfo("Piloto", "No quedan corridas pendientes en el plan.")
            return
        if bloque == "ajuste_perfil":
            # Solo el nivel más alto que siga pendiente. La escalera baja un
            # escalón por tanda, y entre escalones se revisa qué pasó.
            nivel = max(f["nivel"] for f in pendientes)
            pendientes = [f for f in pendientes if f["nivel"] == nivel]
            aviso = (f"Se correrán los tres controladores con agarre {nivel:g} en perfil "
                     f"{pendientes[0]['perfil']}.\n\nSi los tres completan sin ruedas fuera, el nivel "
                     "queda aceptado y el agarre se fija.")
        else:
            aviso = (f"Se correrán {len(pendientes)} vueltas del bloque {bloque} en perfil "
                     f"{pendientes[0]['perfil']} con agarre {fijado['valor']:g}.")
        faltan = [f["id_plan"] for f in pendientes if not mod_plan_pil.ruta_config(f).exists()]
        if faltan:
            messagebox.showerror("Piloto", f"Faltan archivos de configuración, por ejemplo {faltan[0]}")
            return
        if not messagebox.askyesno("Tanda de piloto", aviso + "\n\nNo uses el teclado ni el ratón "
                                                             "mientras corre. ¿Empezar?"):
            return
        carpeta_log = procedencia.RAIZ_REPO / "data" / "raw" / "p00" / "tandas"
        carpeta_log.mkdir(parents=True, exist_ok=True)
        archivo_log = carpeta_log / f"piloto_{time.strftime('%Y%m%d_%H%M%S')}.log"
        self.tanda = {"tipo": "piloto", "fase": "piloto", "pendientes": pendientes, "hechas": 0,
                      "fallidas": 0, "total": len(pendientes), "parar": False,
                      "archivo_log": str(archivo_log)}
        self._log(f"Tanda de piloto iniciada, bloque {bloque}, registro en {archivo_log}")
        self.bt_tanda_parar.configure(state="normal")
        self._siguiente_de_tanda()

    def _cerrar_ajuste_si_pasa(self):
        """
        Tras cada corrida del ajuste comprueba la escalera. Si el nivel en curso
        pasó con los tres controladores, fija el agarre y detiene la tanda.
        Devuelve True si la tanda quedó detenida.
        """
        plan = mod_plan_pil.cargar_plan()
        nivel = mod_plan_pil.nivel_aceptado(plan, self._resultados_piloto())
        if nivel is None:
            return False
        if not plan.get("agarre_fijado"):
            mod_plan_pil.asignar_agarre_repetibilidad(plan, nivel)
            plan = mod_plan_pil.cargar_plan()
            escritas = 0
            for c in plan["corridas"]:
                if c["bloque"] == "repetibilidad":
                    mod_plan_pil.escribir_config(c, CONFIG_BASE)
                    escritas += 1
            self._log(f"Piloto, nivel de agarre {nivel:g} aceptado con los tres controladores. "
                      f"Queda fijado y el bloque de repetibilidad ya tiene configuración.")
        self._detener_tanda(f"nivel de agarre {nivel:g} aceptado, ajuste del perfil cerrado")
        self.refrescar_piloto()
        messagebox.showinfo("Piloto", f"Nivel de agarre {nivel:g} aceptado. El ajuste del perfil queda "
                                      "cerrado y el bloque de repetibilidad está listo para correr.")
        return True

    # ------------------------------------------------------------------ pestaña verificación
    def _tab_verificacion(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="Verificación fase 3")
        barra = ttk.Frame(tab)
        barra.pack(fill="x", padx=6, pady=6)
        ttk.Label(barra, text="Corrida").pack(side="left")
        self.var_corrida_verif = tk.StringVar()
        self.cb_verif = ttk.Combobox(barra, textvariable=self.var_corrida_verif, state="readonly", width=70)
        self.cb_verif.pack(side="left", padx=6)
        ttk.Button(barra, text="Evaluar", command=self.evaluar).pack(side="left")
        ttk.Button(barra, text="Actualizar lista", command=self.refrescar_verificacion).pack(side="left", padx=6)

        columnas = ("punto", "valor", "criterio", "resultado")
        self.tree_verif = ttk.Treeview(tab, columns=columnas, show="headings", height=22)
        for c, a in zip(columnas, (260, 380, 330, 90)):
            self.tree_verif.heading(c, text=c)
            self.tree_verif.column(c, width=a, anchor="w")
        self.tree_verif.tag_configure("cumple", background="#e3f4e3")
        self.tree_verif.tag_configure("no cumple", background="#f8d9d9")
        self.tree_verif.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        ttk.Label(tab, wraplength=1100, justify="left", text=(
            "Los puntos marcados como informativo no tienen un umbral decidido y se revisan a criterio. "
            "La constante de la cadena de dirección se reidentifica con la batalla medida y la tasa de guiñada "
            "local, cuyo signo no está verificado.")).pack(fill="x", padx=6, pady=(0, 6))
        self.mapa_verif = {}

    def refrescar_verificacion(self):
        try:
            lista = mod_corridas.escanear(mod_corridas.directorio_corridas(self._cfg()))
        except Exception:
            lista = []
        lista = sorted(lista, key=lambda c: c["inicio"], reverse=True)
        self.mapa_verif = {f"{c['inicio']}  {c['id']}": c["carpeta"] for c in lista}
        self.cb_verif.configure(values=list(self.mapa_verif))
        if self.mapa_verif and not self.var_corrida_verif.get():
            self.var_corrida_verif.set(next(iter(self.mapa_verif)))

    def evaluar(self):
        clave = self.var_corrida_verif.get()
        if clave not in self.mapa_verif:
            return
        self.tree_verif.delete(*self.tree_verif.get_children())
        try:
            items = mod_corridas.verificacion_fase3(self.mapa_verif[clave], self._cfg())
        except Exception as e:
            messagebox.showerror("Verificación", f"No se pudo evaluar, {e}")
            return
        for it in items:
            if it["cumple"] is None:
                resultado, tag = "informativo", ""
            elif it["cumple"]:
                resultado, tag = "cumple", "cumple"
            else:
                resultado, tag = "no cumple", "no cumple"
            self.tree_verif.insert("", "end", values=(it["nombre"], it["valor"], it["criterio"], resultado),
                                   tags=(tag,) if tag else ())

    # ------------------------------------------------------------------ pestaña MPC completo
    def _tab_mpc(self, tab):
        info = ttk.LabelFrame(tab, text="MPC completo de dirección y velocidad")
        info.pack(fill="x", padx=6, pady=6)
        huella = procedencia.sha256(SCRIPT_MPC)
        ttk.Label(info, justify="left", wraplength=1100, text=(
            f"Script {SCRIPT_MPC}\n"
            f"Huella SHA256 {huella[:16] if huella else 'ARCHIVO NO ENCONTRADO'}. Se ejecuta tal cual, sin argumentos, "
            "desde su carpeta y con volante simulado. Sin argumentos es el MPC completo con la salvaguarda anti bloqueo "
            "del MPC de velocidad. Guarda su corrida en su carpeta runs al detenerlo.")).pack(
            fill="x", padx=4, pady=3)
        ttk.Checkbutton(info, text="Activar el MPC automáticamente, equivale a pulsar Enter cuando el carro está listo",
                        variable=self.var_mpc_auto).pack(anchor="w", padx=4, pady=(0, 4))

        botones = ttk.Frame(tab)
        botones.pack(fill="x", padx=6)
        self.bt_mpc_iniciar = ttk.Button(botones, text="Iniciar MPC completo", command=self.iniciar_mpc)
        self.bt_mpc_iniciar.pack(side="left")
        self.bt_mpc_activar = ttk.Button(botones, text="Activar MPC", command=self.activar_mpc, state="disabled")
        self.bt_mpc_activar.pack(side="left", padx=6)
        self.bt_mpc_detener = ttk.Button(botones, text="Detener y guardar", command=self.detener_mpc, state="disabled")
        self.bt_mpc_detener.pack(side="left")
        self.bt_mpc_graficas = ttk.Button(botones, text="Generar gráficas", command=self.generar_graficas_mpc,
                                          state="disabled")
        self.bt_mpc_graficas.pack(side="left", padx=6)
        ttk.Label(botones, textvariable=self.var_estado_mpc, foreground="#444").pack(side="left", padx=12)

        panel = ttk.Panedwindow(tab, orient="vertical") if hasattr(ttk, "Panedwindow") else ttk.PanedWindow(tab, orient="vertical")
        panel.pack(fill="both", expand=True, padx=6, pady=6)
        marco_log = ttk.LabelFrame(panel, text="Salida en vivo")
        self.txt_mpc = tk.Text(marco_log, height=18, font=("Consolas", 9), wrap="none")
        sb = ttk.Scrollbar(marco_log, command=self.txt_mpc.yview)
        self.txt_mpc.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt_mpc.pack(fill="both", expand=True)
        marco_res = ttk.LabelFrame(panel, text="Resumen de la última corrida del MPC completo")
        self.txt_mpc_res = tk.Text(marco_res, height=10, font=("Consolas", 9), wrap="word")
        self.txt_mpc_res.pack(fill="both", expand=True)
        panel.add(marco_log, weight=3)
        panel.add(marco_res, weight=2)

    def _log_mpc(self, texto):
        self.txt_mpc.insert("end", texto + "\n")
        self.txt_mpc.see("end")

    def iniciar_mpc(self):
        if self._ocupado():
            return
        if not SCRIPT_MPC.exists():
            messagebox.showerror("MPC completo", f"No se encuentra {SCRIPT_MPC}")
            return
        self.txt_mpc.delete("1.0", "end")
        # Igual que en la corrida de P00, el juego se prepara siempre antes.
        self._iniciar_preparacion(reiniciar=False, destino="mpc")

    def _lanzar_mpc(self, ruta_preparacion):
        self.preparacion_mpc = ruta_preparacion
        self.carpeta_mpc = None
        self.anti_bloqueos_mpc = 0
        self.bt_mpc_graficas.configure(state="disabled")
        args = proceso_consola.argumentos_script(SCRIPT_MPC)
        entorno = dict(os.environ, PYTHONIOENCODING="utf-8")
        self._log_mpc(f"$ {SCRIPT_MPC.name}, carpeta {SCRIPT_MPC.parent}")
        self.proceso_mpc = proceso_consola.iniciar(args, SCRIPT_MPC.parent, entorno)
        threading.Thread(target=self._lector_mpc, args=(self.proceso_mpc,), daemon=True).start()
        self._botones_ocupados(True)
        self.bt_mpc_detener.configure(state="normal")
        if self.var_mpc_auto.get():
            self.activar_mpc()
        else:
            self.bt_mpc_activar.configure(state="normal")
            self.var_estado_mpc.set("MPC cargando, pulsa Activar MPC cuando el carro esté listo")
        self.var_estado.set("MPC completo en curso")

    def activar_mpc(self):
        if self.proceso_mpc is None:
            return
        try:
            # input() del script toma esta línea cuando llega a la pausa de activación.
            self.proceso_mpc.stdin.write("\n")
            self.proceso_mpc.stdin.flush()
        except OSError as e:
            self._log_mpc(f"No se pudo activar, {e}")
            return
        self.bt_mpc_activar.configure(state="disabled")
        self.var_estado_mpc.set("MPC activo, controla dirección, gas y freno")
        self._log_mpc("Activación enviada, equivale a Enter")

    def _lector_mpc(self, proceso):
        for linea in proceso.stdout:
            self.cola.put(("mpc_linea", linea.rstrip("\n")))
        self.cola.put(("mpc_fin",))

    def detener_mpc(self):
        if self.proceso_mpc is None:
            return
        codigo = proceso_consola.enviar_ctrl_c(self.proceso_mpc.pid)
        self._log_mpc("Ctrl+C enviado, el MPC centra los mandos y guarda la corrida al salir" if codigo == 0
                      else f"No se pudo enviar Ctrl+C, código {codigo}")
        self.var_estado_mpc.set("Deteniendo y guardando")
        self.after(20000, self._verificar_detencion_mpc)

    def _verificar_detencion_mpc(self):
        if self.proceso_mpc is not None and self.proceso_mpc.poll() is None:
            if messagebox.askyesno("El MPC no responde",
                                   "El MPC no terminó 20 s después del Ctrl+C. ¿Forzar el cierre? "
                                   "El script no alcanzará a guardar la corrida."):
                self.proceso_mpc.kill()

    def _evento_mpc(self, evento):
        if evento[0] == "mpc_linea":
            linea = evento[1]
            if PREFIJO_MPC_GUARDADA in linea:
                self.carpeta_mpc = linea.split(PREFIJO_MPC_GUARDADA, 1)[1].strip()
            if PREFIJO_ANTI_BLOQUEO in linea:
                self.anti_bloqueos_mpc += 1
                self.var_estado_mpc.set(f"MPC activo, salvaguarda anti bloqueo activada {self.anti_bloqueos_mpc} vez o veces")
            self._log_mpc(linea)
            return
        if self.proceso_mpc is None:
            return
        codigo = self.proceso_mpc.wait()
        self.proceso_mpc = None
        self._botones_ocupados(False)
        self.bt_mpc_activar.configure(state="disabled")
        self.bt_mpc_detener.configure(state="disabled")
        self._log_mpc(f"Proceso terminado con código {codigo}.")
        self.txt_mpc_res.delete("1.0", "end")
        if self.carpeta_mpc and Path(self.carpeta_mpc).exists():
            carpeta = Path(self.carpeta_mpc)
            if self.preparacion_mpc:
                # Archivo nuevo junto a la corrida, el script y sus archivos no se tocan.
                destino = carpeta / "preparacion_juego_ac.json"
                if not destino.exists():
                    destino.write_bytes(Path(self.preparacion_mpc).read_bytes())
            resumen = carpeta / "summary.txt"
            texto = resumen.read_text(encoding="utf-8", errors="replace") if resumen.exists() else "Sin summary.txt"
            self.txt_mpc_res.insert("end", f"Carpeta {carpeta}\nActivaciones de la salvaguarda anti bloqueo "
                                           f"{self.anti_bloqueos_mpc}\n\n{texto}")
            self.var_estado_mpc.set("MPC detenido, corrida guardada")
            self.var_estado.set("Corrida del MPC completo guardada")
            self.bt_mpc_graficas.configure(state="normal")
        else:
            self.txt_mpc_res.insert("end", "El MPC no guardó corrida. Revisa la salida en vivo.")
            self.var_estado_mpc.set("MPC detenido sin corrida guardada")
            self.var_estado.set("MPC completo sin registro")

    def generar_graficas_mpc(self):
        if not self.carpeta_mpc or not Path(self.carpeta_mpc).exists():
            messagebox.showerror("Gráficas", "No hay una corrida guardada para graficar.")
            return
        if not SCRIPT_GRAFICAS.exists():
            messagebox.showerror("Gráficas", f"No se encuentra {SCRIPT_GRAFICAS}")
            return
        self.bt_mpc_graficas.configure(state="disabled")
        self.var_estado_mpc.set("Generando gráficas de la corrida")
        self._log_mpc(f"$ {SCRIPT_GRAFICAS.name} {self.carpeta_mpc}")
        threading.Thread(target=self._hilo_graficas, args=(self.carpeta_mpc,), daemon=True).start()

    def _hilo_graficas(self, carpeta):
        try:
            r = subprocess.run(
                [proceso_consola.python_consola(), "-u", str(SCRIPT_GRAFICAS), carpeta],
                cwd=str(procedencia.RAIZ_REPO), capture_output=True, text=True,
                encoding="utf-8", errors="replace",
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            salida = (r.stdout or "") + (r.stderr or "")
            if r.returncode == 0:
                self.cola.put(("graficas_ok", carpeta, salida))
            else:
                self.cola.put(("graficas_error", salida))
        except Exception as e:
            self.cola.put(("graficas_error", f"{type(e).__name__}: {e}"))

    def _evento_graficas(self, evento):
        tipo = evento[0]
        self.bt_mpc_graficas.configure(state="normal")
        if tipo == "graficas_ok":
            carpeta, salida = evento[1], evento[2]
            for linea in salida.splitlines():
                self._log_mpc("[gráficas] " + linea)
            self.var_estado_mpc.set("Gráficas generadas")
            carpeta_figuras = Path(carpeta) / "figuras"
            if os.name == "nt" and carpeta_figuras.exists():
                os.startfile(carpeta_figuras)  # noqa: S606, abre el explorador en la carpeta de figuras
        else:
            salida = evento[1]
            self._log_mpc("[gráficas] ERROR " + salida)
            self.var_estado_mpc.set("Las gráficas no se generaron, revisa la salida")
            messagebox.showerror("Gráficas", "No se pudieron generar las gráficas. Revisa la salida en vivo.")

    # ------------------------------------------------------------------ cierre
    def _cerrar(self):
        if self.proceso_mpc is not None:
            if not messagebox.askyesno("MPC en curso", "El MPC completo está corriendo. ¿Detenerlo, guardar y cerrar?"):
                return
            proceso_consola.enviar_ctrl_c(self.proceso_mpc.pid)
            try:
                self.proceso_mpc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.proceso_mpc.kill()
        if self.proceso is not None:
            if not messagebox.askyesno("Corrida en curso", "Hay una corrida en curso. ¿Detenerla, guardar y cerrar?"):
                return
            self.detener()
            try:
                self.proceso.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.proceso.terminate()
        self.destroy()


def main():
    Lanzador().mainloop()


if __name__ == "__main__":
    main()
