"""
Ejecución de un script con consola propia oculta y detención con Ctrl+C real.

mpc_monza_Completo.py solo guarda su corrida cuando recibe KeyboardInterrupt,
que en Windows llega con un Ctrl+C de consola. Ctrl+Break o terminar el
proceso lo cierran sin guardar. Por eso el script se lanza con consola propia
oculta, y la detención la hace un proceso auxiliar que se conecta a esa
consola y genera el Ctrl+C, ignorándolo él mismo.
"""
import os
import subprocess
import sys
from pathlib import Path


def python_consola():
    """
    Intérprete con consola para los procesos hijos. Si el lanzador se abre con
    pythonw.exe, sin consola, un hijo con pythonw.exe no recibe Ctrl+C y el MPC
    completo no alcanza a guardar, verificado el 2026-09-16.
    """
    exe = Path(sys.executable)
    if exe.name.lower() == "pythonw.exe" and (exe.parent / "python.exe").exists():
        return str(exe.parent / "python.exe")
    return str(exe)


_AUXILIAR = (
    "import ctypes, sys\n"
    "k = ctypes.windll.kernel32\n"
    "k.FreeConsole()\n"
    "if not k.AttachConsole(int(sys.argv[1])):\n"
    "    sys.exit(2)\n"
    "k.SetConsoleCtrlHandler(None, True)\n"
    "sys.exit(0 if k.GenerateConsoleCtrlEvent(0, 0) else 3)\n"
)


_ENVOLTORIO = (
    "import ctypes, runpy, sys\n"
    "ctypes.windll.kernel32.SetConsoleCtrlHandler(None, False)\n"
    "sys.argv = sys.argv[1:]\n"
    "runpy.run_path(sys.argv[0], run_name='__main__')\n"
)


def argumentos_script(script, extra=()):
    """
    Ejecuta el script sin modificarlo, a través de un envoltorio que reactiva
    Ctrl+C. Esa opción se hereda del proceso padre y puede venir desactivada,
    verificado el 2026-09-16 con un script de prueba.
    """
    return [python_consola(), "-u", "-c", _ENVOLTORIO, str(script), *extra]


def iniciar(args, cwd, env=None):
    info = subprocess.STARTUPINFO()
    info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    info.wShowWindow = 0
    return subprocess.Popen(args, cwd=str(cwd), env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                            creationflags=subprocess.CREATE_NEW_CONSOLE, startupinfo=info)


def enviar_ctrl_c(pid):
    """Devuelve 0 si el Ctrl+C se generó en la consola del proceso."""
    r = subprocess.run([python_consola(), "-c", _AUXILIAR, str(pid)],
                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    return r.returncode
