import pyvjoy
import time


# ============================================================
# CONFIGURACIÓN
# ============================================================

VJOY_ID = 1

CENTRO = 16384
MINIMO = 1
MAXIMO = 32767


# ============================================================
# CONEXIÓN CON VJOY
# ============================================================

try:
    vjoy = pyvjoy.VJoyDevice(VJOY_ID)
    print("==============================================")
    print("        CONFIGURADOR vJoy - Assetto Corsa")
    print("==============================================")
    print()
    print("vJoy Device 1 conectado correctamente.")
    print()

except Exception as e:
    print("ERROR: No se pudo conectar con vJoy.")
    print()
    print(e)
    input("\nPresiona ENTER para cerrar...")
    exit()


# ============================================================
# FUNCIONES
# ============================================================

def centro():
    """Deja todos los controles en reposo."""

    vjoy.data.wAxisX = CENTRO
    vjoy.data.wAxisY = 0
    vjoy.data.wAxisZRot = 0

    vjoy.update()


def mover_volante():
    """
    Mueve SOLAMENTE el eje X.
    Content Manager debe detectar Steering.
    """

    print()
    print("----------------------------------------------")
    print("VOLANTE - EJE X")
    print("----------------------------------------------")
    print("Haz clic en STEERING dentro de Content Manager.")
    print("Cuando esté esperando el eje, presiona ENTER.")
    input()

    print("Moviendo eje X...")

    # Izquierda
    vjoy.data.wAxisX = MINIMO
    vjoy.data.wAxisY = 0
    vjoy.data.wAxisZRot = 0
    vjoy.update()

    time.sleep(1)

    # Derecha
    vjoy.data.wAxisX = MAXIMO
    vjoy.update()

    time.sleep(1)

    # Centro
    vjoy.data.wAxisX = CENTRO
    vjoy.update()

    print("Eje X enviado.")
    print("Content Manager debería mostrar vJoy X / Steering.")


def mover_acelerador():
    """
    Mueve SOLAMENTE el eje Y.
    Content Manager debe detectar Throttle.
    """

    print()
    print("----------------------------------------------")
    print("ACELERADOR - EJE Y")
    print("----------------------------------------------")
    print("Haz clic en THROTTLE dentro de Content Manager.")
    print("Cuando esté esperando el eje, presiona ENTER.")
    input()

    print("Moviendo eje Y...")

    # 0%
    vjoy.data.wAxisX = CENTRO
    vjoy.data.wAxisY = 0
    vjoy.data.wAxisZRot = 0
    vjoy.update()

    time.sleep(0.5)

    # 50%
    vjoy.data.wAxisY = 16384
    vjoy.update()

    time.sleep(1)

    # 100%
    vjoy.data.wAxisY = MAXIMO
    vjoy.update()

    time.sleep(1)

    # Volver a 0
    vjoy.data.wAxisY = 0
    vjoy.update()

    print("Eje Y enviado.")
    print("Content Manager debería mostrar vJoy Y / Throttle.")


def mover_freno():
    """
    Mueve SOLAMENTE el eje RZ.
    Content Manager debe detectar Brake.
    """

    print()
    print("----------------------------------------------")
    print("FRENO - EJE RZ")
    print("----------------------------------------------")
    print("Haz clic en BRAKE dentro de Content Manager.")
    print("Cuando esté esperando el eje, presiona ENTER.")
    input()

    print("Moviendo eje RZ...")

    # 0%
    vjoy.data.wAxisX = CENTRO
    vjoy.data.wAxisY = 0
    vjoy.data.wAxisZRot = 0
    vjoy.update()

    time.sleep(0.5)

    # 50%
    vjoy.data.wAxisZRot = 16384
    vjoy.update()

    time.sleep(1)

    # 100%
    vjoy.data.wAxisZRot = MAXIMO
    vjoy.update()

    time.sleep(1)

    # Volver a 0
    vjoy.data.wAxisZRot = 0
    vjoy.update()

    print("Eje RZ enviado.")
    print("Content Manager debería mostrar vJoy RZ / Brake.")


# ============================================================
# INICIO
# ============================================================

centro()

print("El programa está listo.")
print()
print("IMPORTANTE:")
print("Tú debes hacer clic manualmente en cada control")
print("de Content Manager antes de presionar ENTER.")
print()
print("Asignación:")
print("  X  = Steering")
print("  Y  = Throttle")
print("  RZ = Brake")
print()


# ============================================================
# 1 - VOLANTE
# ============================================================

mover_volante()


# ============================================================
# 2 - ACELERADOR
# ============================================================

mover_acelerador()


# ============================================================
# 3 - FRENO
# ============================================================

mover_freno()


# ============================================================
# FINAL
# ============================================================

centro()

print()
print("==============================================")
print("CONFIGURACIÓN TERMINADA")
print("==============================================")
print()
print("X  -> Steering")
print("Y  -> Throttle")
print("RZ -> Brake")
print()
print("Todos los ejes quedaron en reposo.")
print()

input("Presiona ENTER para cerrar...")