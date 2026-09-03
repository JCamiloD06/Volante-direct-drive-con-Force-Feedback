import pyvjoy
import time


vjoy = pyvjoy.VJoyDevice(1)


def set_axes(steer, gas, brake):
    """
    steer: 0 - 32767
    gas:   0 - 32767
    brake: 0 - 32767
    """

    vjoy.data.wAxisX = steer
    vjoy.data.wAxisY = gas
    vjoy.data.wAxisZRot = brake

    vjoy.update()


print("Iniciando prueba de vJoy...")
print("Device: 1")
print("X = volante")
print("Y = acelerador")
print("RZ = freno")
print()


# CENTRO
print("1. Centro")
set_axes(16384, 0, 0)
time.sleep(2)


# IZQUIERDA
print("2. Volante izquierda")
set_axes(1, 0, 0)
time.sleep(2)


# CENTRO
print("3. Volante centro")
set_axes(16384, 0, 0)
time.sleep(1)


# DERECHA
print("4. Volante derecha")
set_axes(32767, 0, 0)
time.sleep(2)


# CENTRO
print("5. Volante centro")
set_axes(16384, 0, 0)
time.sleep(1)


# ACELERADOR
print("6. Acelerador 50%")
set_axes(16384, 16384, 0)
time.sleep(2)


# ACELERADOR 100%
print("7. Acelerador 100%")
set_axes(16384, 32767, 0)
time.sleep(2)


# CENTRO
print("8. Sin acelerador")
set_axes(16384, 0, 0)
time.sleep(1)


# FRENO 50%
print("9. Freno 50%")
set_axes(16384, 0, 16384)
time.sleep(2)


# FRENO 100%
print("10. Freno 100%")
set_axes(16384, 0, 32767)
time.sleep(2)


# TODO A CERO
print("11. Liberando controles")
set_axes(16384, 0, 0)

print()
print("Prueba terminada.")