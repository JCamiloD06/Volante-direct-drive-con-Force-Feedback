"""
Interfaz común de dirección y salida a vJoy.

InterfazDireccion aplica los mismos límites a los tres controladores,
decisión I5, magnitud de 0.43 rad y tasa de 30 grados por segundo. El paso de
tasa usa el tiempo real desde el último envío, acotado a dos periodos
nominales para que un ciclo largo no habilite un salto grande.

SalidaVJoy adapta VJoyOutput de mpc_monza_Completo_barrido.py. El ángulo de
rueda se convierte a eje con la constante rueda_rad_eje_completo, y el signo
se fija con signo_vjoy, verificado positivo el 2026-09-15.
"""
import numpy as np


class InterfazDireccion:
    def __init__(self, delta_max_rad, tasa_max_rad_s, ts_nominal_s):
        self.delta_max = float(delta_max_rad)
        self.tasa_max = float(tasa_max_rad_s)
        self.ts = float(ts_nominal_s)
        self.delta_prev = 0.0
        self.t_prev = None

    def aplicar(self, delta_pedido, t_ahora):
        if self.t_prev is None:
            dt = self.ts
        else:
            dt = min(max(t_ahora - self.t_prev, 0.0), 2.0 * self.ts)
        sat_mag = abs(delta_pedido) > self.delta_max
        d = float(np.clip(delta_pedido, -self.delta_max, self.delta_max))
        paso = self.tasa_max * dt
        cambio = d - self.delta_prev
        sat_tasa = abs(cambio) > paso
        d = self.delta_prev + float(np.clip(cambio, -paso, paso))
        self.delta_prev = d
        self.t_prev = t_ahora
        return d, sat_mag, sat_tasa

    def reiniciar(self):
        self.delta_prev = 0.0
        self.t_prev = None


def entrada_progresiva(delta_rad, v_kmh, v_nula_kmh, v_plena_kmh):
    """
    Entrada gradual de la dirección a baja velocidad, igual para los tres
    controladores. Devuelve el comando escalado y el factor aplicado.

    Por debajo de v_nula el comando es cero, por encima de v_plena es el del
    controlador, y en medio se escala de forma lineal con la velocidad.

    Motivo. Con el vehículo casi detenido las leyes laterales piden ángulos
    muy grandes para corregir el error inicial de la posición de salida. El
    MPC predice sin avance, la anticipación de Pure Pursuit es de pocos metros
    y el término lateral de Stanley divide por una velocidad casi nula. En
    pista el 2026-09-15 esto sacó a Pure Pursuit de la pista durante el
    arranque, con las cuatro ruedas fuera en 23 ciclos, y produjo oscilación
    en Stanley. La rampa afecta solo el arranque y no la vuelta medida, que
    transcurre muy por encima de v_plena.
    """
    if v_kmh <= v_nula_kmh:
        return 0.0, 0.0
    if v_kmh >= v_plena_kmh:
        return delta_rad, 1.0
    factor = (v_kmh - v_nula_kmh) / (v_plena_kmh - v_nula_kmh)
    return delta_rad * factor, factor


def a_eje_normalizado(delta_rad, rueda_rad_eje_completo, signo_vjoy):
    razon = signo_vjoy * delta_rad / rueda_rad_eje_completo
    return float(np.clip(razon, -1.0, 1.0)), abs(razon) > 1.0


class SalidaVJoy:
    def __init__(self, rueda_rad_eje_completo, signo_vjoy=1, id_dispositivo=1):
        import pyvjoy  # solo se importa al correr en vivo
        self.j = pyvjoy.VJoyDevice(id_dispositivo)
        self.rueda = float(rueda_rad_eje_completo)
        self.signo = int(signo_vjoy)
        self.centrar()

    def enviar(self, delta_rad, u_long):
        norm, sat_eje = a_eje_normalizado(delta_rad, self.rueda, self.signo)
        valor_dir = max(1, min(32767, int((norm + 1.0) / 2.0 * 32766 + 1)))
        if u_long >= 0:
            gas, freno = int(u_long * 32767), 0
        else:
            gas, freno = 0, int(-u_long * 32767)
        self.j.data.wAxisX = valor_dir
        self.j.data.wAxisY = gas
        self.j.data.wAxisZRot = freno
        self.j.update()
        return gas / 32767.0, freno / 32767.0, norm, sat_eje

    def centrar(self):
        self.j.data.wAxisX = 16384
        self.j.data.wAxisY = 0
        self.j.data.wAxisZRot = 0
        self.j.update()


class SalidaSimulada:
    """Misma interfaz que SalidaVJoy, sin dispositivo. Para pruebas."""

    def __init__(self, rueda_rad_eje_completo, signo_vjoy=1):
        self.rueda = float(rueda_rad_eje_completo)
        self.signo = int(signo_vjoy)
        self.ultimo = None

    def enviar(self, delta_rad, u_long):
        norm, sat_eje = a_eje_normalizado(delta_rad, self.rueda, self.signo)
        gas = max(0.0, u_long)
        freno = max(0.0, -u_long)
        self.ultimo = (delta_rad, norm, gas, freno)
        return gas, freno, norm, sat_eje

    def centrar(self):
        self.ultimo = (0.0, 0.0, 0.0, 0.0)
