"""
Abandono automático de una corrida, 2026-09-18.

Un candidato de sintonía inestable deja el vehículo contra el muro o girando
sobre sí mismo. La vuelta no termina nunca, y antes había que forzar el cierre
desde el lanzador, lo que mataba el proceso sin ejecutar el guardado y perdía
toda la telemetría. Ocurrió con sint_stanley_02 y sint_stanley_03.

Con esto la corrida se cierra sola y guarda. La vuelta queda como no
completada, que es el resultado que corresponde a ese candidato, y la tanda
puede seguir con el siguiente.

El vigilante solo se arma después de que el vehículo supere una velocidad de
arranque, para no disparar durante la salida de pits, y no cuenta el tiempo
dentro de pits.
"""

VALORES = {
    "velocidad_min_kmh": 5.0,
    "tiempo_quieto_s": 8.0,
    "ruedas_fuera_min": 3,
    "tiempo_ruedas_fuera_s": 3.0,
    "velocidad_arranque_kmh": 30.0,
}


class Abandono:
    def __init__(self, cfg=None):
        c = dict(VALORES)
        c.update(cfg or {})
        self.v_min = float(c["velocidad_min_kmh"])
        self.t_quieto_max = float(c["tiempo_quieto_s"])
        self.ruedas_min = int(c["ruedas_fuera_min"])
        self.t_ruedas_max = float(c["tiempo_ruedas_fuera_s"])
        self.v_arranque = float(c["velocidad_arranque_kmh"])
        self.arrancado = False
        self.t_quieto = None
        self.t_ruedas = None

    def actualizar(self, t, v_kmh, en_pits, ruedas_fuera):
        """Devuelve el motivo del abandono, o None si la corrida sigue."""
        if v_kmh > self.v_arranque:
            self.arrancado = True
        if not self.arrancado or en_pits:
            self.t_quieto = None
            self.t_ruedas = None
            return None
        if v_kmh < self.v_min:
            self.t_quieto = t if self.t_quieto is None else self.t_quieto
            if t - self.t_quieto > self.t_quieto_max:
                return (f"vehiculo detenido, menos de {self.v_min:g} km/h durante mas de "
                        f"{self.t_quieto_max:g} s")
        else:
            self.t_quieto = None
        if ruedas_fuera >= self.ruedas_min:
            self.t_ruedas = t if self.t_ruedas is None else self.t_ruedas
            if t - self.t_ruedas > self.t_ruedas_max:
                return (f"{self.ruedas_min} ruedas o mas fuera de pista durante mas de "
                        f"{self.t_ruedas_max:g} s")
        else:
            self.t_ruedas = None
        return None
