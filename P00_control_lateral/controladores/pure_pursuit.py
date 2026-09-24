"""
Pure Pursuit clásico, sección 2.6 del manuscrito.

Ld igual a L0 más kv por v. El punto objetivo es el punto de la trazada a
distancia Ld del eje trasero, avanzando desde su proyección. α es el ángulo
entre el eje longitudinal del vehículo y la línea al punto objetivo, y
δ igual a arctan de 2 L sen α sobre Ld. Sin feedforward adicional.

En el registro, extra1 es Ld en metros y extra2 es α en radianes.
"""
import math

from controladores.base import ControladorLateral, SalidaControlador
from plataforma.angulos import envolver


class PurePursuit(ControladorLateral):
    nombre = "pure_pursuit"

    def __init__(self, parametros):
        super().__init__(parametros)
        self.L0 = float(self.parametros["L0_m"])
        self.kv = float(self.parametros["kv_s"])
        if self.L0 <= 0.0:
            raise ValueError("L0_m debe ser positivo")

    def calcular(self, e):
        Ld = self.L0 + self.kv * max(e.v_ms, 0.0)
        xt, zt = e.trazada.punto_a_distancia(e.idx_tras, e.x_tras, e.z_tras, Ld)
        alpha = envolver(math.atan2(zt - e.z_tras, xt - e.x_tras) - e.psi)
        delta = math.atan2(2.0 * e.L * math.sin(alpha), Ld)
        return SalidaControlador(delta=delta, extra1=Ld, extra2=alpha)
