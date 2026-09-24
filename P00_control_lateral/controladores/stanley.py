"""
Stanley clásico, sección 2.7 del manuscrito, sin feedforward de curvatura.

El manuscrito escribe δ igual a eψ más arctan de k e_y sobre v más ε, con los
errores definidos como trazada menos vehículo. En la convención de este
paquete los errores son vehículo menos trazada, así que la misma ley queda
δ igual a menos eψ menos arctan de k e_y sobre v más ε, con los errores del
eje delantero.

En el registro, extra1 es el término de orientación y extra2 el término de
error lateral, ambos en radianes.
"""
import math

from controladores.base import ControladorLateral, SalidaControlador


class Stanley(ControladorLateral):
    nombre = "stanley"

    def __init__(self, parametros):
        super().__init__(parametros)
        self.k = float(self.parametros["k"])
        self.epsilon = float(self.parametros["epsilon_ms"])
        if self.epsilon <= 0.0:
            raise ValueError("epsilon_ms debe ser positivo")

    def calcular(self, e):
        termino_psi = -e.e_psi_del
        termino_ey = -math.atan(self.k * e.e_y_del / (max(e.v_ms, 0.0) + self.epsilon))
        return SalidaControlador(delta=termino_psi + termino_ey,
                                 extra1=termino_psi, extra2=termino_ey)
