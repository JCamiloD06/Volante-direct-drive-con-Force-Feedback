"""
Interfaz común de los tres controladores laterales.

Todos reciben la misma EntradaLateral y devuelven un ángulo de rueda δ en
radianes. Convención de signos del paquete. δ positivo hace crecer ψ. e_y
positivo significa punto a la izquierda de la trazada. eψ es ψ menos la
orientación de la trazada en la proyección del punto.

Los errores del eje trasero los usan Pure Pursuit y el MPC. Los del eje
delantero los usa Stanley. Decisión I2.
"""
from dataclasses import dataclass


@dataclass
class EntradaLateral:
    trazada: object
    v_ms: float
    L: float
    psi: float
    x_tras: float
    z_tras: float
    x_del: float
    z_del: float
    idx_tras: int
    e_y_tras: float
    e_psi_tras: float
    idx_del: int
    e_y_del: float
    e_psi_del: float
    delta_prev: float


@dataclass
class SalidaControlador:
    delta: float
    estado: str = "ok"
    iteraciones: int = 0
    respaldo: int = 0
    extra1: float = float("nan")
    extra2: float = float("nan")


class ControladorLateral:
    nombre = "base"

    def __init__(self, parametros):
        self.parametros = dict(parametros)

    def reiniciar(self):
        pass

    def calcular(self, entrada):
        raise NotImplementedError
