"""
Trazada de referencia y proyección de puntos del vehículo.

Trazada copia la construcción de ReferencePath de mpc_monza_Completo_barrido.py,
orientación por diferencias y curvatura suavizada con media móvil circular.
Proyector y punto_a_distancia son nuevos. Cada punto del vehículo, eje
trasero, eje delantero y posición de Assetto Corsa, usa su propio Proyector,
porque cada uno lleva su propio índice de búsqueda.

Precisión de la orientación. El arreglo heading es la dirección de cada
cuerda i a i+1. Tomarla como orientación de la trazada en todo el segmento
produce un error de hasta κ por ds sobre 2, del orden de 1.9 grados en la
curva más cerrada de Monza con 1.5 m de espaciado. Por eso la proyección
interpola la tangente entre los vértices del segmento.

Convención. e_y positivo significa punto a la izquierda de la trazada en el
marco de atan2 de z sobre x.
"""
import csv
import math
from dataclasses import dataclass

import numpy as np

from plataforma.angulos import envolver


class Trazada:
    def __init__(self, ruta_csv, suavizado_curvatura_m=15.0):
        xs, zs, s = [], [], []
        with open(ruta_csv, newline="") as f:
            for fila in csv.DictReader(f):
                xs.append(float(fila["x"]))
                zs.append(float(fila["z"]))
                s.append(float(fila["cumulative_length_m"]))

        self.x = np.array(xs)
        self.z = np.array(zs)
        self.s = np.array(s)
        self.n = len(self.x)
        self.total_length = self.s[-1]
        self.avg_spacing = self.total_length / self.n
        self.suavizado_curvatura_m = suavizado_curvatura_m

        dx = np.roll(self.x, -1) - self.x
        dz = np.roll(self.z, -1) - self.z
        self.heading = np.arctan2(dz, dx)
        ds = np.sqrt(dx ** 2 + dz ** 2)
        self.longitud_segmento = np.where(ds < 1e-3, self.avg_spacing, ds)

        # Tangente en cada vértice, promedio angular de la cuerda que llega y la que sale.
        previa = np.roll(self.heading, 1)
        diferencia = (self.heading - previa + np.pi) % (2 * np.pi) - np.pi
        self.tangente_vertice = previa + 0.5 * diferencia

        dtheta = np.roll(self.heading, -1) - self.heading
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        ds_c = np.where(ds < 1e-3, 1e-3, ds)
        self.curvature_raw = dtheta / ds_c

        ventana = max(1, int(round(suavizado_curvatura_m / self.avg_spacing)))
        if ventana % 2 == 0:
            ventana += 1
        if ventana >= 3:
            nucleo = np.ones(ventana) / ventana
            ext = np.concatenate([self.curvature_raw[-(ventana // 2):],
                                  self.curvature_raw,
                                  self.curvature_raw[:ventana // 2]])
            self.curvature = np.convolve(ext, nucleo, mode="valid")
        else:
            self.curvature = self.curvature_raw.copy()
        self.ventana_curvatura_puntos = ventana

    def indice_adelante(self, idx, distancia_m):
        paso = max(0, int(round(distancia_m / self.avg_spacing)))
        return (idx + paso) % self.n

    def curvatura_adelante(self, idx, distancia_m):
        return float(self.curvature[self.indice_adelante(idx, distancia_m)])

    def punto_a_distancia(self, idx, x, z, distancia_m):
        """
        Primer punto de la trazada, avanzando desde idx, a distancia euclidiana
        igual a distancia_m del punto (x, z). Se interpola dentro del segmento.
        Es el punto objetivo de Pure Pursuit.
        """
        px, pz = float(self.x[idx]), float(self.z[idx])
        if math.hypot(px - x, pz - z) >= distancia_m:
            return px, pz
        for k in range(1, self.n // 2):
            j = (idx + k) % self.n
            qx, qz = float(self.x[j]), float(self.z[j])
            if math.hypot(qx - x, qz - z) >= distancia_m:
                dx, dz = qx - px, qz - pz
                a = dx * dx + dz * dz
                b = 2.0 * (dx * (px - x) + dz * (pz - z))
                c = (px - x) ** 2 + (pz - z) ** 2 - distancia_m ** 2
                disc = max(0.0, b * b - 4.0 * a * c)
                t = (-b + math.sqrt(disc)) / (2.0 * a) if a > 0 else 1.0
                t = min(1.0, max(0.0, t))
                return px + t * dx, pz + t * dz
            px, pz = qx, qz
        return px, pz


@dataclass
class Proyeccion:
    idx: int
    s: float
    e_y: float
    phi: float
    kappa: float


class Proyector:
    """Proyección de un punto sobre la trazada con búsqueda local."""

    def __init__(self, trazada, ventana=60):
        self.t = trazada
        self.ventana = ventana
        self._ultimo = None

    def _mas_cercano(self, x, z):
        t = self.t
        if self._ultimo is None:
            d2 = (t.x - x) ** 2 + (t.z - z) ** 2
            return int(np.argmin(d2))
        idxs = (self._ultimo + np.arange(-self.ventana, self.ventana)) % t.n
        d2 = (t.x[idxs] - x) ** 2 + (t.z[idxs] - z) ** 2
        mejor = int(idxs[np.argmin(d2)])
        if d2.min() > 400.0:
            d2g = (t.x - x) ** 2 + (t.z - z) ** 2
            mejor = int(np.argmin(d2g))
        return mejor

    def reiniciar(self):
        self._ultimo = None

    def proyectar(self, x, z):
        t = self.t
        i = self._mas_cercano(x, z)
        # Se proyecta sobre el segmento que sale de i o sobre el que llega a i,
        # según hacia qué lado cae el punto.
        c, s = math.cos(t.heading[i]), math.sin(t.heading[i])
        avance = c * (x - t.x[i]) + s * (z - t.z[i])
        if avance < 0.0:
            i = (i - 1) % t.n
            c, s = math.cos(t.heading[i]), math.sin(t.heading[i])
            avance = c * (x - t.x[i]) + s * (z - t.z[i])
        self._ultimo = i
        largo = float(t.longitud_segmento[i])
        avance = min(max(avance, 0.0), largo)
        e_y = -s * (x - t.x[i]) + c * (z - t.z[i])
        fraccion = avance / largo
        j = (i + 1) % t.n
        tau_i = float(t.tangente_vertice[i])
        tau_j = float(t.tangente_vertice[j])
        phi = envolver(tau_i + fraccion * envolver(tau_j - tau_i))
        s_pos = (float(t.s[i]) + avance) % float(t.total_length)
        return Proyeccion(idx=i, s=s_pos, e_y=float(e_y), phi=phi, kappa=float(t.curvature[i]))
