"""
Regiones de curvatura, decisión 2.2.

Baja con radio mayor a 500 m, |κ| menor a 0.002 1/m. Media con |κ| desde
0.002 hasta menos de 0.01 1/m. Alta con |κ| igual o mayor a 0.01 1/m, radio
igual o menor a 100 m. Los radios salen de la configuración.
"""
import numpy as np


def asignar_region(kappa, radio_baja_m, radio_alta_m):
    k = np.abs(np.asarray(kappa, dtype=float))
    k_baja = 1.0 / radio_baja_m
    k_alta = 1.0 / radio_alta_m
    return np.where(k < k_baja, "baja", np.where(k < k_alta, "media", "alta"))
