"""
Seguimiento de la vuelta medida, decisión I7.

La corrida arranca en la posición de salida. Todo lo anterior al primer cruce
de meta es fase salida y no entra en las métricas. La vuelta medida va del
primer cruce al segundo.

Detección del cruce, corregida el 2026-09-15. Se usa el salto de la posición
normalizada en pista, de un valor alto a uno bajo. El contador completedLaps
de Assetto Corsa, que fijaba la decisión 7.2, no cuenta el primer cruce
después de la salida, verificado en las tres corridas de verificación, de modo
que la vuelta medida empezaba tras una vuelta completa de calentamiento. El
contador se conserva como comprobación en el resumen.

En el primer cruce se guardan la diferencia con el perfil y el máximo error
lateral de los últimos segundos, para verificar que el vehículo ya estaba
estabilizado.
"""
import math
from collections import deque


class SeguidorVuelta:
    def __init__(self, ventana_estabilizacion_s=2.0, pos_alta=0.8, pos_baja=0.2):
        self.fase = "salida"
        self.ventana = ventana_estabilizacion_s
        self.pos_alta = pos_alta
        self.pos_baja = pos_baja
        self._historia = deque()
        self.pos_previa = None
        self.vueltas_iniciales = None
        self.cruces = 0
        self.cruce_inicio = None
        self.cruce_fin = None

    def actualizar(self, ciclo, t, vueltas_completadas, posicion_normalizada,
                   v_kmh, v_ref_kmh, e_y):
        self._historia.append((t, abs(e_y)))
        while self._historia and t - self._historia[0][0] > self.ventana:
            self._historia.popleft()
        if self.vueltas_iniciales is None:
            self.vueltas_iniciales = vueltas_completadas

        p = float(posicion_normalizada)
        hubo_cruce = (self.pos_previa is not None and math.isfinite(p)
                      and self.pos_previa > self.pos_alta and p < self.pos_baja)
        if math.isfinite(p):
            self.pos_previa = p

        if hubo_cruce:
            self.cruces += 1
            info = {
                "cruce": self.cruces,
                "ciclo": ciclo,
                "t_s": t,
                "vueltas_completadas": int(vueltas_completadas),
                "diferencia_velocidad_kmh": v_kmh - v_ref_kmh,
                "max_abs_e_y_ventana_m": max(e for _, e in self._historia),
            }
            if self.cruces == 1:
                self.fase = "medida"
                self.cruce_inicio = info
            elif self.cruces == 2:
                self.fase = "terminada"
                self.cruce_fin = info
        return self.fase

    def resumen(self, vueltas_completadas=None):
        tiempo = None
        if self.cruce_inicio and self.cruce_fin:
            tiempo = self.cruce_fin["t_s"] - self.cruce_inicio["t_s"]
        delta_contador = None
        if vueltas_completadas is not None and self.vueltas_iniciales is not None:
            delta_contador = int(vueltas_completadas) - int(self.vueltas_iniciales)
        return {
            "fase_final": self.fase,
            "vuelta_completada": self.fase == "terminada",
            "cruces_detectados": self.cruces,
            "delta_completedLaps": delta_contador,
            "cruce_inicio": self.cruce_inicio,
            "cruce_fin": self.cruce_fin,
            "tiempo_vuelta_s": tiempo,
        }
