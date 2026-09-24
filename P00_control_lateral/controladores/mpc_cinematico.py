"""
MPC lateral con modelo bicicleta cinemático, sección 2.8 del manuscrito.

Decisiones que implementa.

I4.1 Modelo de predicción lineal en errores del eje trasero, discretizado con
el periodo de control. Con v medida constante en el horizonte,
    e_y[k+1]  = e_y[k] + Ts v eψ[k]
    eψ[k+1]   = eψ[k] + Ts v (δ[k] / L - κ[k])
donde κ[k] es la curvatura de la trazada a la distancia v Ts k por delante de
la proyección del eje trasero. Se usan las aproximaciones sen eψ igual a eψ y
tan δ igual a δ.

I4.2 Costo
    J = Σ Qy e_y[k]² + Qψ eψ[k]²            k = 1..N
      + Σ Rδ (δ[k] - atan(L κ[k]))²          k = 0..N-1
      + Σ RΔδ (δ[k] - δ[k-1])²               k = 0..N-1, con δ[-1] el último aplicado
El segundo término penaliza la diferencia con el ángulo cinemático de la
curva en lugar de δ², para no sesgar al MPC hacia afuera en curva. Con el
modelo lineal el ángulo de régimen es L κ y no atan(L κ). La diferencia en la
curva más cerrada de Monza es del orden de 0.03 grados.

Restricciones comunes, |δ[k]| ≤ δmax y |δ[k] - δ[k-1]| ≤ Δδmax Ts, I5.

I4.3 El problema condensado es un QP en δ[0..N-1] resuelto con OSQP.
I4.4 Si OSQP no devuelve solved, se aplica la secuencia anterior desplazada.
I6.2 OSQP tiene un tiempo máximo por ciclo, 40 ms por defecto.

En el registro, extra1 es el tiempo de ejecución que reporta OSQP en ms y
extra2 el valor del costo.
"""
import numpy as np
import osqp
import scipy.sparse as sp

from controladores.base import ControladorLateral, SalidaControlador


class MPCCinematico(ControladorLateral):
    nombre = "mpc_cinematico"

    def __init__(self, parametros, ts_s, delta_max_rad, tasa_max_rad_s):
        super().__init__(parametros)
        p = self.parametros
        self.N = int(p["N"])
        if self.N < 2:
            raise ValueError("N debe ser al menos 2")
        self.Qy = float(p.get("Qy", 1.0))
        self.Qpsi = float(p["Qpsi"])
        self.Rd = float(p["Rdelta"])
        self.Rdd = float(p["Rddelta"])
        self.tiempo_max = float(p.get("tiempo_max_s", 0.04))
        self.ts = float(ts_s)
        self.dmax = float(delta_max_rad)
        self.paso = float(tasa_max_rad_s) * self.ts

        N = self.N
        self.D = np.eye(N) - np.eye(N, k=-1)
        self.DtD = self.D.T @ self.D
        self.Qdiag = np.tile([self.Qy, self.Qpsi], N)
        self.A_rest = sp.csc_matrix(np.vstack([np.eye(N), self.D]))

        # Patrón fijo del triángulo superior completo de P, orden por columnas.
        indptr, indices = [0], []
        for j in range(N):
            indices.extend(range(j + 1))
            indptr.append(len(indices))
        self._indices = np.array(indices, dtype=np.int32)
        self._indptr = np.array(indptr, dtype=np.int32)

        self.prob = None
        self.secuencia = np.zeros(N)
        self.fallos_consecutivos = 0

    def reiniciar(self):
        self.prob = None
        self.secuencia = np.zeros(self.N)
        self.fallos_consecutivos = 0

    def _datos_triu(self, H):
        return np.concatenate([H[: j + 1, j] for j in range(self.N)])

    def _prediccion(self, v, L, kappas):
        N, ts = self.N, self.ts
        A = np.array([[1.0, ts * v], [0.0, 1.0]])
        b = np.array([0.0, ts * v / L])
        Sx = np.zeros((2 * N, 2))
        Su = np.zeros((2 * N, N))
        c = np.zeros(2 * N)
        Mx = np.eye(2)
        Mu = np.zeros((2, N))
        mc = np.zeros(2)
        for k in range(N):
            Mx = A @ Mx
            Mu = A @ Mu
            Mu[:, k] += b
            mc = A @ mc + np.array([0.0, -ts * v * kappas[k]])
            Sx[2 * k:2 * k + 2] = Mx
            Su[2 * k:2 * k + 2] = Mu
            c[2 * k:2 * k + 2] = mc
        return Sx, Su, c

    def matrices_qp(self, e):
        """Devuelve P denso, q, l y u del QP para una entrada. Útil en pruebas."""
        N = self.N
        # Velocidad real, sin piso. Con el vehículo detenido la dirección sale
        # del modelo, el término de seguimiento se anula y δ queda en el ángulo
        # de la curva. Un piso de 1 m/s hacía que el MPC creyera que avanzaba y
        # enrollara la dirección hasta el tope, verificado en pista el 2026-09-15.
        v = max(e.v_ms, 0.0)
        L = e.L
        kappas = np.array([e.trazada.curvatura_adelante(e.idx_tras, v * self.ts * k)
                           for k in range(N)])
        u_ff = np.arctan(L * kappas)
        Sx, Su, c = self._prediccion(v, L, kappas)
        x_libre = Sx @ np.array([e.e_y_tras, e.e_psi_tras]) + c
        SuQ = Su.T * self.Qdiag
        H = 2.0 * (SuQ @ Su + self.Rd * np.eye(N) + self.Rdd * self.DtD)
        d0 = np.zeros(N)
        d0[0] = e.delta_prev
        q = 2.0 * (SuQ @ x_libre - self.Rd * u_ff - self.Rdd * (self.D.T @ d0))
        lim = np.full(N, self.dmax)
        l = np.concatenate([-lim, d0 - self.paso])
        u = np.concatenate([lim, d0 + self.paso])
        return H, q, l, u

    def calcular(self, e):
        N = self.N
        H, q, l, u = self.matrices_qp(e)
        Px = self._datos_triu(H)
        if self.prob is None:
            P = sp.csc_matrix((Px, self._indices, self._indptr), shape=(N, N))
            self.prob = osqp.OSQP()
            self.prob.setup(P, q, self.A_rest, l, u, verbose=False, warm_starting=True,
                            polishing=False, time_limit=self.tiempo_max,
                            eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
        else:
            self.prob.update(Px=Px, q=q, l=l, u=u)

        candidata = np.append(self.secuencia[1:], self.secuencia[-1])
        self.prob.warm_start(x=candidata)
        res = self.prob.solve()
        estado = str(res.info.status)
        x = res.x
        if estado == "solved" and x is not None and np.all(np.isfinite(x)):
            self.secuencia = np.array(x, dtype=float)
            respaldo = 0
            self.fallos_consecutivos = 0
        else:
            self.secuencia = candidata
            respaldo = 1
            self.fallos_consecutivos += 1
        return SalidaControlador(delta=float(self.secuencia[0]), estado=estado,
                                 iteraciones=int(res.info.iter), respaldo=respaldo,
                                 extra1=float(res.info.run_time) * 1000.0,
                                 extra2=float(res.info.obj_val))
