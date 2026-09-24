"""Controladores laterales de P00 y fábrica común."""
from controladores.mpc_cinematico import MPCCinematico
from controladores.pure_pursuit import PurePursuit
from controladores.stanley import Stanley

NOMBRES = (PurePursuit.nombre, Stanley.nombre, MPCCinematico.nombre)


def crear_controlador(nombre, cfg):
    parametros = cfg["controladores"][nombre]
    if nombre == PurePursuit.nombre:
        return PurePursuit(parametros)
    if nombre == Stanley.nombre:
        return Stanley(parametros)
    if nombre == MPCCinematico.nombre:
        return MPCCinematico(parametros, ts_s=cfg["ts_s"],
                             delta_max_rad=cfg["direccion"]["delta_max_rad"],
                             tasa_max_rad_s=cfg["direccion"]["tasa_max_rad_s"])
    raise ValueError(f"controlador desconocido {nombre}")
