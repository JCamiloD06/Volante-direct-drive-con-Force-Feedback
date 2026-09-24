"""Utilidades de ángulos."""
import math


def envolver(angulo):
    """Lleva un ángulo en radianes al intervalo de menos pi a pi."""
    return (angulo + math.pi) % (2.0 * math.pi) - math.pi
