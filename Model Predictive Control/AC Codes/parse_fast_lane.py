"""
parse_fast_lane.py

Extrae la trayectoria de referencia (centerline / linea de IA) de un
circuito de Assetto Corsa a partir de su archivo binario fast_lane.ai,
sin depender de ninguna app de Content Manager ni de terceros.

UBICACION del archivo fuente en tu instalacion de Steam:
    steamapps/common/assettocorsa/content/tracks/<nombre_pista>/ai/fast_lane.ai

Si la pista tiene varios layouts (ej. Monza / Monza 66 / Monza Junior),
puede haber una subcarpeta por layout dentro de tracks/<pista>/.

USO:
    python parse_fast_lane.py fast_lane.ai salida.csv

FORMATO DEL ARCHIVO (descifrado por ingenieria inversa empirica, ver
seccion de verificacion mas abajo):

    Header (16 bytes):
        int32  version        (ej. 7)
        int32  count          (cantidad de puntos, ej. 3750 para Monza)
        int32  padding        (0)
        int32  padding        (0)

    Luego, `count` registros de 20 bytes cada uno:
        float32  x                    (metros, coordenadas mundo de AC)
        float32  y_elevation          (metros, altura/elevacion)
        float32  z                    (metros, coordenadas mundo de AC)
        float32  cumulative_length_m  (metros recorridos desde el punto 0)
        int32    point_index          (indice secuencial del punto, 0..count-1)

Nota: el archivo tiene datos adicionales despues de este bloque principal
(velocidad sugerida, peralte, ancho de pista, etc. -- no se necesitan
para la trayectoria de referencia del MPC) que este script no extrae.
"""

import struct
import csv
import sys
import math


def parse_fast_lane(path):
    with open(path, "rb") as f:
        data = f.read()

    version, count = struct.unpack_from("<2i", data, 0)
    header_size = 16
    stride = 20  # bytes por punto: 4 floats + 1 int32

    points = []
    for i in range(count):
        x, y, z, length, idx = struct.unpack_from(
            "<4fi", data, header_size + i * stride
        )
        points.append((idx, x, y, z, length))

    return version, points


def verify(points):
    """
    Chequeos de sanidad basicos, para confirmar que el parseo esta bien
    antes de confiar en los datos:
      1. Distancia entre puntos consecutivos: no deberia haber saltos
         gigantes (puntos corruptos) ni ceros (puntos duplicados).
      2. Cierre del circuito: el primer y ultimo punto deberian estar
         cerca entre si si es un trazado cerrado.
      3. Longitud total vs. lo que dice la ficha oficial del circuito.
    """
    dists = []
    for i in range(len(points) - 1):
        _, x1, y1, z1, _ = points[i]
        _, x2, y2, z2, _ = points[i + 1]
        d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        dists.append(d)

    idx0, x0, y0, z0, _ = points[0]
    idxn, xn, yn, zn, _ = points[-1]
    closure_dist = math.sqrt((xn - x0) ** 2 + (yn - y0) ** 2 + (zn - z0) ** 2)

    print(f"  Puntos: {len(points)}")
    print(f"  Distancia entre puntos: min={min(dists):.2f} m, "
          f"max={max(dists):.2f} m, promedio={sum(dists)/len(dists):.2f} m")
    print(f"  Distancia de cierre (primer vs ultimo punto): {closure_dist:.2f} m")
    print(f"  Longitud total (segun 'cumulative_length'): {points[-1][4]:.1f} m")
    print("  (compara esta longitud contra la ficha oficial del circuito "
          "para confirmar que el parseo esta bien; una diferencia de "
          "hasta ~1% es normal, la linea de IA no siempre toca el borde "
          "exacto de la pista en cada curva)")


def main():
    if len(sys.argv) != 3:
        print("Uso: python parse_fast_lane.py <entrada fast_lane.ai> <salida.csv>")
        sys.exit(1)

    in_path, out_path = sys.argv[1], sys.argv[2]

    version, points = parse_fast_lane(in_path)
    print(f"Version del archivo: {version}")
    verify(points)

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y_elevation", "z", "cumulative_length_m"])
        for idx, x, y, z, length in points:
            writer.writerow([idx, x, y, z, length])

    print(f"\nCSV guardado en: {out_path}")


if __name__ == "__main__":
    main()
