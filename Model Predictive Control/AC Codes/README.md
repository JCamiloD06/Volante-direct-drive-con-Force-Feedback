# Localización del auto para MPC en Assetto Corsa — Monza

Este módulo forma parte de un proyecto de conducción autónoma sobre un
volante casero con force-feedback, usando Assetto Corsa (Monza) como
simulador. El objetivo de esta parte del proyecto es resolver un problema
concreto: **saber en todo momento dónde está el auto respecto a la pista**,
en un formato que un controlador MPC (Model Predictive Control) pueda usar
directamente.

Para eso se necesitan dos cosas, y cada una tiene su propio script:

1. **Una trayectoria de referencia** — la línea ideal de Monza, con
   coordenadas conocidas. → `parse_fast_lane.py`
2. **La posición del auto en tiempo real**, comparada contra esa
   referencia. → `mpc_localization.py`

---

## Índice

- [Contexto general](#contexto-general)
- [Requisitos](#requisitos)
- [1. `parse_fast_lane.py` — extracción de la trayectoria de referencia](#1-parse_fast_lanepy--extracción-de-la-trayectoria-de-referencia)
- [2. `mpc_localization.py` — localización del auto en tiempo real](#2-mpc_localizationpy--localización-del-auto-en-tiempo-real)
- [El estado que le llega al MPC](#el-estado-que-le-llega-al-mpc)
- [Limitaciones conocidas](#limitaciones-conocidas)

---

## Contexto general

Un MPC de seguimiento de trayectoria no trabaja con coordenadas absolutas
(x, z) — trabaja con **error respecto al camino de referencia**: qué tan
lejos lateralmente está el auto de la línea ideal, qué tan girado está
respecto a la dirección de la pista, y cuánto va a curvar la pista en los
próximos metros. Estos dos scripts arman esa información de punta a punta:

```
fast_lane.ai (archivo del juego)
        │
        ▼
parse_fast_lane.py  →  CSV con la trayectoria de referencia (x, z, distancia acumulada)
        │
        ▼
mpc_localization.py (en vivo, con AC corriendo)
        │
        ├─ lee posición real del auto (Shared Memory API de AC)
        ├─ la proyecta sobre la trayectoria de referencia
        └─ calcula e_y, e_psi, s, curvatura → esto es lo que consume el MPC
```

## Requisitos

- Python 3.10+
- `numpy`
- Para `mpc_localization.py`: **Windows**, con Assetto Corsa instalado y
  corriendo (la Shared Memory de AC no existe en Linux/Mac)

```bash
pip install numpy
```

---

## 1. `parse_fast_lane.py` — extracción de la trayectoria de referencia

### Qué hace

Lee el archivo binario `fast_lane.ai` que Assetto Corsa trae para cada
pista (es la línea que usan los autos de IA) y lo convierte a un CSV
simple con las coordenadas de cada punto del trazado.

Se corre **una sola vez por pista**, offline, sin necesitar el juego
abierto — es un archivo que ya está en tu instalación del juego.

### Dónde encontrar el archivo de entrada

```
steamapps/common/assettocorsa/content/tracks/<nombre_pista>/ai/fast_lane.ai
```

Si la pista tiene varios trazados (por ejemplo Monza / Monza 66 / Monza
Junior), puede haber una subcarpeta por layout dentro de `tracks/<pista>/`.

### Cómo se descifró el formato

`fast_lane.ai` es un archivo binario sin documentación oficial pública. El
formato se determinó por ingeniería inversa empírica: se leyeron los bytes
crudos del archivo, se probaron distintas hipótesis de estructura
(tamaño de header, tipos de dato, tamaño de cada registro), y se validó
cada hipótesis contra propiedades físicas conocidas del circuito real
(longitud total, forma del trazado, si cierra o no como circuito). El
proceso completo, con el detalle de cómo se fue descartando cada hipótesis
incorrecta, está documentado en el historial de conversación del proyecto.

### Formato del archivo (resultado)

```
Header (16 bytes):
    int32  version         (ej. 7)
    int32  count           (cantidad de puntos, ej. 3750 para Monza)
    int32  padding         (0)
    int32  padding         (0)

Luego, `count` registros de 20 bytes cada uno:
    float32  x                     (metros, coordenadas mundo de AC)
    float32  y_elevation           (metros, altura/elevación)
    float32  z                     (metros, coordenadas mundo de AC)
    float32  cumulative_length_m   (metros recorridos desde el punto 0)
    int32    point_index           (índice secuencial, 0..count-1)
```

> El archivo tiene datos adicionales después de este bloque (velocidad
> sugerida, peralte, ancho de pista, etc.) que este script no extrae
> porque no hacen falta para la trayectoria de referencia del MPC.

### Uso

```bash
python parse_fast_lane.py fast_lane.ai monza_fast_lane.csv
```

Salida en consola (verificación automática de que el parseo salió bien):

```
Version del archivo: 7
  Puntos: 3750
  Distancia entre puntos: min=1.46 m, max=4.66 m, promedio=1.53 m
  Distancia de cierre (primer vs ultimo punto): 1.55 m
  Longitud total (segun 'cumulative_length'): 5757.2 m
  (compara esta longitud contra la ficha oficial del circuito...)

CSV guardado en: monza_fast_lane.csv
```

Estos tres chequeos son la forma de confirmar que el parseo es correcto
sin necesitar abrir el juego:

| Chequeo | Qué confirma |
|---|---|
| Distancia entre puntos consecutivos | Que no hay puntos corruptos ni saltos (debería ser regular, ~1.5 m para Monza) |
| Distancia de cierre (primer vs. último punto) | Que el circuito es efectivamente un lazo cerrado |
| Longitud total vs. longitud oficial del circuito | Que la escala/unidades del parseo son correctas (Monza: 5793 m oficiales, ~5757 m en la línea de IA — la diferencia es normal, la IA no siempre toca el borde exacto de cada curva) |

### CSV de salida

| Columna | Descripción |
|---|---|
| `index` | Índice secuencial del punto (0 a N-1) |
| `x` | Coordenada X, metros, sistema de coordenadas mundo de AC |
| `y_elevation` | Elevación (altura), metros |
| `z` | Coordenada Z, metros, sistema de coordenadas mundo de AC |
| `cumulative_length_m` | Distancia acumulada desde el punto 0, metros |

---

## 2. `mpc_localization.py` — localización del auto en tiempo real

### Qué hace

Con Assetto Corsa corriendo y una sesión activa en pista, este script:

1. Lee la posición del auto en cada frame vía la **Shared Memory API**
   de Assetto Corsa (`acpmf_physics`, `acpmf_graphics`).
2. Proyecta esa posición sobre la trayectoria de referencia (el CSV
   generado por `parse_fast_lane.py`).
3. Calcula el estado que necesita el MPC: error lateral, error de
   orientación, distancia recorrida y curvatura futura de la pista.
4. Imprime ese estado en vivo y lo guarda en un CSV de log por sesión.

No requiere ningún paquete de terceros para leer la memoria compartida:
usa `ctypes` + `mmap` directo sobre los bloques de memoria que expone el
juego, siguiendo el SDK oficial y público de Assetto Corsa para apps de
terceros.

### Shared Memory API — qué se lee

| Bloque | Campos usados | Para qué |
|---|---|---|
| `SPageFilePhysics` | `velocity` (x,y,z), `speedKmh`, `steerAngle`, `gas`, `brake` | Velocidad del auto y estado de los pedales/volante |
| `SPageFileGraphic` | `carCoordinates` (x,y,z), `packetId` | Posición del auto y detección de frames duplicados |

### `car_heading`: por qué se calcula así

Assetto Corsa expone un campo `heading` en `SPageFilePhysics`, pero **no
se usa directamente** porque su convención de ejes no coincide con la que
usamos para las coordenadas de la pista (se detectó un offset sistemático
de ~90° al validarlo contra datos reales de manejo).

En cambio, `car_heading` se calcula a partir del **vector velocidad**:

```python
car_heading = atan2(velocity_z, velocity_x)
```

Esto funciona bien porque tanto `velocity` como `carCoordinates` están en
el mismo sistema de coordenadas mundo — no hace falta calibrar ningún
offset. La contrapartida (ver [Limitaciones conocidas](#limitaciones-conocidas))
es que en curvas fuertes a velocidad, con deslizamiento de los neumáticos,
el vector velocidad deja de coincidir exactamente con hacia dónde apunta
la carrocería del auto.

### Cálculo de `e_y` y `e_psi` (marco de Frenet)

Para cada posición del auto, `ReferencePath` busca el punto más cercano
de la trayectoria de referencia y proyecta el error:

- **`e_y`** (cross-track error): distancia perpendicular del auto a la
  línea de referencia, en metros. Positivo = auto a la izquierda de la
  línea.
- **`e_psi`** (heading error): diferencia angular entre hacia dónde
  apunta el auto y hacia dónde apunta la trayectoria en ese punto,
  en radianes.
- **`s`**: distancia acumulada recorrida sobre la trayectoria, en metros.
- **`curvatura`**: calculada como la derivada del heading de la
  trayectoria respecto al arco (`dθ/ds`), muestreada en un horizonte de
  metros hacia adelante — es lo que le permite al MPC anticipar una
  curva antes de llegar a ella.

La búsqueda del punto más cercano usa una ventana local alrededor del
último punto encontrado (el auto se mueve de forma continua, así que es
mucho más rápido que buscar en los 3750 puntos cada vez), con una
búsqueda global de respaldo si el auto "salta" más de 20 m de golpe
(por ejemplo, al arrancar o después de un reinicio de sesión).

### Uso

1. Poné `mpc_localization.py` y el CSV de la pista (generado con
   `parse_fast_lane.py`) en la misma carpeta.
2. Abrí Assetto Corsa, cargá la pista, metete en pista (Practice o
   Hotlap).
3. Corré:

```bash
python mpc_localization.py
```

Vas a ver una línea nueva por frame:

```
{'car_x': -206.93, 'car_z': 414.0, 'e_y_m': 0.12, 'e_psi_rad': -0.004, 's_m': 1893.4, 'v_ms': 47.2, 'steer_deg': 3.1, 'gas': 0.8, 'brake': 0.0, 'kappa_next_10m_avg': 0.0021}
```

Al terminar (`Ctrl+C`), se cierra un archivo `session_log_<fecha>_<hora>.csv`
con el historial completo de la sesión.

### CSV de log — columnas

| Columna | Descripción |
|---|---|
| `t_s` | Tiempo transcurrido desde que arrancó el script, segundos |
| `car_x`, `car_y_elevation`, `car_z` | Posición cruda del auto, mismo sistema de coordenadas que la pista |
| `e_y_m` | Error lateral respecto a la línea de referencia, metros |
| `e_psi_rad` | Error de orientación respecto a la pista, radianes |
| `s_m` | Distancia recorrida sobre la trayectoria de referencia, metros |
| `v_ms` | Velocidad del auto, m/s |
| `steer_angle_rad`, `steer_angle_deg` | Ángulo de dirección del auto |
| `gas`, `brake` | Posición de los pedales, 0 a 1 |
| `kappa_next_10m_avg` | Curvatura promedio de la pista en los próximos ~10 m |

### Filtro de frames duplicados

El script chequea `packetId` en cada ciclo: si no cambió respecto al
frame anterior (el juego está pausado, en un menú, o hubo un Alt+Tab), el
frame se descarta y no se loguea — evita llenar el CSV de filas
idénticas repetidas.

---

## El estado que le llega al MPC

Resumiendo, el vector de estado que produce este pipeline (y que un MPC
de seguimiento de trayectoria consume típicamente) es:

```
[e_y, e_psi, v]   +   curvatura de referencia en el horizonte de predicción
```

En vez de "dónde está el auto en el mapa", el MPC recibe "qué tan
descentrado y desalineado está respecto al camino que tiene que seguir" —
que es el formato natural para plantear el problema de control.

## Limitaciones conocidas

- **`car_heading` desde el vector velocidad no es confiable en curvas
  fuertes a velocidad.** El vector velocidad coincide con la orientación
  real de la carrocería solo cuando el deslizamiento de los neumáticos es
  bajo (recta, curvas suaves). En frenadas fuertes o curvas tomadas al
  límite, el ángulo de deslizamiento contamina esta estimación de
  heading. Para uso de solo-registro/análisis (que es lo que hace este
  script) esto no es crítico, pero **no se recomienda usar este mismo
  cálculo de heading para retroalimentar un controlador en lazo
  cerrado** sin antes agregar una calibración contra el campo `heading`
  nativo de la física de AC (que sí representa la orientación real de la
  carrocería, inmune al deslizamiento).
- **Resolución del CSV de referencia**: el punto más cercano de la
  trayectoria está, en promedio, a ~1.5 m del punto anterior/siguiente
  (para Monza). Esto pone un piso natural a la precisión de `e_y`
  (validado empíricamente: error promedio ~11 cm, máximo ~47 cm contra
  el cálculo de distancia mínima por fuerza bruta).
- **Primeros frames con el auto detenido**: con velocidad ~0, el vector
  velocidad no tiene una dirección confiable, así que `e_psi` no es
  significativo hasta que el auto arranca a moverse.
