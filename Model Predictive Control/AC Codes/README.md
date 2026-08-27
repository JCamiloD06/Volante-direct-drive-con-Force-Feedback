# Localización del auto para MPC en Assetto Corsa

Este módulo forma parte de un proyecto de conducción autónoma sobre un
volante casero con force-feedback, usando Assetto Corsa como simulador
(desarrollado y validado sobre Monza, pensado para extenderse a
cualquier pista). El objetivo de esta parte del proyecto es resolver un
problema concreto: **saber en todo momento dónde está el auto respecto a
la pista**, en un formato que un controlador MPC (Model Predictive
Control) pueda usar directamente.

Para eso se necesitan dos cosas:

1. **Una trayectoria de referencia** — la línea ideal de la pista, con
   coordenadas conocidas.
2. **La posición del auto en tiempo real**, comparada contra esa
   referencia.

| Script | Qué hace | Cuándo se usa |
|---|---|---|
| **`extract_track.py`** | Herramienta interactiva: pide la ruta al `fast_lane.ai`, lo copia al proyecto, genera el CSV de la trayectoria y el PNG de verificación. Soporta procesar varias pistas en una sola corrida. | Una vez por pista, offline, sin el juego abierto |
| **`mpc_localization.py`** (Localización auto) | Lee la posición del auto en vivo (Shared Memory de AC) y la compara contra el CSV de referencia: calcula `e_y`, `e_psi`, `s`, curvatura. | En cada sesión de manejo, con AC corriendo |

---

## Índice

- [Contexto general](#contexto-general)
- [Requisitos](#requisitos)
- [1. `extract_track.py`](#1-extract_trackpy)
- [2. Localización auto (`mpc_localization.py`)](#2-localización-auto-mpc_localizationpy)
- [El estado que le llega al MPC](#el-estado-que-le-llega-al-mpc)
- [Limitaciones conocidas](#limitaciones-conocidas)

---

## Contexto general

Un MPC de seguimiento de trayectoria no trabaja con coordenadas absolutas
(x, z) — trabaja con **error respecto al camino de referencia**: qué tan
lejos lateralmente está el auto de la línea ideal, qué tan girado está
respecto a la dirección de la pista, y cuánto va a curvar la pista en los
próximos metros. Este pipeline arma esa información de punta a punta:

```
fast_lane.ai (archivo del juego, una copia por pista)
        │
        ▼
extract_track.py  →  tracks_data/<pista>/<pista>_fast_lane.csv
                  →  tracks_data/<pista>/<pista>_verificacion.png
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
- `matplotlib` (para el PNG de verificación)
- Para `mpc_localization.py`: **Windows**, con Assetto Corsa instalado y
  corriendo (la Shared Memory de AC no existe en Linux/Mac)

```bash
pip install numpy matplotlib
```

> Si tu Python tira `error: externally-managed-environment` al instalar,
> agregá `--break-system-packages` al final del comando, o mejor, armá un
> entorno virtual (`python -m venv venv`, después `venv\Scripts\activate`
> en Windows) para no volver a lidiar con esto en cada paquete nuevo.

---

## 1. `extract_track.py`

### Qué hace

Es la forma recomendada de sacar la trayectoria de una pista: un solo
script interactivo que evita tener que copiar el `fast_lane.ai` a mano y
correr dos comandos por separado.

1. Te pide que pegues la ruta completa al `fast_lane.ai` de la pista
   (la ruta la sacás de la carpeta de instalación de Steam, ver más
   abajo).
2. **Copia** ese archivo a la carpeta del proyecto — nunca modifica ni
   escribe nada dentro de la carpeta del juego, solo lee de ahí.
3. Detecta automáticamente el nombre de la pista a partir de la
   estructura de carpetas de Assetto Corsa
   (`.../content/tracks/<pista>/[<layout>/]ai/fast_lane.ai`). Si la
   ruta no tiene esa estructura reconocible (por ejemplo, si ya habías
   copiado el archivo a otro lado con otro nombre), te pregunta el
   nombre de la pista directamente.
4. Genera el CSV de la trayectoria **y** el PNG de verificación, los
   dos nombrados con el nombre de la pista.
5. Te pregunta si querés procesar otra pista, así podés hacer varias
   en una sola corrida del script.

### Organización de los archivos generados

Para no pisar los datos de una pista con los de otra, todo queda
ordenado en subcarpetas por pista:

```
tracks_data/
    spa/
        fast_lane.ai              <- copia del original (para referencia/backup)
        spa_fast_lane.csv
        spa_verificacion.png
    monza/
        fast_lane.ai
        monza_fast_lane.csv
        monza_verificacion.png
    silverstone_gp/                <- si la pista tiene layouts, se agrega al nombre
        fast_lane.ai
        silverstone_gp_fast_lane.csv
        silverstone_gp_verificacion.png
```

### Uso

```bash
python extract_track.py
```

```
============================================================
 Extractor de trazados de Assetto Corsa (fast_lane.ai)
============================================================
Todo se va a guardar organizado dentro de: ./tracks_data/<pista>/

Pega la ruta completa al archivo fast_lane.ai: C:\Program Files (x86)\Steam\steamapps\common\assettocorsa\content\tracks\spa\ai\fast_lane.ai
  Pista detectada automaticamente: spa
  Copiado a: tracks_data/spa/fast_lane.ai
  Version del archivo: 7
  Puntos: 4470
  Distancia entre puntos: min=1.12 m, max=3.98 m, promedio=1.55 m
  Distancia de cierre (primer vs ultimo punto): 2.03 m
  Longitud total (segun 'cumulative_length'): 6934.1 m
  -> Compara esta longitud contra la ficha oficial del circuito.
     Una diferencia de hasta ~1% es normal.
  CSV guardado en: tracks_data/spa/spa_fast_lane.csv
  PNG de verificacion guardado en: tracks_data/spa/spa_verificacion.png

¿Queres procesar otra pista? (s/n): n

Listo.
```

### Dónde encontrar la ruta al `fast_lane.ai`

En Steam: click derecho en Assetto Corsa → **Administrar → Explorar
archivos locales**, y desde ahí navegar a:

```
content/tracks/<nombre_de_carpeta_de_la_pista>/ai/fast_lane.ai
```

Los nombres de carpeta no siempre coinciden con el nombre que se ve en
el juego (pueden tener prefijos como `ks_`, o nombres distintos). Si la
pista tiene varios trazados (GP, corto, nacional, etc.), puede haber
una subcarpeta de layout entre el nombre de la pista y `ai/`.

Podés pegar la ruta con o sin comillas (si la copiaste con "Copiar como
ruta de acceso" en Windows, viene con comillas — el script las saca
solo).

### Sobre la orientación del gráfico de verificación

El PNG generado puede verse "rotado" o incluso como en espejo respecto
a un diagrama de referencia que encuentres en internet (por ejemplo, un
póster o un artículo de Wikipedia). **Esto es normal y no indica un
error.** Cada pista en Assetto Corsa tiene su propio sistema de
coordenadas interno, orientado de forma arbitraria según cómo la
construyeron los artistas 3D — no tiene ninguna relación con el norte
geográfico real ni con la orientación que eligió quien dibujó el
diagrama de referencia que estés mirando.

Lo que hay que verificar no es la rotación, sino que la **secuencia de
curvas** coincida: seguí el trazado desde el punto verde (inicio/meta)
y confirmá que las curvas aparecen en el mismo orden que en el circuito
real, aunque el dibujo esté girado o parezca invertido. Esto no afecta
en nada al resto del pipeline: `e_y`, `e_psi` y la curvatura se calculan
siempre de forma relativa a la propia geometría de la pista, sin
importar su orientación.

---

### Cómo se descifró el formato de `fast_lane.ai`

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

Estos son los tres chequeos que `extract_track.py` imprime en consola
para confirmar que el parseo salió bien, sin necesitar abrir el juego:

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

## 2. Localización auto (`mpc_localization.py`)

### Qué hace

Con Assetto Corsa corriendo y una sesión activa en pista, este script:


1. Lee la posición del auto en cada frame vía la **Shared Memory API**
   de Assetto Corsa (`acpmf_physics`, `acpmf_graphics`).
2. Proyecta esa posición sobre la trayectoria de referencia (el CSV
   generado por `extract_track.py`).
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
   `extract_track.py`, dentro de `tracks_data/<pista>/`) en la misma
   carpeta — o ajustá la ruta que le pasás al script si preferís
   dejarlo donde `extract_track.py` lo guardó.
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
