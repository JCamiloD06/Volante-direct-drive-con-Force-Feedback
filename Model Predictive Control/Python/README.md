# Python — Implementación en tiempo real del MPC (Monza, Assetto Corsa)

Esta carpeta contiene la implementación en Python del controlador MPC (Model Predictive Control) que conduce el carro de forma autónoma en Assetto Corsa, en el circuito de Monza. A diferencia de la carpeta `Matlab` (donde se diseña, simula y valida el modelo del MPC), aquí el MPC corre en tiempo real, leyendo la posición del carro directamente desde Assetto Corsa y comandando el volante (simulado o físico) para seguir la trazada.

## Archivos de la carpeta

| Archivo | Tipo | Descripción |
|---------|------|-------------|
| `mpc_monza_FINAL.py` | Script principal | Implementación completa del MPC en tiempo real. |
| `monza_fast_lane.csv` | Datos de entrada | Trazada de referencia de Monza (fast lane), usada por el MPC como camino a seguir. |
| `mpc_session_*.csv` | Datos de salida | Log generado automáticamente en cada ejecución, con el detalle ciclo a ciclo de la sesión de manejo. |

## Requisitos

```
pip install numpy scipy pyvjoy
```

Para usar el volante físico (Fase B) además se necesita:

```
pip install pysdl2 pysdl2-dll
```

Adicionalmente:

- Assetto Corsa corriendo, con una sesión activa en pista (no en menú).
- vJoy instalado, con el **Device 1** configurado como el eje de dirección (steering) en Assetto Corsa.
- El archivo `monza_fast_lane.csv` en la misma carpeta que el script.

## Cómo se ejecuta

```
python mpc_monza_FINAL.py               # Fase A: planta simulada (sin volante físico) + vJoy
python mpc_monza_FINAL.py --ffbeast      # Fase B: motor real del volante (FFBeast) + vJoy
python mpc_monza_FINAL.py --calibrate    # Modo de calibración manual del FFBeast
```

- **Fase A** sirve para probar y ajustar el MPC sin arriesgar el hardware: el "volante" es puramente matemático.
- **Fase B** usa el volante físico real (motor de hoverboard + MKS ODrive con firmware FFBeast) como planta.
- **`--calibrate`** aplica torques pequeños y crecientes al volante físico para verificar el signo de giro y la relación Nm→magnitud antes de confiar en el MPC con el motor conectado.

## Nota importante sobre el hardware (FFBeast, no ODrive nativo)

El MKS ODrive Mini de este proyecto corre **firmware FFBeast**, no el firmware nativo de ODrive. Esto es clave para entender el código: FFBeast expone el volante a Windows como un **dispositivo HID de Force Feedback (DirectInput)**, no como un dispositivo ODrive por protocolo serial.

Por eso, en este script el torque **no** se manda con algo como `odrive.axis0.controller.input_torque` (como se haría con un ODrive nativo). En su lugar, se manda como un efecto de **"Constant Force"** vía SDL2 haptics (`SDL_HapticUpdateEffect`). La magnitud que se le manda a SDL2 (rango ±32767) **no es Nm directo**: es un valor normalizado que FFBeast escala internamente según el límite de fuerza configurado en su propia app. Ese factor de conversión (`nm_to_magnitude`) hay que calibrarlo empíricamente — no es un valor universal.

vJoy se mantiene como intermediario: Assetto Corsa lee el eje virtual de vJoy, no el eje físico de FFBeast directamente. El flujo completo es:

```
MPC calcula τ  →  FFBeast aplica ese torque al motor  →  se LEE la posición
real resultante del volante físico (SDL2)  →  esa posición real se manda a
vJoy  →  Assetto Corsa mueve el carro según lo que el volante físico
realmente hizo.
```

## Arquitectura del código, sección por sección

### 1. SDK oficial de Assetto Corsa (Shared Memory)

`SPageFilePhysics` y `SPageFileGraphic` son estructuras de C (via `ctypes`) que reflejan exactamente el layout de memoria que Assetto Corsa expone en tiempo real (`acpmf_physics` y `acpmf_graphics`). La clase `ACSharedMemory` abre esa memoria compartida y permite leer, en cada ciclo, datos como la posición del carro (`carCoordinates`), su velocidad (`speedKmh`, `velocity`), y otros datos físicos, sin necesidad de ningún plugin adicional en AC.

### 2. Trazada de Monza y localización (coordenadas Frenet)

La clase `ReferencePath` carga `monza_fast_lane.csv` (una secuencia de puntos `x, z` con su longitud acumulada) y calcula, para cada punto, el heading (dirección tangente) y la curvatura de la pista.

Su función principal, `frenet()`, ubica al carro sobre la trazada y calcula:

- **`e_y`**: error lateral (qué tan lejos está el carro de la línea ideal, en metros).
- **`e_psi`**: error de heading (qué tan girado está el carro respecto a la dirección de la pista, en radianes).

Esto convierte la posición absoluta del carro (x, z) en un error relativo a la trazada, que es lo que el controlador necesita corregir.

`curvature_at_offset()` además permite mirar la curvatura de la pista **más adelante** del punto actual — esto es lo que le da al MPC una "vista previa" del trazado (preview), fundamental para que anticipe curvas en vez de reaccionar tarde.

### 3. Generador de referencia (Pure Pursuit + preview de curvatura)

`ReferenceGenerator` construye la secuencia de ángulos de volante deseados (`theta_ref`) para todo el horizonte de predicción del MPC (no solo el instante actual). Combina dos ingredientes:

- **Corrección tipo Pure Pursuit** en el primer paso (k=0), usando el error lateral y de heading medidos.
- **Feedforward de curvatura** en los pasos siguientes (k>0): como no hay forma de medir el error futuro real (eso requeriría un modelo completo del vehículo), se usa la curvatura de la pista en el punto donde se espera que esté el carro, combinada con un decaimiento muy suave de la corrección actual (`error_decay`).

**Detalle importante documentado en el propio código:** el decaimiento se dejó deliberadamente lento (`error_decay=0.998`). En pruebas, con un decaimiento más agresivo (`0.75`) y un peso final `Qf` mayor que `Q`, el MPC terminaba priorizando el objetivo de "error casi cero" al final del horizonte y dejaba de corregir errores laterales grandes y sostenidos — el volante se quedaba corto de ángulo justo cuando más se necesitaba girar.

También existe un parámetro `steer_sign` (±1) que compensa que el sistema de coordenadas de Assetto Corsa no necesariamente coincide con la convención matemática usada en `ReferencePath.frenet()`. Si el signo está mal, el sistema no da errores ni se cae, pero corrige en la dirección contraria: el error crece en vez de reducirse y el volante se satura casi de inmediato — ese es el síntoma característico de un lazo de control con el signo invertido.

### 4. El MPC real (horizonte + optimización)

`MPCSteeringController` es el corazón del sistema. Modela el volante como un sistema de segundo orden discreto (doble integrador con fricción viscosa):

```
x = [theta, theta_dot]
x_(k+1) = A·x_k + B·u_k
```

donde `A` y `B` salen de discretizar `J·θ̈ = τ - b·θ̇` (inercia `J`, fricción `b`, torque `u=τ`).

En cada ciclo, resuelve un problema de optimización (QP, vía `scipy.optimize.minimize` con el método SLSQP) sobre un horizonte de `N` pasos, minimizando:

- El error entre el ángulo predicho y el ángulo de referencia (peso `Q`, y `Qf` para el último paso del horizonte).
- El esfuerzo de torque (`R`) y su tasa de cambio (`Rd`, para evitar comandos bruscos).

Sujeto a restricciones de torque máximo (`tau_max`), tasa de cambio máxima (`rate_max`) y ángulo máximo del volante (`theta_max`).

De toda la secuencia óptima de N torques, **solo se aplica el primero** (`u_0`). En el siguiente ciclo se vuelve a resolver todo el problema con el estado actualizado — esto es lo que se conoce como **horizonte deslizante (receding horizon)**, y es lo que distingue a un MPC real de un simple controlador PID o feedforward.

Si el optimizador no converge (`res.success == False`), el código no aplica un valor no verificado: usa como respaldo la solución anterior (`warm start`), priorizando seguridad sobre agresividad.

### 4B. Estimador de estado (Filtro de Kalman)

`SteeringKalmanFilter` estima `[theta, theta_dot]` a partir de una única medición ruidosa: el ángulo del volante leído del eje HID (solo disponible en Fase B, con hardware real).

Antes, `theta_dot` se calculaba por diferenciación numérica cruda (`(theta - theta_prev) / dt`), lo cual amplifica el ruido de cuantización del eje. El filtro de Kalman en cambio:

1. **Predice** el siguiente estado usando el mismo modelo (`A`, `B`) que ya usa el MPC — así el estimador y el controlador nunca discrepan sobre la dinámica asumida del volante.
2. **Corrige** esa predicción con la medición real de `theta` de ese ciclo.

Los parámetros `q_theta`, `q_theta_dot` (ruido de proceso, qué tanto confiar en el modelo) y `r_theta` (ruido de medición, qué tanto confiar en el sensor) deben calibrarse empíricamente con el hardware real — no existen valores universales, igual que el factor `nm_to_magnitude` de la planta física.

En Fase A no se usa Kalman: la planta simulada entrega el estado exacto por construcción, así que filtrar solo añadiría retraso sin ninguna ganancia.

### 5. Plantas del volante (simulada vs. física)

Dos implementaciones intercambiables de "planta" (el objeto físico que el MPC controla):

- **`SimulatedSteeringPlant`** (Fase A): integra la misma ecuación `J·θ̈ = τ - b·θ̇` que asume el MPC, sin ningún volante real conectado. Es la única fuente de verdad del estado en esta fase — deliberadamente no usa `steerAngle` que reporta el propio Assetto Corsa, para no mezclar la salida del sistema con su propia entrada.
- **`FFBeastSteeringPlant`** (Fase B): controla el volante físico vía SDL2 haptics (ver la nota de hardware más arriba). Expone parámetros críticos de calibración:
  - `max_steering_rad`: la mitad del rango total de rotación configurado en la app de FFBeast.
  - `nm_to_magnitude`: factor de conversión Nm → magnitud SDL2, a calibrar empíricamente.
  - `direction_sign`: corrige el sentido de giro si el FFBeast lo invierte respecto a lo esperado.

`calibrate_ffbeast()` es un modo interactivo (`--calibrate`) que aplica torques de prueba pequeños y crecientes para verificar visualmente que el volante gira en el sentido correcto y de forma proporcional, antes de arriesgarse a correr el MPC completo con el motor conectado.

### 6. Salida hacia vJoy

`VJoyOutput` traduce el ángulo del volante (en radianes) a un valor de eje vJoy (0–32767), que es lo que Assetto Corsa realmente lee como comando de dirección, sin importar si esa posición vino de la planta simulada o del volante físico real.

### 7. Loop principal

`main()` amarra todo el sistema:

1. Carga la trazada de Monza.
2. Conecta a la memoria compartida de Assetto Corsa.
3. Conecta vJoy.
4. Inicializa la planta (simulada o FFBeast según el flag `--ffbeast`).
5. Inicializa el MPC y, si aplica, el filtro de Kalman.
6. En cada ciclo (a un periodo de muestreo `Ts=0.05s`, es decir 20 Hz):
   - Lee posición y velocidad reales del carro.
   - Calcula `e_y`, `e_psi` respecto a la trazada.
   - Obtiene el estado actual del volante (exacto en Fase A, estimado por Kalman en Fase B).
   - Genera la referencia sobre el horizonte y resuelve el MPC.
   - Aplica el torque resultante a la planta y envía la nueva posición a vJoy.
   - Registra todo en el archivo de log CSV.
7. Al terminar (Ctrl+C), centra el volante, cierra todas las conexiones y guarda el log.

Si el carro está prácticamente detenido (`speed_ms <= MIN_SPEED_MS`), el MPC no actúa: se centra el volante y se reinicia el filtro de Kalman, evitando calcular una referencia sin sentido a velocidad casi nula.

## El archivo `monza_fast_lane.csv` (entrada)

Es la trazada de referencia extraída de la "fast lane" de Monza (línea ideal de carrera). Contiene, por cada punto de la pista, su posición (`x`, `z`) y la longitud acumulada (`cumulative_length_m`) desde el inicio de la vuelta. `ReferencePath` usa este archivo para calcular heading y curvatura en toda la pista, y así ubicar al carro y generar la referencia del MPC.

## El archivo `mpc_session_*.csv` (salida) — explicación columna por columna

Cada vez que se corre el script se genera un log nuevo (`mpc_session_<fecha>_<hora>.csv`) con una fila por cada ciclo de control (cada ~0.05s). Estas son sus columnas:

| Columna | Significado |
|---------|-------------|
| `t_s` | Tiempo transcurrido desde que arrancó la sesión, en segundos. |
| `car_x`, `car_z` | Posición absoluta del carro en el mundo de Assetto Corsa (coordenadas del juego). |
| `e_y_m` | Error lateral respecto a la trazada de referencia, en metros. Positivo o negativo según a qué lado de la línea está el carro. |
| `e_psi_rad` | Error de heading respecto a la trazada, en radianes (diferencia entre hacia dónde apunta el carro y hacia dónde apunta la pista). |
| `theta_ref0_deg` | Ángulo de volante que el MPC calculó como referencia para el primer paso del horizonte, en grados. |
| `theta_real_deg` | Ángulo de volante realmente aplicado/alcanzado ese ciclo, en grados. |
| `theta_dot_degs` | Velocidad angular del volante **estimada** (por el Kalman en Fase B), en grados/segundo. Es la que realmente alimenta al MPC. |
| `theta_dot_raw_degs` | Velocidad angular calculada por diferenciación numérica cruda, solo para comparar contra el estimado del Kalman — no se usa para controlar. |
| `error_deg` | Diferencia entre `theta_ref0_deg` y `theta_real_deg`: qué tan bien está siguiendo el volante a la referencia calculada. |
| `tau_Nm` | Torque aplicado por el MPC ese ciclo, en Nm. |
| `speed_kmh` | Velocidad del carro, en km/h. |
| `kappa_avg` | Curvatura promedio de la pista 20 m adelante del carro (positiva/negativa según el sentido de la curva). Útil para correlacionar el comportamiento del MPC con el tipo de tramo (recta vs. curva). |
| `loop_dt_ms` | Duración real de ese ciclo de control, en milisegundos. Debe mantenerse cerca de `Ts × 1000 = 50 ms`; valores mucho mayores indican que el loop se está atrasando respecto al periodo de muestreo asumido por el modelo del MPC, lo cual degrada la calidad del control. |

### Cómo leer una sesión de log

- Si `e_y_m` y `error_deg` se mantienen pequeños y oscilando cerca de cero, el MPC está siguiendo bien la trazada.
- Si `error_deg` crece de forma sostenida (no oscila, sino que se aleja), puede indicar el bug de `steer_sign` invertido descrito en la Sección 3, o que `error_decay`/`Qf` están mal balanceados.
- `loop_dt_ms` sistemáticamente por encima de 50 ms indica que el ciclo de Python no está alcanzando a correr en tiempo real (por ejemplo, el solver del MPC está tardando más de lo esperado), lo que puede requerir bajar `N` (el horizonte) o simplificar el modelo.
- `tau_Nm` saturado constantemente en `±tau_max` sugiere que el controlador no tiene suficiente autoridad de torque para seguir la referencia pedida, o que hay un problema de signo/calibración en la planta física.
