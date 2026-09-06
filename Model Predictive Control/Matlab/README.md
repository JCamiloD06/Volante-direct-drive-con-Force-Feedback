# MPC para Volante Háptico Direct Drive (Force Feedback)

Control Predictivo Basado en Modelo (MPC) aplicado a un volante de carreras *direct drive* de bajo costo, construido con un motor de hoverboard, un driver **ODrive** y retroalimentación de posición mediante un encoder rotativo acoplado por engranajes.

Este repositorio contiene el **modelado, diseño y validación en MATLAB/Simulink** del controlador, previo a su portado al hardware embebido real.

---

## 1. Resumen del proyecto

El objetivo es que el MPC calcule el **torque de referencia** que el motor debe aplicar al volante para:
- Seguir una trayectoria/referencia angular deseada (θ_ref).
- Generar la sensación de *force feedback* de forma suave y acotada (sin saturaciones bruscas de torque).
- (En la variante 3D) Traducir el giro del volante en el **ángulo de dirección de un vehículo simulado** (modelo bicicleta), cerrando el lazo volante → vehículo → visualización 3D.

El desarrollo está dividido en 5 artefactos:

| Archivo | Rol dentro del proyecto |
|---|---|
| `Modelo.m` | Obtiene el modelo dinámico del volante en espacio de estados (continuo y discreto). **Dependencia funcional:** genera `sys_c`/`sys_d` en el workspace, que `MPC_View_2D.slx` y `MPC_View_3D.slx` necesitan para simular — no es solo documentación |
| `MPC_View_2D.slx` | Simulink: MPC controlando el volante aislado (θ vs θ_ref) |
| `MPC_View_3D.slx` | Simulink: MPC + volante + modelo de vehículo (bicicleta) + escena 3D |
| `Validacion.m` | Post-proceso: métricas de error de seguimiento, trayectoria XY, curvatura y velocidad |
| `MPCDesignerSessionPython.mat` | Sesión guardada de la app **MPC Designer** de MATLAB (contiene el objeto `mpc1` ya diseñado/ajustado) |
| `mpc_ambos_modelos.m` | Script standalone con la **derivación matemática completa de los dos MPC del proyecto** (dirección y longitudinal), pensado como apoyo para la sección de metodología del reporte — ver sección 7. **No alimenta a Simulink:** deriva su propio modelo internamente (por Euler) solo con fines de reporte; aunque la matemática de dirección se solapa con `Modelo.m`, cumplen roles distintos y ninguno reemplaza al otro |

> **Nota:** el objeto MPC (`mpc1`) referenciado por los bloques `Control MPC` de ambos modelos se genera/ajusta con la app *MPC Designer* y queda almacenado en `MPCDesignerSessionPython.mat`.

---

## 2. Hardware objetivo (contexto)

- **Actuador:** motor de hoverboard (BLDC, control por torque).
- **Driver:** ODrive (par calculado por el MPC → corriente de referencia).
- **Sensor:** encoder rotativo, acoplado al eje del volante mediante engranajes.
- **Destino final del controlador:** ejecución embebida en tiempo real (mencionado en comentarios del código como STM32).

---

## 3. Modelo dinámico — `Modelo.m`

### 3.1 Hipótesis de modelado

El volante se modela como un **sistema rotacional de 2° orden**: una inercia sometida a un torque de entrada y una fricción viscosa.

**Estado:**

```
x = [θ; θ̇]        (ángulo, velocidad angular)
```

**Entrada:** `u = τ` (torque del motor, en N·m)
**Salida:** `y = θ` (ángulo medido por el encoder, en rad)

### 3.2 Parámetros físicos

| Parámetro | Símbolo | Valor actual | Unidades |
|---|---|---|---|
| Inercia rotacional total | `J` | 0.02 | kg·m² |
| Fricción viscosa total | `b` | 0.1 | N·m·s/rad |

El propio script deja indicado en un comentario que estos valores son **estimados** y deben reemplazarse por los identificados experimentalmente una vez se tengan mediciones del volante real.

### 3.3 Espacio de estados continuo

```
A = [ 0      1   ]        B = [  0  ]
    [ 0   -b/J   ]            [ 1/J ]

C = [1  0]        D = 0
```

Es decir:

```
θ̈ = -(b/J)·θ̇ + (1/J)·τ
```

Un integrador puro de posición más un polo de primer orden en velocidad ubicado en `s = -b/J = -5 rad/s` (constante de tiempo asociada `τ = J/b = 0.2 s`).

El objeto `sys_c` se construye con nombres de estado/entrada/salida explícitos (`'Angulo (rad)'`, `'Velocidad (rad/s)'`, `'Torque (Nm)'`), lo que facilita la lectura de los bloques de espacio de estados directamente en el Command Window de MATLAB.

### 3.4 Función de transferencia (continua y discreta)

`Modelo.m` deriva también la **función de transferencia** del volante, en paralelo al espacio de estados, y muestra todo el desarrollo detalladamente en el *Command Window*.

**Derivación (a partir de la misma ecuación dinámica):**

```
J·θ̈ + b·θ̇ = τ
```

Aplicando Laplace con condiciones iniciales nulas:

```
(J·s² + b·s)·Θ(s) = Τ(s)

G(s) = Θ(s)/Τ(s) = 1 / (J·s² + b·s)
```

Con los valores actuales (`J = 0.02`, `b = 0.1`):

```
G(s) = 1 / (0.02·s² + 0.1·s)
```

En el script esto se construye como `sys_tf = tf(1, [J, b, 0])` y se **verifica por partida doble** contra `tf(sys_c)` (la conversión automática de MATLAB desde el espacio de estados), para confirmar que ambas representaciones coinciden.

El script imprime en el Command Window, tanto para la versión **continua** (`sys_tf`) como para la **discreta** (`sys_tf_d = c2d(sys_tf, Ts, 'zoh')`):
- La derivación simbólica paso a paso (ecuación → Laplace → G(s) con los valores numéricos de `J` y `b` sustituidos).
- El objeto `tf` completo.
- Numerador, denominador, ganancia, ceros y polos (vía `zpkdata`).
- Ganancia DC (`dcgain`).
- Interpretación física de los polos:
  - **Continuo:** polo en `s = 0` (integrador puro del ángulo) y polo en `s = -b/J = -5 rad/s` (dinámica de velocidad, con constante de tiempo `τ = J/b = 0.2 s`).
  - **Discreto:** polo en `z = 1` (arrastrado del integrador continuo) y polo en `z = exp(-b/J·Ts) ≈ 0.7788` (versión discretizada por ZOH del polo de velocidad, con `Ts = 0.05 s`).

Esto deja el modelo documentado en **ambas representaciones equivalentes** (espacio de estados y función de transferencia), útil tanto para el diseño del MPC (que usa espacio de estados internamente) como para el análisis clásico de la dinámica (polos, constante de tiempo, tipo de sistema) en la sección de metodología del artículo.

### 3.5 Discretización

```matlab
Ts = 0.05;  % 50 ms → 20 Hz
sys_d = c2d(sys_c, Ts, 'zoh');
```

> **Importante:** este `Ts = 0.05 s` es el paso de discretización usado en `Modelo.m` para análisis/diseño offline. Es distinto del `n_ts = 0.01 s` (100 Hz) configurado en el bloque `Control MPC` dentro de Simulink (ver sección 4.3) — no deben confundirse en el reporte.

### 3.6 Verificación

El script grafica la respuesta al escalón de 1 N·m del sistema continuo (`step(sys_c, 5)`) como chequeo visual de que el modelo tiene sentido físico (arranque desde reposo, aceleración angular constante limitada por la fricción).

---

## 4. Arquitectura de control MPC

Ambos modelos de Simulink comparten la misma columna vertebral:

```
Referencia (grados) → [pi/180] → MPC Controller → Volante (State-Space) → [180/pi] → Scopes / To Workspace
                                        ↑
                              realimentación (θ medido, con ruido de encoder)
```

### 4.1 `MPC_View_2D.slx` — MPC aislado sobre el volante

Bloques principales:

| Bloque | Tipo | Función |
|---|---|---|
| `Control MPC` | `MPC Controller` (mpclib) | Calcula el torque óptimo |
| `Volante` | `State-Space` | Planta (usa `sys_c.A/B/C/D` del workspace, generado por `Modelo.m`) |
| `Ref_Trayectoria1` | `Sin Wave` | Referencia senoidal: amplitud 180°, frecuencia 0.3 Hz |
| `Repeating Sequence2` | `Repeating Table` (**deshabilitado/comentado**) | Referencia alterna tipo "escalera": `t=[0,2,4,6,8,10,12,14]`, `y=[0,0,90,90,0,-90,-90,0]` grados |
| `Ruido Encoder` | `Band-Limited White Noise` | Ruido de medición inyectado a la realimentación (`Cov = 1e-6`, `Ts = 0.01`) |
| `Add` | `Sum` | Suma el ruido a la señal realimentada antes de entrar al MPC |
| `Trayectoria` | `Scope` | Referencia vs. ángulo real del volante (grados) |
| `Torque` | `Scope` | Torque de salida del MPC |

Conversión de unidades: la referencia entra en **grados** y se convierte a **radianes** (`pi/180`) antes del MPC; la salida del volante se reconvierte a **grados** (`180/pi`) para visualización.

### 4.2 `MPC_View_3D.slx` — MPC + vehículo + escena 3D

Extiende el modelo anterior agregando la cadena volante → dirección del vehículo:

| Bloque | Tipo | Función |
|---|---|---|
| `Control MPC`, `Volante`, `Ruido Encoder`, `Add` | (idénticos al 2D) | Mismo lazo de control del volante |
| `Ref-Trayectoria2` | `Repeating Table` | Referencia activa: `t=[0,3,5,7,9,25]`, `y=[0,0,30,30,0,0]` grados (giro de volante tipo maniobra de carril) |
| `Ref_Trayectoria1` | `Sin Wave` (**comentado/inactivo**) | Alternativa senoidal, amplitud 20°, 0.3 Hz |
| `Relación de transmisión` | `Gain = 1/8` | Convierte el ángulo del **volante** en ángulo de **dirección de las ruedas** (relación de dirección 8:1) |
| `Velocidad en m/s` | `Constant = 11` | Velocidad longitudinal fija que alimenta el modelo de vehículo |
| `Bicycle Model – Velocity Input` | Driving Scenario Toolbox (`autolibshared`) | Modelo lateral de vehículo 3-DOF (bicicleta), masa 2000 kg, `Izz = 4000`, distancia a ejes `a=1.4 m`, `b=1.6 m`, rigidez de neumáticos `Cy_f=12e3`, `Cy_r=11e3`, entre otros parámetros de dinámica lateral |
| `Integrator`, `Integrator1` | `Integrator` | Integran velocidad lateral/longitudinal del vehículo para obtener posición `X`, `Y` |
| `Simulation 3D Scene Configuration` | Sim 3D | Visualización en el motor Unreal (escena *"Double lane change"*) |
| `To Workspace` (`X_pos`), `To Workspace1` (`Y_pos`) | `ToWorkspace` | Guardan la trayectoria del vehículo |
| `To Workspace2` (`theta_real`), `To Workspace3` (`theta_ref`) | `ToWorkspace` | Guardan ángulo real y de referencia del volante (grados) — **usados por `Validacion.m`** |
| `Giro vehiculo`, `Referencia vs Volante`, `Torque` | `Scope` | Visualización en vivo |

### 4.3 Configuración del bloque MPC Controller (idéntica en ambos modelos)

| Parámetro | Valor | Significado |
|---|---|---|
| `mpcobj` | `mpc1` | Nombre del objeto MPC en el workspace (creado con MPC Designer, ver `MPCDesignerSessionPython.mat`) |
| `n_ts` (Ts del MPC) | 0.01 s | Frecuencia de control: **100 Hz** |
| `n_p` (horizonte de predicción, *Np*) | 150 pasos | Equivale a 1.5 s de horizonte hacia adelante |
| `n_mv` (variables manipuladas) | 1 | Torque del motor |
| `n_mo` (salidas medidas) | 1 | Ángulo del volante (θ) |
| `n_md`, `n_ud`, `n_uo` | 0 | Sin perturbaciones medibles ni salidas no medidas configuradas |
| `HorizonChoice` | `FixedHorizon` | Horizonte fijo (no variable) |
| Entradas opcionales (`umin/umax/ymin/ymax/uwt/ywt`, etc.) | todas `off` | **No hay restricciones ni pesos configurados directamente en el bloque** — todo eso vive dentro del objeto `mpc1`, definido/ajustado desde la app MPC Designer |

Los pesos de la función de costo y las restricciones de torque/velocidad del objeto `mpc1` se definen y ajustan desde la app MPC Designer, y quedan almacenados en `MPCDesignerSessionPython.mat`.

---

## 5. Validación — `Validacion.m`

Script de post-procesamiento que se corre **después** de simular cualquiera de los dos modelos en Simulink (usa variables `out.*` capturadas por los bloques `To Workspace`/logging).

### 5.1 Métrica 1 — Error de seguimiento angular del volante
Compara `theta_ref` vs. `theta_real` (en grados):
- **RMSE**, **MAE**, **error máximo absoluto**.
- Criterio orientativo definido en el propio script: RMSE < 5° = excelente, < 10° = aceptable, > 15° = requiere reajuste.

### 5.2 Métrica 2 — Trayectoria XY del vehículo (solo aplica al modelo 3D)
A partir de `X_pos`, `Y_pos`:
- Distancia total recorrida (integral discreta de la norma del desplazamiento).
- Desviación lateral máxima/mínima y rango lateral total (`Y`).
- Duración total de la simulación.

### 5.3 Métrica 3 — Suavidad de la trayectoria (curvatura)
Curvatura calculada numéricamente con derivadas de `X`, `Y` (`gradient`):

```
κ = |x'·y'' - y'·x''| / (x'² + y'²)^(3/2)
```

Se reporta curvatura media y máxima. Picos de curvatura indican cambios bruscos de dirección (indeseables para el confort/realismo del force feedback).

### 5.4 Métrica 4 — Velocidad resultante del vehículo
Derivada numérica de `X`, `Y` respecto a `Ts` para obtener `Vx`, `Vy`, y velocidad total. Se compara contra la velocidad constante esperada de la simulación.

### 5.5 Salidas gráficas
Una figura con 5 paneles: trayectoria XY, desviación lateral vs. tiempo, posición X vs. tiempo (contra referencia teórica `v·t`), curvatura vs. tiempo, y velocidad resultante vs. tiempo. Se guarda como `Validacion_MPC_Completa.png`.

---

## 6. Modelo dinámico del control longitudinal (acelerador/freno)

### 6.1 Hipótesis de modelado

A diferencia del volante, que es un actuador físico propio con dinámica conocida derivable desde primeros principios (sección 3), el control longitudinal actúa sobre la física interna de Assetto Corsa (motor, transmisión, neumáticos) — una "caja negra" que no puede modelarse así. Por eso la respuesta de velocidad del auto ante cada pedal se **identifica empíricamente**, no se deriva.

Además, a diferencia de dirección (un solo modelo con signo de entrada), aquí **throttle y freno usan modelos distintos**, porque su física es distinta.

**Estado:** `v` (velocidad del auto, en km/h)
**Entrada:** `u ∈ [-1, 1]` (positivo = acelerador, negativo = freno)

### 6.2 Modelo de throttle — primer orden (K/tau)

Igual que la respuesta de un motor DC o un circuito RC, la velocidad ante el acelerador se trata como un sistema de un polo:

```
tau_th · dv/dt + v = K_th · u        (u ∈ [0,1])
```

- `tau_th = 2.81 s` — identificado con `step_test_logger.py` + `identify_model.py`, consistente entre las 4 amplitudes de prueba (desv. std 0.062 s). Es el parámetro **confiable** de este modelo.
- `K_th` — NO es confiable tal cual (varía 46% entre amplitudes): la meseta de velocidad medida en las pruebas está limitada por el corte de RPM de la marcha usada, no por una propiedad general del auto. Se deja como valor *placeholder*, pendiente de sustituir por uno basado en la velocidad máxima real del auto.

### 6.3 Modelo de freno — desaceleración constante (no exponencial)

A diferencia del throttle, el freno **no** se modela como un sistema de un polo. La razón es física, no de conveniencia:

- El acelerador satura de forma **suave**, porque el arrastre aerodinámico que se le opone crece con `v²`.
- El freno aplica una fricción neumático-pista aproximadamente **constante** en el rango de operación normal — no depende fuertemente de `v`.

Por la segunda ley de Newton:

```
m · dv/dt = -F_freno   →   dv/dt = -a_brake_max   (constante)
```

Es un modelo de **rampa** (integrador puro con entrada constante), no de decaimiento exponencial. Esto se confirmó con datos reales: un ajuste lineal a la velocidad de frenado dio R²≈0.9989, contra R²≈0.83–0.94 forzando una exponencial. El valor identificado es `a_brake_max = 10.25 m/s²`, prácticamente constante entre las 4 amplitudes de prueba (10.09 a 10.46 m/s²) — con solo 30% de freno ya se está cerca del límite de agarre de las llantas (~1.05g), así que presionar más fuerte casi no frena más rápido.

### 6.4 Discretización

**Throttle** (Euler, paso de predicción interno `Ts_long = 0.2 s`, distinto del paso de actuación — ver 7.4):

```
v[k+1] = Ad_th·v[k] + Bd_th·u[k]      con   Ad_th = 1 - Ts_long/tau_th,   Bd_th = (Ts_long/tau_th)·K_th
```

**Freno** (ya es lineal en el tiempo, se discretiza directamente sin pasar por una ODE):

```
v[k+1] = max(0, v[k] - Ts·a_brake_max·|u|)
```

El `max(0, ...)` es obligatorio: el auto se detiene y se queda detenido, no puede ir a velocidades negativas — a diferencia del throttle, que no tiene ese límite incorporado matemáticamente.

### 6.5 Ecuación conmutada (implementación real)

En `MPCLongitudinalController`, el modelo aplicado en cada ciclo depende del signo de `u`:

```python
if u >= 0:
    v += (ts/tau_th) * (-v + K_th*u)      # throttle: primer orden
else:
    v -= ts * a_brake_max * (-u)          # freno: desaceleración constante
    v = max(0.0, v)
```

`a_brake_max` se escala proporcionalmente a `|u|` (a `u=1.0` se usa el valor medido, a `u` parcial se reduce linealmente) en vez de asumir saturación completa desde cualquier pedal — es la opción conservadora: subestima un poco el frenado a pedal parcial en vez de sobreestimarlo.

### 6.6 Valores finales y validación

```python
model_params = {
    'throttle': {'K': 122.8, 'tau': 2.81},   # K es placeholder, ver 6.2
    'brake_a_max_ms2': 10.25,
}
```

Pendiente: reemplazar `K=122.8` por un valor basado en la velocidad máxima real del auto en Monza; `tau=2.81` y `brake_a_max_ms2=10.25` ya están listos para usarse tal cual.

`predict_velocity()` se validó de forma aislada antes de integrarse: throttle a fondo desde `v=0` sube en curva suave (exponencial); freno a fondo desde `v=100 km/h` baja en **línea recta** y hace clip exacto en 0 — coincide con el comportamiento real observado en los datos de calibración.

---

## 7. Modelo matemático unificado — `mpc_ambos_modelos.m`

Script standalone (corre completo con F5, sin archivos externos) que consolida la **derivación matemática de los dos controladores MPC del proyecto** — dirección (volante) y longitudinal (acelerador/freno) — pensado como material de apoyo para la sección de metodología del reporte. Requiere el *Optimization Toolbox* (`fmincon`) para correr las simulaciones de lazo cerrado; si no está disponible, el script avisa y de todas formas muestra las derivaciones y gráficas de referencia.

> **Nota sobre la superposición con `Modelo.m`:** la Parte 1 de este script vuelve a derivar el modelo físico del volante (mismos parámetros `J=0.02`, `b=0.1` que la sección 3), pero con un propósito distinto: aquí es solo el punto de partida para la formulación batch (Parte 2), no una dependencia que otro archivo necesite importar. `Modelo.m` sigue siendo el que hay que ejecutar para poder simular en Simulink (ver sección 7); este script es autocontenido y no sustituye a ese paso.

### 7.1 Partes 1-2 — MPC de dirección (formulación batch, QP lineal genuina)

A diferencia de `Modelo.m` (que discretiza con `c2d`/ZOH), aquí la discretización es por **Euler hacia adelante** sobre el mismo modelo continuo (`J=0.02`, `b=0.1`, `Ts_dir=0.05s`):

```
Ad = I + Ts·Ac        Bd = Ts·Bc
```

La predicción sobre el horizonte se condensa en forma matricial (`Theta = Phi·x0 + Gamma·U`), con `Phi` y `Gamma` construidas a partir de potencias de `Ad`. La función de costo pondera el error de seguimiento (`Q=10.41`, `Qf=10.41` para el último paso del horizonte), el esfuerzo de torque (`R=0`) y su tasa de cambio (`Rd=0.288`), sujeta a `tau_max=2.0 Nm`, `rate_max=1.0 Nm/paso` y `theta_max=1.2 rad`. Al ser un modelo lineal sin conmutación, este es un problema de control cuadrático puro, resuelto con `fmincon` (SQP) sobre un horizonte `N=10` (0.5 s), y validado con una referencia tipo "chicana" (seno truncado).

### 7.2 Parte 3 — Modelo de throttle (primer orden, sin cambios)

Mismo modelo K/tau identificado empíricamente en la calibración longitudinal: `tau_th·dv/dt + v = K_th·u`, con `tau_th=2.81 s` (confiable) y `K_th` como placeholder (incierto por saturación de RPM en la marcha de prueba — ver el README de calibración de acelerador/freno). Se discretiza por Euler con un paso de predicción interno `Ts_long=0.2 s` (distinto del paso de actuación, ver 6.4).

### 7.3 Parte 4 — Modelo de freno: el cambio importante

El freno **deja de modelarse como K/tau exponencial** y pasa a modelarse como **desaceleración constante**, con justificación física explícita en el propio script:

- El acelerador satura de forma *suave* porque el arrastre aerodinámico crece con `v²`.
- El freno aplica una fricción neumático-pista aproximadamente *constante* en el rango de operación normal, por lo que `dv/dt = -a_brake_max` (constante), no un sistema de un polo.

Esto se traduce en un modelo de rampa con clip en cero:

```
v[k+1] = max(0, v[k] - Ts·a_brake_max·|u|)
```

con `a_brake_max = 10.25 m/s²` (identificado empíricamente). El `max(0, ...)` es obligatorio: el auto se detiene y se queda detenido, a diferencia del throttle, que no tiene ese límite matemático incorporado. Este cambio es consistente con el hallazgo de calibración de que un ajuste lineal a los datos de frenado da R²≈0.999, frente a R²≈0.83–0.94 forzando una exponencial.

### 7.4 Parte 5 — Horizonte de predicción no uniforme (multi-rate)

Dirección y longitudinal tienen constantes de tiempo muy distintas: `tau_direccion = J/b = 0.2 s` frente a `tau_throttle ≈ 2.81 s` (~14 veces más lenta). Si el MPC longitudinal predijera con el mismo `Ts=0.05 s` de dirección, un horizonte de 15 pasos solo cubriría 0.75 s — menos de un tercio de `tau_throttle`, dejando al controlador "miope" respecto a su propia dinámica. La solución: el longitudinal sigue **actuando** cada `Ts_dir=0.05 s` (misma frecuencia de reacción que dirección), pero **predice** internamente con pasos de `Ts_long=0.2 s`, de modo que `N=15` pasos cubren 3.0 s de horizonte — sí alcanza a cubrir `tau_throttle`. Es la técnica estándar de rejilla de predicción no uniforme ("move blocking") para sistemas con dinámicas de múltiple escala temporal.

### 7.5 Parte 6 — Resolución del sistema conmutado longitudinal

A diferencia de dirección (QP pura), el longitudinal es un **sistema conmutado**: qué ecuación aplica (throttle o freno) depende del signo de cada `u_i` de la secuencia, así que `Gamma` ya no es una matriz fija — depende de la propia solución buscada. Por eso se resuelve con `fmincon` (no `quadprog`), con pesos `Q=10.0`, `R=0.5`, `Rrate=2.0` sobre un horizonte `N=15`, `u∈[-1,1]`. Se valida con un perfil de referencia de velocidad tipo escalón (100 → 0 → 120 km/h), donde se observa explícitamente la frenada en **línea recta** hacia cero (consistente con el modelo de la Parte 4), y el comando de control conmutando de signo entre acelerador y freno.

### 7.6 Parte 7 — Perfil de velocidad por curvatura (contexto)

Incluida solo como contexto de dónde sale la referencia de velocidad (`v_ref`) que alimenta al MPC longitudinal en el sistema real: `v_max(s) = sqrt(ay_max/|kappa(s)|)`, seguido de un backward-pass que limita la velocidad según cuánto se puede frenar (Torricelli) desde el punto siguiente. No es un tercer MPC. El propio script remite a `mpc_longitudinal_matematica.m` para el detalle completo de esta derivación.

### 7.7 Funciones auxiliares del script

| Función | Rol |
|---|---|
| `costo_direccion` | Función de costo del MPC de dirección (seguimiento + esfuerzo + tasa de cambio) |
| `restricciones_direccion` | Restricciones no lineales de dirección (`theta_max`, `rate_max`) para `fmincon` |
| `costo_longitudinal` | Función de costo del MPC longitudinal, simulando internamente la conmutación throttle/freno paso a paso dentro del horizonte |

---

## 8. Cómo reproducir

1. Ejecutar `Modelo.m` — genera `sys_c` (continuo) y `sys_d` (discreto) en el workspace, y grafica la respuesta al escalón.
2. Abrir y diseñar/cargar el objeto `mpc1` (por ejemplo, reabriendo la sesión de `MPCDesignerSessionPython.mat` desde la app **MPC Designer**, o creándolo por código con `mpc(sys_d, Ts, Np, Nc)`).
3. Abrir `MPC_View_2D.slx` (validación del lazo volante aislado) o `MPC_View_3D.slx` (validación con vehículo y escena 3D) y simular.
4. Si se usa el modelo 3D, verificar que los bloques `To Workspace` (`X_pos`, `Y_pos`, `theta_real`, `theta_ref`) estén habilitados para logging.
5. Ejecutar `Validacion.m` para obtener las 4 métricas y la figura consolidada `Validacion_MPC_Completa.png`.
6. Para la derivación matemática y las simulaciones de referencia de ambos MPC (sin necesidad de Simulink ni de Assetto Corsa), correr `mpc_ambos_modelos.m` directamente con F5.

---

## 9. Estructura de archivos

```
.
├── Modelo.m                        # Modelo dinámico del volante (espacio de estados)
├── MPC_View_2D.slx                 # Simulink: MPC + volante (sin vehículo)
├── MPC_View_3D.slx                 # Simulink: MPC + volante + vehículo (bicicleta) + escena 3D
├── Validacion.m                    # Métricas de validación post-simulación
├── MPCDesignerSessionPython.mat    # Sesión de MPC Designer (objeto mpc1)
├── mpc_ambos_modelos.m             # Derivación matemática de ambos MPC (dirección + longitudinal)
└── Validacion_MPC_Completa.png     # (generado al correr Validacion.m)
```