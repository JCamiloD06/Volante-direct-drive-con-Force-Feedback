# MPC para Volante Háptico Direct Drive (Force Feedback)

Control Predictivo Basado en Modelo (MPC) aplicado a un volante de carreras *direct drive* de bajo costo, construido con un motor de hoverboard, un driver **ODrive** y retroalimentación de posición mediante un encoder rotativo acoplado por engranajes.

Este repositorio contiene el **modelado, diseño y validación en MATLAB/Simulink** del controlador, previo a su portado al hardware embebido real.

---

## 1. Resumen del proyecto

El objetivo es que el MPC calcule el **torque de referencia** que el motor debe aplicar al volante para:
- Seguir una trayectoria/referencia angular deseada (θ_ref).
- Generar la sensación de *force feedback* de forma suave y acotada (sin saturaciones bruscas de torque).
- (En la variante 3D) Traducir el giro del volante en el **ángulo de dirección de un vehículo simulado** (modelo bicicleta), cerrando el lazo volante → vehículo → visualización 3D.

El desarrollo está dividido en 4 artefactos:

| Archivo | Rol dentro del proyecto |
|---|---|
| `Modelo.m` | Obtiene el modelo dinámico del volante en espacio de estados (continuo y discreto) |
| `MPC_View_2D.slx` | Simulink: MPC controlando el volante aislado (θ vs θ_ref) |
| `MPC_View_3D.slx` | Simulink: MPC + volante + modelo de vehículo (bicicleta) + escena 3D |
| `Validacion.m` | Post-proceso: métricas de error de seguimiento, trayectoria XY, curvatura y velocidad |
| `MPCDesignerSessionPython.mat` | Sesión guardada de la app **MPC Designer** de MATLAB (contiene el objeto `mpc1` ya diseñado/ajustado) |

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
| Inercia rotacional total | `J` | 0.08 | kg·m² |
| Fricción viscosa total | `b` | 0.05 | N·m·s/rad |

El propio script deja indicado en un comentario que estos valores deben reemplazarse por los identificados experimentalmente una vez se tengan mediciones del volante real.

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

Un integrador puro de posición más un polo de primer orden en velocidad ubicado en `s = -b/J = -0.625 rad/s`.

### 3.4 Función de transferencia (continua y discreta)

`Modelo.m` ahora deriva también la **función de transferencia** del volante, en paralelo al espacio de estados, y muestra todo el desarrollo detalladamente en el *Command Window*.

**Derivación (a partir de la misma ecuación dinámica):**

```
J·θ̈ + b·θ̇ = τ
```

Aplicando Laplace con condiciones iniciales nulas:

```
(J·s² + b·s)·Θ(s) = Τ(s)

G(s) = Θ(s)/Τ(s) = 1 / (J·s² + b·s)
```

En el script esto se construye como `sys_tf = tf(1, [J, b, 0])` y se **verifica por partida doble** contra `tf(sys_c)` (la conversión automática de MATLAB desde el espacio de estados), para confirmar que ambas representaciones coinciden.

El script imprime en el Command Window, tanto para la versión **continua** (`sys_tf`) como para la **discreta** (`sys_tf_d = c2d(sys_tf, Ts, 'zoh')`):
- La derivación simbólica paso a paso (ecuación → Laplace → G(s) con los valores numéricos de `J` y `b` sustituidos).
- El objeto `tf` completo.
- Numerador, denominador, ganancia, ceros y polos (vía `zpkdata`).
- Ganancia DC (`dcgain`).
- Interpretación física de los polos:
  - **Continuo:** polo en `s = 0` (integrador puro del ángulo) y polo en `s = -b/J` (dinámica de velocidad, con constante de tiempo `τ = J/b`).
  - **Discreto:** polo en `z = 1` (arrastrado del integrador continuo) y polo en `z = exp(-b/J·Ts)` (versión discretizada por ZOH del polo de velocidad).

Esto deja el modelo documentado en **ambas representaciones equivalentes** (espacio de estados y función de transferencia), útil tanto para el diseño del MPC (que usa espacio de estados internamente) como para el análisis clásico de la dinámica (polos, constante de tiempo, tipo de sistema) en la sección de metodología del artículo.

### 3.5 Discretización

```matlab
Ts = 0.05;  % 50 ms → 20 Hz
sys_d = c2d(sys_c, Ts, 'zoh');
```

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

## 6. Cómo reproducir

1. Ejecutar `Modelo.m` — genera `sys_c` (continuo) y `sys_d` (discreto) en el workspace, y grafica la respuesta al escalón.
2. Abrir y diseñar/cargar el objeto `mpc1` (por ejemplo, reabriendo la sesión de `MPCDesignerSessionPython.mat` desde la app **MPC Designer**, o creándolo por código con `mpc(sys_d, Ts, Np, Nc)`).
3. Abrir `MPC_View_2D.slx` (validación del lazo volante aislado) o `MPC_View_3D.slx` (validación con vehículo y escena 3D) y simular.
4. Si se usa el modelo 3D, verificar que los bloques `To Workspace` (`X_pos`, `Y_pos`, `theta_real`, `theta_ref`) estén habilitados para logging.
5. Ejecutar `Validacion.m` para obtener las 4 métricas y la figura consolidada `Validacion_MPC_Completa.png`.

---

## 7. Estructura de archivos

```
.
├── Modelo.m                        # Modelo dinámico del volante (espacio de estados)
├── MPC_View_2D.slx                 # Simulink: MPC + volante (sin vehículo)
├── MPC_View_3D.slx                 # Simulink: MPC + volante + vehículo (bicicleta) + escena 3D
├── Validacion.m                    # Métricas de validación post-simulación
├── MPCDesignerSessionPython.mat    # Sesión de MPC Designer (objeto mpc1)
└── Validacion_MPC_Completa.png     # (generado al correr Validacion.m)
```
