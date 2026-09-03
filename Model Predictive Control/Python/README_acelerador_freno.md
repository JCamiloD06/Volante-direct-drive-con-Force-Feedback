# Calibración del control longitudinal (acelerador/freno) — MPC Monza

Este documento cubre todo lo relacionado con `MPCLongitudinalController`
dentro de `mpc_monza_UNIFICADO.py`: qué modelo usa, cómo se calibró, qué
salió mal en el camino, y qué se corrigió. El resto del sistema (dirección,
Kalman, perfil de velocidad, salida a vJoy) no se tocó en este proceso —
solo el control de gas/freno.

---

## 1. De dónde partimos

El control longitudinal empezó con un modelo de primer orden identificado
empíricamente para **ambos** pedales:

```
tau * dv/dt + v = K * u        (u en [-1,1]: + gas, - freno)
```

La idea: en vez de modelar el motor/transmisión/neumáticos de AC desde
primeros principios (imposible, es una caja negra), se trata la respuesta
de velocidad ante un comando de pedal como un sistema de "un polo",
igual que se identificaría la respuesta de un motor DC o un circuito RC,
y se ajustan `K` (ganancia estática) y `tau` (constante de tiempo) a datos
reales de step-response.

## 2. Herramientas construidas para calibrar

| Script | Qué hace |
|---|---|
| `step_test_logger.py` | Aplica un escalón de gas o freno a una amplitud fija, graba velocidad/marcha/RPM a 100Hz, y verifica que la marcha no haya cambiado durante la prueba |
| `identify_model.py` | Ajusta los parámetros del modelo a los CSV que produce el logger, con gráficas de validación |

Protocolo: 8 pruebas de 6-8 segundos cada una — throttle y brake, cada uno
en amplitudes 0.3, 0.5, 0.7 y 1.0 — con la caja de cambios en modo Manual
y la misma marcha fija durante cada prueba individual.

## 3. Primera ronda — resultado malo, pero informativo

Los primeros 8 CSV dieron resultados claramente rotos:

- **Throttle**: las 4 curvas (u=0.3 a 1.0) salían casi idénticas entre sí,
  con un patrón de "escalones" (mesetas en ~9, ~18, ~28 km/h).
- **Brake**: la velocidad SUBÍA 12-14 km/h en el primer ~1.2s antes de
  empezar a bajar, en 3 de 4 pruebas.
- En ambos casos, `tau` se pegaba contra el límite superior del ajuste
  (20s) — señal de que el optimizador no tenía suficiente información
  para converger a un valor real.

### Diagnóstico (con los datos, no solo con la gráfica)

```
Throttle: las 4 pruebas arrancaron desde v=0 en gear=5, RPM~900 (ralentí)
Brake:    las 4 pruebas mantuvieron gear=4 constante (eso sí estaba bien)
          3 de 4 mostraron un salto de +11.8 a +14.3 km/h antes de frenar
```

- **Throttle arrancó en una marcha demasiado alta** para salir de parado
  (en la convención estándar de memoria compartida de AC, `gear=5`
  correspondería a 4ª marcha — no confirmado al 100% para tu versión
  exacta, pero consistente con el síntoma). Arrancar en una marcha alta
  hace que el auto dependa del patinaje del embrague, no del acelerador,
  para moverse — por eso las 4 amplitudes se veían casi iguales.
- **Brake tenía residuo de aceleración** del control físico que no se
  soltó a tiempo antes de que el escalón de freno se aplicara.

### Correcciones aplicadas al código (antes de repetir pruebas)

1. `step_test_logger.py`: ahora verifica `ph.gas < 0.05` sostenido por
   0.3s antes de dejar arrancar un escalón de freno — ya no depende de
   que el humano se acuerde de soltar el pedal a tiempo.
2. Se avisa en consola la marcha detectada al arrancar desde parado en
   una prueba de throttle, para que el usuario la confirme visualmente
   contra el HUD de AC antes de correr la prueba.

## 4. Segunda ronda — throttle mejoró, pero reveló un problema distinto

Con la marcha correcta (gear=2), las curvas de throttle salieron limpias,
sin escalones — pero las 4 amplitudes convergían a un techo **idéntico**
(53.7 km/h, con una desviación estándar de apenas 0.01-0.03 km/h entre
ellas). Eso no es un problema de la prueba: es el auto topando contra el
**limitador de RPM** de esa marcha, algo físicamente real, no un artefacto.

### Por qué esto rompe el ajuste K/tau

El modelo `tau*dv/dt+v=K*u` asume una saturación **suave** (como el
arrastre aerodinámico, que crece gradualmente con v²). Un corte de RPM es
una saturación **dura** — la velocidad sube y se detiene en seco. Al
forzar una exponencial a explicar un corte duro, el ajuste "sabe" que
tiene que aplanarse en algún punto, pero el valor de `K*u` que calcula
como asíntota (~63 km/h) queda por ENCIMA de la meseta real (53.7 km/h)
porque la exponencial nunca llega a aplanarse del todo dentro de los 7
segundos de la prueba.

### El intento fallido de "arreglarlo" (documentado a propósito)

La primera idea fue: igual que se excluye el tramo plano en 0 del ajuste
de freno, excluir la meseta dura del ajuste de throttle. **Se probó y
empeoró el resultado**: sin ver dónde se aplana la curva, el optimizador
pierde la única información que tenía para fijar `tau`, y vuelve a
pegarse contra el límite de 20s. La versión que SÍ funciona es la
contraria: dejar que el ajuste use toda la curva, meseta incluida, aunque
el modelo no la represente perfectamente — eso le da al optimizador la
referencia que necesita para converger a un `tau` estable y repetible.

**Lección:** no toda inconsistencia entre el modelo y los datos se
arregla recortando datos. A veces los datos "imperfectos" son los que
mejor restringen el parámetro que sí importa.

### Solución final para throttle

`identify_model.py` ahora detecta la meseta dura (`find_plateau_start()`)
y la reporta como diagnóstico informativo — sin excluirla del ajuste. El
resultado con gear=2:

```
tau: media=2.814s   desv.std=0.062s   (muy consistente entre amplitudes)
K:   media=122.76   desv.std=56.18    (NO consistente — ver sección 5)
R² promedio: 0.9540
```

## 5. Por qué `K` de throttle no se usa tal cual

`K*u` salió casi constante entre las 4 amplitudes (coef. variación 1%)
mientras que `K` varía 46% — la firma matemática de "la velocidad final
no depende de cuánto aceleras", exactamente lo esperable cuando el techo
real es el limitador de RPM de una marcha específica, no el pedal.

Usar el promedio empírico de K (122.8) tal cual haría que el MPC crea que
a fondo el auto se asienta en ~123 km/h en estado estacionario — falso,
ese número es un artefacto de la marcha usada en la prueba, no una
propiedad general del auto.

**Lo que SÍ es confiable de esta prueba:** `tau=2.81s` — qué tan rápido
responde el motor a un cambio de pedal, que es información genuinamente
útil y estable independiente de en qué marcha se midió.

**Recomendación:** sustituir `K` por un valor basado en la velocidad
máxima real del auto en Monza (con todas las marchas disponibles, no solo
la marcha de la prueba), dejando `tau=2.81` sin tocar. Es una decisión de
ingeniería, no algo que el script pueda calcular por sí solo — y es
exactamente el tipo de ajuste ya anticipado como limitación conocida del
modelo K/tau (válido "cerca de donde se calibró", no en todo el rango).

## 6. Cambio de estructura en el modelo de freno

A diferencia de throttle, el freno no solo necesitaba mejores datos —
necesitaba un **modelo distinto**.

### Qué mostraron los datos limpios

Con el tramo contaminado (el salto inicial) excluido automáticamente por
`fit_brake()`, el frenado real resultó ser casi una **línea recta**, no
una exponencial:

```
a_brake: media=10.25 m/s²   desv.std=0.16 m/s²   (CV=1.5%)
R² promedio: 0.9989   (contra 0.83-0.94 forzando una exponencial)
```

Dos hallazgos:

1. **La física es de desaceleración constante**, no de decaimiento
   exponencial — el freno aplica fricción casi constante, muy distinto
   del arrastre aerodinámico que sí decae con la velocidad. Un ajuste
   lineal es más fiel a la física real, no solo un mejor número de R².
2. **La desaceleración es casi idéntica en las 4 amplitudes** (10.09 a
   10.46 m/s², prácticamente sin variar) — con solo 30% de freno ya se
   está cerca del límite de agarre de las llantas (~1.05g). Presionar más
   fuerte casi no frena más rápido en este setup.

### Por qué este modelo es más consistente con el resto del sistema

`SpeedProfileGenerator` (la capa de planeación de velocidad por
curvatura) YA asumía una desaceleración de frenado constante
(`ax_brake_max`) para calcular el backward-pass del perfil de velocidad.
Con este cambio, el MPC longitudinal y el generador de perfil de
velocidad usan la MISMA física de frenado — antes eran inconsistentes
entre sí sin que se notara.

### El cambio de código

`MPCLongitudinalController` pasó de:

```python
# ANTES: exponencial para ambos pedales
if u >= 0:
    K, tau = K_th, tau_th
else:
    K, tau = K_br, tau_br
v += (ts/tau) * (-v + K*u)
```

a:

```python
# AHORA: modelo distinto por pedal
if u >= 0:
    v += (ts/tau_th) * (-v + K_th*u)      # throttle: primer orden
else:
    v -= ts * a_brake_max * (-u)          # brake: desaceleración constante
    v = max(0.0, v)                        # no puede ir a v negativa
```

`a_brake_max` se escala proporcionalmente a `|u|` (a u=1.0 se usa el
valor medido, a u parcial se reduce linealmente) en vez de asumir
saturación completa desde cualquier pedal — es la opción conservadora:
subestima un poco el frenado a pedal parcial en vez de sobreestimarlo,
que es el error seguro de tener si hay que equivocarse en algún sentido.

## 7. Valores finales en `mpc_monza_UNIFICADO.py`

```python
model_params = {
    'throttle': {'K': 122.8, 'tau': 2.81},   # K es placeholder, ver sección 5
    'brake_a_max_ms2': 10.25,
}
```

**Pendiente de tu parte:** reemplazar `K=122.8` por un valor basado en la
velocidad máxima real de tu auto en Monza antes de confiar en el
comportamiento de aceleración a velocidades altas — `tau=2.81` y
`brake_a_max_ms2=10.25` ya están listos para usarse tal cual.

## 8. Validación funcional del código nuevo

Se probó `MPCLongitudinalController.predict_velocity()` de forma aislada
antes de integrarlo:

- Throttle a fondo desde v=0: sube en curva suave (exponencial), como se
  espera.
- Freno a fondo desde v=100 km/h: baja en **línea recta** y hace clip
  exacto en 0 (no se va a velocidades negativas) — coincide con el
  comportamiento real observado en los datos.
- `compute_control()` responde correctamente en ambos extremos (pide
  u≈+1 para acelerar desde parado, u≈-1 para frenar a fondo).
