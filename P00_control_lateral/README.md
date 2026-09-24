# P00 control lateral

Código propio del artículo P00, comparación de Pure Pursuit, Stanley y MPC con modelo bicicleta cinemático sobre Assetto Corsa, con mapa por velocidad y curvatura.

No modifica ningún script existente del repositorio. Las piezas que ya funcionaban se copiaron desde `Model Predictive Control/Python/mpc_monza_Completo_barrido.py` y su huella queda en `plataforma/procedencia.py`.

Decisiones que implementa. `docs/HYPOTHESES.md` para el diseño del estudio y `docs/P00_PLAN_CONTROLADORES.md` para las decisiones de implementación I1 a I7.

## Estructura

* `plataforma/` lectura de Assetto Corsa, trazada y proyección, perfil de velocidad, rama longitudinal, interfaz de dirección y vJoy, registro y seguimiento de vuelta.
* `controladores/` Pure Pursuit, Stanley y MPC cinemático con la misma interfaz.
* `configs/base.json` parámetros comunes. Los valores de los controladores y del perfil base son iniciales de prueba y no están sintonizados.
* `ejecutar_corrida.py` una corrida con un controlador y un perfil.
* `analisis/` cobertura de regiones de curvatura y métricas por vuelta y región.
* `pruebas/` verificación sin Assetto Corsa.

## Convención de signos

Marco de la trazada, ángulos medidos con atan2 de z sobre x. ψ es la orientación de la carrocería, heading de Assetto Corsa más un desfase de 90 grados medido. δ positivo hace crecer ψ y corresponde a comando positivo de vJoy, verificado el 2026-09-15 en 37 corridas guardadas. e_y positivo significa vehículo a la izquierda de la trazada en ese marco y eψ es ψ menos la orientación de la trazada.

## Uso

La forma recomendada es el lanzador, con Assetto Corsa abierto y el vehículo en pista, desde la raíz del repositorio.

```
python P00_control_lateral/abrir_lanzador.py
```

El lanzador es una herramienta para ejecutar el experimento y no un aporte del artículo. No controla el vehículo. Arma el comando, ejecuta `ejecutar_corrida.py` como proceso aparte y muestra su salida, así el ciclo de 50 ms no comparte proceso con la ventana. Tiene tres pestañas.

* Corrida. Fase, controlador, perfil, sesión, etiqueta y configuración, salida en vivo, botón para detener guardando el registro y resumen al terminar.
* Plan de campaña. Genera una sola vez con semilla el orden aleatorio de las 10 sesiones en `configs/plan_campana.json`, muestra el estado de cada corrida y la siguiente, y pide confirmación para correr fuera de orden o repetir.
* Verificación fase 3. Evalúa una corrida guardada contra los puntos de la fase 3, incluida la reidentificación de la constante de la cadena de dirección con la batalla medida.

También se puede correr directo.

```
python P00_control_lateral/ejecutar_corrida.py --controlador stanley --perfil conservador --sesion 0 --fase verificacion
```

Cobertura de regiones de curvatura.

```
python P00_control_lateral/analisis/regiones_curvatura.py
```
