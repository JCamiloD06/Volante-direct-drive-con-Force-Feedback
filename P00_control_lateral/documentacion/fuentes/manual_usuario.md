::: {custom-style="Title"}
{{NOMBRE}}
:::

::: {custom-style="Subtitle"}
Manual de usuario del programa
:::

::: {custom-style="Cubierta"}
{{SUBTITULO}}
:::

| | |
|---|---|
| Autores | {{AUTORES}} |
| Versión | {{VERSION}} |
| Fecha | {{FECHA}} |
| Afiliación institucional | {{AFILIACION}} |
| Contacto | {{CONTACTO}} |
| Repositorio | {{REPOSITORIO}} |

[[SALTO]]

[[TOC]]

[[SALTO]]

# 1. Introducción

## 1.1. Propósito del software

{{NOMBRE}} permite correr vueltas de un vehículo controlado por software en el simulador Assetto Corsa, con uno de tres controladores de dirección, Pure Pursuit, Stanley o un control predictivo basado en modelo, y guardar la telemetría de cada vuelta para compararla después. Todas las corridas usan el mismo perfil de velocidad, el mismo control de acelerador y freno, los mismos límites de dirección y el mismo registro, de modo que lo único que cambia entre corridas es el controlador y sus parámetros.

## 1.2. Contexto general

El software se desarrolló para un estudio sobre controladores de seguimiento de trayectoria en función de la velocidad y de la curvatura. Este manual explica su uso general y no depende de ese estudio. Los ejemplos del capítulo 6 son demostraciones de uso y no sustituyen los resultados de ningún estudio.

## 1.3. Alcance del documento

El manual cubre la instalación, la interfaz, el uso básico, seis experimentos paso a paso, la ubicación de los resultados, las buenas prácticas y la solución de problemas. Los detalles internos, ecuaciones y formatos están en el Manual técnico.

# 2. Descripción general del sistema

## 2.1. Qué necesita el usuario

El usuario necesita un computador con Windows, Assetto Corsa instalado desde Steam con el circuito de Monza y el Alfa Romeo Giulietta QV, el controlador vJoy configurado como dispositivo 1, Python 3.14 con las librerías del archivo `requirements.txt` y una copia del repositorio. No necesita volante físico, porque el software maneja el vehículo a través de vJoy.

## 2.2. Qué hace el sistema

Al pulsar *Iniciar corrida*, el software abre el simulador con una sesión fija, deja el vehículo en la salida, comprueba que vJoy lo controla y arranca la corrida. Durante la corrida el vehículo sale de pits, completa una vuelta de salida y luego una vuelta medida entre dos cruces de la línea de meta. Al terminar, guarda la corrida en una carpeta propia y muestra un resumen.

## 2.3. Para qué sirve

Sirve para comparar controladores de dirección bajo las mismas condiciones, para probar el efecto del perfil de velocidad, para ejecutar planes experimentales con orden aleatorio y semilla, y para verificar que la plataforma funciona antes de un experimento.

## 2.4. Principales capacidades

* Corridas individuales con tres controladores y tres perfiles de velocidad, conservador, nominal y rápido.
* Apertura automática del simulador con respaldo de su configuración.
* Salida en vivo y resumen de cada corrida.
* Planes de campaña, sintonía y piloto, con tandas que corren varias vueltas seguidas.
* Verificación de una corrida guardada contra una lista de puntos de control.
* Métricas por vuelta y por región de curvatura desde la línea de comandos.

# 3. Acceso y requisitos

## 3.1. Requisitos de software

La Tabla [[tab:software]] lista los programas necesarios.

: Tabla [[tab:software]]. Programas necesarios.

| Programa | Versión verificada |
|---|---|
| Windows 11 de 64 bits | 10.0.26200 |
| Steam y Assetto Corsa | la instalada en el equipo de referencia |
| vJoy | la instalada en el equipo de referencia |
| Python | 3.14.7 |
| numpy, scipy, osqp, pyvjoy | 2.5.3, 1.18.1, 1.1.3 y 1.0.1 |

## 3.2. Requisitos de hardware

El software se verificó en un portátil con procesador AMD Ryzen 5 5600H, 7.3 GB de memoria y tarjeta NVIDIA GeForce GTX 1650. Cada corrida guardada ocupa alrededor de 5 MB.

## 3.3. Instalación y ejecución del programa

1. Instale Steam, Assetto Corsa y vJoy.
2. Instale Python 3.14.
3. Abra una terminal en la raíz del repositorio y ejecute `python -m pip install -r P00_control_lateral/requirements.txt`.
4. Compruebe que la carpeta del juego contiene el archivo `steam_appid.txt` con el número 244210. Sin él, Steam abre el lanzador oficial del juego en lugar de la sesión.
5. Revise en `P00_control_lateral/configs/juego_ac.json` que la ruta del juego es la de su equipo.
6. Ejecute `python P00_control_lateral/abrir_lanzador.py` desde la raíz del repositorio.

## 3.4. Configuración de vJoy en el simulador

La plantilla de sesión que escribe el software ya asigna vJoy como volante del simulador. Esa plantilla se tomó de una sesión preparada con Content Manager y contiene la pista, el vehículo, las ayudas y los controles. El usuario no necesita configurar el control a mano si usa la plantilla incluida.

# 4. Interfaz gráfica

## 4.1. Ventana principal

La ventana se titula Lanzador de pruebas en Assetto Corsa. Tiene dos pestañas principales, *Prueba de controladores* y *MPC completo*, y una barra de estado inferior que muestra a la izquierda el estado de la corrida y a la derecha el estado del juego. La Figura [[fig:inicio]] muestra la ventana al abrirse.

![Figura [[fig:inicio]]. Ventana del lanzador al abrirse.](capturas/01_lanzador_inicio.png){width=15.5cm}

## 4.2. Pestaña Corrida

La pestaña *Corrida*, dentro de *Prueba de controladores*, configura y ejecuta una corrida individual. La Figura [[fig:corrida]] la muestra y la Tabla [[tab:campos]] describe sus campos.

![Figura [[fig:corrida]]. Pestaña Corrida.](capturas/02_pestana_corrida.png){width=15.5cm}

: Tabla [[tab:campos]]. Campos de la pestaña Corrida.

| Campo | Función |
|---|---|
| *Fase* | verificacion, sintonia, piloto, campana o prueba. Para uso libre se elige prueba |
| *Controlador* | pure_pursuit, stanley o mpc_cinematico |
| *Perfil* | conservador, nominal o rapido |
| *Sesión* | número de sesión. Las sesiones 1 a 10 están reservadas a la campaña y la ventana pide confirmación para usarlas en otra fase |
| *Etiqueta* | texto libre que queda en el manifiesto |
| *Id del plan* | identificador de la fila del plan. Es obligatorio en las fases campana, sintonia y piloto |
| *Tiempo máximo s* | tiempo máximo opcional. Si se deja vacío se usa el tiempo límite de la configuración, 420 s |
| *Configuración* | archivo JSON de configuración, por omisión `configs/base.json`. El botón *Elegir* abre un selector |
| *No verificar vehículo, solo pruebas* | omite la comprobación del modelo de vehículo. Solo se permite en las fases verificacion y prueba |

Los botones son *Iniciar corrida*, que abre el juego, pulsa el volante del menú y arranca la vuelta, *Detener y guardar*, que pide al proceso de corrida que termine y guarde, y *Restaurar configuración del juego*, que copia de vuelta el último respaldo de la configuración del simulador. Debajo están los paneles *Salida en vivo*, con la salida del proceso, y *Resumen de la última corrida*.

## 4.3. Pestaña Plan de campaña

Muestra el plan de campaña en una tabla con las columnas id_plan, sesion, posicion, controlador, perfil, estado, intentos y carpeta. Los botones son *Generar plan*, que crea el plan una sola vez con semilla y no lo sobrescribe si ya existe, *Actualizar*, *Usar siguiente*, que carga en la pestaña *Corrida* la siguiente corrida pendiente, *Usar seleccionada*, que carga la fila elegida y pide confirmación si está fuera de orden o ya se hizo, y *Correr la sesión*, que corre en tanda las corridas pendientes. El campo *Sesiones seguidas* indica cuántas sesiones encadena la tanda. La Figura [[fig:plan]] muestra la pestaña con el plan vigente.

![Figura [[fig:plan]]. Pestaña Plan de campaña con el plan vigente.](capturas/06_plan_campana.png){width=15.5cm}

## 4.4. Pestaña Tanda de sintonía

Muestra el plan de sintonía con las columnas id_plan, controlador, parametros, estado y carpeta. Los botones son *Generar plan*, *Actualizar*, *Usar seleccionada*, *Iniciar tanda* y *Detener tanda*. La Figura [[fig:sintonia]] la muestra.

![Figura [[fig:sintonia]]. Pestaña Tanda de sintonía.](capturas/07_tanda_sintonia.png){width=15.5cm}

## 4.5. Pestaña Piloto

Muestra el plan del piloto con las columnas id_plan, bloque, agarre, controlador, perfil, estado y carpeta. Los botones son *Generar plan*, *Añadir bloque tiempo límite*, *Actualizar*, *Repetir seleccionada* e *Iniciar tanda*. La Figura [[fig:piloto]] la muestra.

![Figura [[fig:piloto]]. Pestaña Piloto.](capturas/08_piloto.png){width=15.5cm}

## 4.6. Pestaña Verificación fase 3

Permite elegir una corrida guardada en la lista *Corrida* y pulsar *Evaluar*. El botón *Actualizar lista* vuelve a leer las carpetas de corridas. El resultado aparece en una tabla con las columnas punto, valor, criterio y resultado. Los puntos que cumplen se marcan en verde y los que no cumplen en rojo. Los puntos informativos no tienen un umbral decidido y se revisan a criterio del usuario.

## 4.7. Pestaña MPC completo

Ejecuta un script previo de los autores, `Model Predictive Control/Python/mpc_monza_Completo_barrido.py`, que controla dirección y velocidad con un MPC. Ese script no forma parte del código de `P00_control_lateral/` y se ejecuta sin modificarlo. La pestaña muestra su ruta y su huella, la casilla *Activar el MPC automáticamente, equivale a pulsar Enter cuando el carro está listo*, y los botones *Iniciar MPC completo*, *Activar MPC*, *Detener y guardar* y *Generar gráficas*. La Figura [[fig:mpc]] la muestra.

![Figura [[fig:mpc]]. Pestaña MPC completo.](capturas/10_mpc_completo.png){width=15.5cm}

# 5. Uso básico

## 5.1. Preparar el simulador

No es necesario abrir el simulador antes. El lanzador lo abre al pulsar *Iniciar corrida*. Si el simulador ya está abierto, el lanzador lo usa, pero en ese caso la sesión cargada puede no venir de la plantilla y así queda anotado en el manifiesto.

## 5.2. Configurar e iniciar una corrida

1. Abra una terminal en la raíz del repositorio y ejecute `python P00_control_lateral/abrir_lanzador.py`.
2. En la pestaña principal *Prueba de controladores* seleccione la pestaña *Corrida*.
3. En *Fase* elija prueba.
4. En *Controlador* elija el controlador.
5. En *Perfil* elija el perfil de velocidad.
6. Deje *Sesión* en 0.
7. Escriba una *Etiqueta* que identifique la corrida.
8. Pulse *Iniciar corrida* y no use el teclado ni el ratón mientras el lanzador prepara el juego.
9. Observe el avance en *Salida en vivo*.

## 5.3. Detener una corrida

La corrida termina sola al completar la vuelta medida, al superar el tiempo máximo o por abandono automático. Para detenerla antes pulse *Detener y guardar*. Si la corrida no termina en 8 s, la ventana pregunta si se fuerza el cierre, y en ese caso el registro de esa corrida se pierde.

## 5.4. Revisar el resumen y ubicar los archivos

Al terminar, *Resumen de la última corrida* muestra el identificador, el controlador, el perfil, la fase, el vehículo y la pista leídos del simulador, si la vuelta se completó y su tiempo, el número de ciclos, los tiempos de cómputo, el periodo medio, las saturaciones, el respaldo del controlador, el estado del primer cruce de meta, las notas y la carpeta. La corrida queda en `data/raw/p00/corridas/` en una carpeta cuyo nombre empieza por la fase, sigue con la sesión, el controlador y el perfil, y termina con la fecha y la hora.

## 5.5. Evaluar y calcular métricas

1. En la pestaña *Verificación fase 3* pulse *Actualizar lista*.
2. Elija la corrida en *Corrida*.
3. Pulse *Evaluar* y revise la tabla.
4. Para las métricas por región, ejecute desde la raíz del repositorio `python P00_control_lateral/analisis/metricas_vuelta.py` seguido de la ruta de la carpeta de la corrida. El guion escribe `metricas.json` dentro de esa carpeta.

# 6. Experimentos paso a paso

Los experimentos de este capítulo se hacen en fase prueba, con sesión 0 y con etiquetas que empiezan por doc, para que no se mezclen con los datos de ningún estudio. El análisis de campaña filtra por fase y no los incluye. Los resultados dependen de la sesión del simulador y del agarre dinámico de la pista, y una sola vuelta por configuración no permite conclusiones generales.

## 6.1. Experimento 1, corrida con Pure Pursuit

Objetivo. Mostrar una corrida completa desde la apertura del simulador hasta el resumen.

Configuración. Fase prueba, controlador pure_pursuit, perfil conservador, sesión 0, etiqueta doc_pp_conservador.

Procedimiento.

1. En la pestaña *Corrida* ajuste los campos como indica la configuración y compruebe que *Configuración* apunta a `configs/base.json`, como en la Figura [[fig:config]].
2. Pulse *Iniciar corrida*. El lanzador escribe la plantilla de sesión si hace falta, abre el juego, espera la sesión en vivo, comprueba que el vehículo está en la salida, pulsa el volante del menú y prueba el freno por vJoy.
3. Observe en *Salida en vivo* las líneas de preparación, que empiezan por [juego], y después las líneas del ciclo, una cada segundo, con tiempo, fase de la vuelta, error lateral, ángulo de dirección, velocidad y objetivo, tiempo de cómputo y posición normalizada, como en la Figura [[fig:vivo]].
4. Espere a que aparezca Vuelta medida completada y el resumen, como en la Figura [[fig:resumen]].

![Figura [[fig:config]]. Configuración de la corrida del experimento 1.](capturas/03_configuracion_corrida.png){width=15.5cm}

![Figura [[fig:vivo]]. Salida en vivo durante la corrida del experimento 1.](capturas/04_salida_en_vivo.png){width=15.5cm}

![Figura [[fig:resumen]]. Resumen de la corrida del experimento 1.](capturas/05_resumen_corrida.png){width=15.5cm}

Resultados. [RESULTADO PENDIENTE DE EJECUCIÓN]

Archivos. La carpeta de la corrida contiene `telemetria.csv`, `perfil.csv` y `manifiesto.json`.

## 6.2. Experimento 2, corrida con Stanley

Objetivo. Mostrar que el cambio de controlador se hace solo con el campo *Controlador*, con la misma plataforma.

Configuración. Igual al experimento 1 con controlador stanley y etiqueta doc_st_conservador.

Procedimiento. Repita los pasos del experimento 1 cambiando solo *Controlador* y *Etiqueta*.

Resultados. [RESULTADO PENDIENTE DE EJECUCIÓN]

## 6.3. Experimento 3, corrida con el MPC cinemático

Objetivo. Mostrar una corrida con el MPC y los indicadores propios de este controlador.

Configuración. Igual al experimento 1 con controlador mpc_cinematico y etiqueta doc_mpc_conservador.

Procedimiento. Repita los pasos del experimento 1. En el registro del MPC, la columna `ctrl_estado` guarda el estado del solucionador, `ctrl_iteraciones` las iteraciones, `ctrl_respaldo` si se usó la secuencia anterior, `ctrl_extra1` el tiempo del solucionador en milisegundos y `ctrl_extra2` el valor del costo.

Resultados. [RESULTADO PENDIENTE DE EJECUCIÓN]

## 6.4. Experimento 4, comparación de controladores

Objetivo. Mostrar cómo se comparan las tres corridas anteriores con los guiones de análisis.

Procedimiento.

1. Ejecute `metricas_vuelta.py` sobre cada una de las tres carpetas.
2. Compare en cada `metricas.json` el error cuadrático medio del error lateral de la vuelta y de cada región, el tiempo de vuelta y la tasa de dirección.

Resultados. [RESULTADO PENDIENTE DE EJECUCIÓN]

Interpretación. Es una demostración de uso con una vuelta por controlador, sin valor inferencial.

## 6.5. Experimento 5, comparación de perfiles

Objetivo. Mostrar un experimento comparativo cambiando un solo parámetro desde la interfaz, sin editar la configuración.

Configuración. Igual al experimento 2 con perfil nominal y etiqueta doc_st_nominal.

Procedimiento. Corra la vuelta, calcule sus métricas y compárelas con las del experimento 2.

Resultados. [RESULTADO PENDIENTE DE EJECUCIÓN]

## 6.6. Experimento 6, verificación de la plataforma

Objetivo. Mostrar qué revisa la pestaña *Verificación fase 3* sobre una corrida.

Procedimiento. Evalúe la corrida del experimento 1 como indica la sección 5.5.

La pestaña revisa el vehículo leído, la disponibilidad de la física ampliada, los ciclos con contactos válidos, la batalla medida, la desalineación entre ejes y orientación, la deriva, si la vuelta medida se completó, la velocidad y el error lateral al primer cruce, los ciclos con tiempo de cómputo bajo 50 ms, el periodo medio, las saturaciones, el respaldo del controlador, las salidas de pista, el signo entre el comando de vJoy y el ángulo de dirección del simulador, y la constante de la cadena de dirección reidentificada con la batalla medida. La Figura [[fig:verif]] muestra el resultado.

![Figura [[fig:verif]]. Pestaña Verificación fase 3 con la evaluación de la corrida del experimento 1.](capturas/09_verificacion_fase3.png){width=15.5cm}

Resultados. [RESULTADO PENDIENTE DE EJECUCIÓN]

# 7. Visualización de resultados

La ventana muestra el resumen de cada corrida y la tabla de verificación. Las gráficas de un estudio se generan fuera de la ventana con los guiones del repositorio, `scripts/16_p00_figuras.py` para figuras y `scripts/17_p00_tablas.py` para tablas, que leen las salidas del análisis. El botón *Generar gráficas* de la pestaña *MPC completo* solo aplica a las corridas del script previo.

# 8. Almacenamiento y exportación

Cada corrida se guarda en `data/raw/p00/corridas/` con tres archivos. `telemetria.csv` tiene una fila por ciclo de 50 ms con 70 columnas y puede abrirse con cualquier hoja de cálculo o con Python. `perfil.csv` tiene el perfil de velocidad usado en cada punto de la trazada. `manifiesto.json` describe la corrida, su configuración, las versiones del software y las huellas del código. El guion de métricas añade `metricas.json`. Los respaldos de la configuración del simulador quedan en `data/raw/p00/respaldos_cfg_ac` y el registro de cada tanda en `data/raw/p00/tandas`.

# 9. Buenas prácticas

* Use la fase prueba y sesiones fuera del rango 1 a 10 para todo lo que no pertenezca a un plan.
* Use etiquetas descriptivas y únicas.
* No genere de nuevo un plan que ya existe. El software no lo sobrescribe, y crear otro archivo de plan rompe la trazabilidad del orden aleatorio.
* No cambie las condiciones de sesión de la plantilla en medio de un experimento, porque el agarre y el viento cambian el resultado.
* No use el teclado ni el ratón mientras el lanzador prepara el juego o mientras corre una tanda.
* Haga copia de seguridad de `data/raw/p00/corridas` al terminar cada jornada.
* Si alguna vez restauró la configuración del juego, compruebe la sesión antes de la siguiente corrida.

# 10. Solución de problemas

La Tabla [[tab:problemas]] reúne los problemas conocidos, observados durante el desarrollo o previstos en el código, con su solución.

: Tabla [[tab:problemas]]. Problemas conocidos y solución.

| Síntoma | Causa | Solución |
|---|---|---|
| Se abre el lanzador oficial de Steam en lugar de la sesión | Falta `steam_appid.txt` en la carpeta del juego | Crear el archivo con el número 244210. El lanzador oficial reescribe la configuración del juego, que puede restaurarse con *Restaurar configuración del juego* con el juego cerrado |
| El juego queda en el menú de boxes | El juego espera el botón del volante del menú | El lanzador lo pulsa. Si el clic no entra, la salida muestra PULSA TÚ EL VOLANTE y el usuario tiene 120 s para pulsarlo |
| Mensaje de que el carro no está en la salida | La sesión abierta no está en la posición de salida | Aceptar la opción de cerrar y reabrir el juego que ofrece la ventana |
| vJoy no controla el carro | vJoy no está instalado, no es el dispositivo 1 o no está asignado en el juego | Revisar la instalación de vJoy y la plantilla de controles |
| Vehículo leído distinto del esperado, corrida cancelada | La sesión no tiene el Giulietta QV | Revisar la plantilla de sesión. La casilla *No verificar vehículo, solo pruebas* omite la comprobación solo en verificacion y prueba |
| En la pestaña *MPC completo* el carro se queda sin acelerar | Bloqueo conocido del MPC de velocidad del script original | La pestaña ejecuta la versión con salvaguarda, que en la prueba del 16 de septiembre de 2026 completó la vuelta con 4 activaciones de la salvaguarda |
| La vuelta no termina | El vehículo no cruzó la meta o quedó detenido | La corrida se cierra sola por abandono o por el tiempo límite de 420 s y guarda el registro |
| La corrida no responde a Detener y guardar | El juego está en pausa o congelado | Esperar la pregunta de forzar cierre. Forzar el cierre pierde el registro |
| Aviso de Custom Shaders Patch | El complemento está en la carpeta del juego | Puede cambiar clima y temperatura de pista. Retirarlo si se necesita la sesión exacta de la plantilla |
| Respaldo del solucionador en el MPC | OSQP no resolvió dentro de 40 ms | Revisar el porcentaje de respaldo en el resumen y en la verificación |
| No se puede iniciar en fase campana, sintonia o piloto | Falta *Id del plan* | Cargar la corrida desde la pestaña del plan correspondiente |
