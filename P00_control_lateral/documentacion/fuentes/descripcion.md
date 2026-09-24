::: {custom-style="Title"}
{{NOMBRE}}
:::

::: {custom-style="Subtitle"}
Descripción del software
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
| Director | {{DIRECTOR}} |
| Contacto | {{CONTACTO}} |
| Repositorio | {{REPOSITORIO}} |

::: {custom-style="Cubierta"}
{{CONTEXTO}}
:::

[[SALTO]]

[[TOC]]

[[SALTO]]

# 1. Información general del producto

La Tabla [[tab:info]] resume los datos de identificación del software. Los valores técnicos se tomaron del código fuente y de la configuración del repositorio, verificados el 23 de septiembre de 2026.

: Tabla [[tab:info]]. Información general del producto.

| Campo | Valor |
|---|---|
| Nombre | {{NOMBRE}} |
| Versión | {{VERSION}} |
| Tipo de producto | Software científico de escritorio |
| Área de aplicación | Control automático de vehículos, seguimiento de trayectoria, simulación en tiempo real |
| Autores | {{AUTORES}} |
| Afiliación | {{AFILIACION}} |
| Año | 2026 |
| Estado | Funcional, en uso para una campaña experimental en curso |
| Lenguaje | Python 3.14 |
| Plataforma | Windows 11 de 64 bits |
| Programas externos | Assetto Corsa con Steam y el controlador de dispositivo virtual vJoy |
| Hardware de verificación | Procesador AMD Ryzen 5 5600H, 7.3 GB de memoria, tarjeta NVIDIA GeForce GTX 1650 |
| Repositorio | {{REPOSITORIO}} |

# 2. Descripción general del software

{{NOMBRE}} es una plataforma de escritorio para ejecutar, registrar y analizar experimentos de seguimiento de trayectoria con controladores laterales de vehículos sobre el simulador de conducción Assetto Corsa. El software lee el estado del vehículo desde la memoria compartida del simulador, calcula en cada ciclo un comando de dirección con el controlador elegido y un comando de acelerador y freno con un lazo longitudinal común, y envía esos comandos al simulador a través de un dispositivo de juego virtual. Cada corrida queda guardada con su telemetría completa, la configuración usada y las huellas del código que la produjo.

El problema que aborda es la comparación de controladores de seguimiento de trayectoria bajo las mismas condiciones. Cuando dos controladores se prueban con plantas, perfiles de velocidad, lazos longitudinales o registros distintos, las diferencias observadas pueden deberse a esas piezas y no a la ley de control. El software fija esas piezas comunes y deja como única diferencia entre corridas el controlador lateral y sus parámetros. Incluye tres controladores, Pure Pursuit, Stanley y un control predictivo basado en modelo (MPC) con modelo bicicleta cinemático, que comparten una interfaz de programación única.

El software se desarrolló como herramienta de ejecución para un estudio sobre el desempeño de esos tres controladores en función de la velocidad y de la curvatura del trazado. La herramienta no es el resultado de ese estudio. Permite correr vueltas individuales con cualquier controlador y perfil de velocidad, generar y ejecutar planes aleatorizados con semilla para la sintonía y la campaña y un plan por bloques para el piloto, verificar la plataforma con una lista de puntos de control, y calcular métricas por vuelta y por región de curvatura. Está pensado para investigadores y estudiantes de control automático que necesitan una planta vehicular externa, con dinámica que no programaron ellos mismos, para probar leyes de control en tiempo real.

La Figura [[fig:arquitectura]] muestra los componentes principales y el flujo de información entre la ventana del lanzador, el proceso de corrida, el simulador y los archivos de salida.

![Figura [[fig:arquitectura]]. Arquitectura general del software. Los bloques grises son programas de terceros.](figuras/fig_arquitectura.png){width=15.5cm}

## 2.1. Especificación de requisitos de hardware y software

### Requisitos para el usuario final

El software se ejecutó y verificó en un computador portátil con Windows 11 Home de 64 bits, procesador AMD Ryzen 5 5600H de 6 núcleos y 12 hilos, 7.3 GB de memoria y tarjeta gráfica NVIDIA GeForce GTX 1650. Esa es la única configuración verificada y se toma como referencia. El usuario necesita además los programas de la Tabla [[tab:req]].

: Tabla [[tab:req]]. Programas necesarios para la operación con el simulador.

| Programa | Uso | Observación |
|---|---|---|
| Steam y Assetto Corsa | Planta vehicular y fuente del estado | Programa comercial de Kunos Simulazioni, con la pista Monza y el vehículo Alfa Romeo Giulietta QV |
| vJoy | Dispositivo de juego virtual que recibe dirección, acelerador y freno | Configurado como dispositivo 1 y asignado como control en el simulador |
| Python 3.14 | Intérprete | Verificado con la versión 3.14.7 |
| numpy 2.5.3, scipy 1.18.1, osqp 1.1.3, pyvjoy 1.0.1 | Librerías | Versiones fijadas en requirements.txt |

Cada corrida guardada ocupa entre 4.2 y 5.0 MB en disco, según las 36 corridas del primer bloque de la campaña medidas el 23 de septiembre de 2026. Una campaña de 90 vueltas requiere del orden de 450 MB.

### Entorno de software en modo desarrollo

El código está escrito en Python y usa tkinter, incluido en la distribución estándar, para la interfaz gráfica. El análisis estadístico usa scipy. El problema de optimización del MPC se resuelve con OSQP [[ref:stellato2020]]. Las pruebas automáticas se ejecutan como guiones de Python sin marco de pruebas externo y no requieren el simulador.

### Requisito de operación sin red

El software opera en un solo equipo y no usa red. La comunicación con el simulador se hace por memoria compartida de Windows y la salida de comandos por el controlador de vJoy, ambos locales.

## 2.2. Justificación de los requisitos

La dependencia de Windows viene de dos componentes externos. Assetto Corsa publica su estado en archivos de memoria compartida de Windows, y vJoy es un controlador de dispositivo para ese sistema operativo. El ciclo de control tiene un periodo nominal de 50 ms, fijado en `configs/base.json`, y el equipo debe ejecutar el simulador y el cálculo del controlador dentro de ese periodo. En las corridas de verificación del 15 de septiembre de 2026 se midió un periodo medio de 50.3 ms y tiempos de cómputo de los controladores por debajo de 0.9 ms, según `docs/STATE.md`, lo que deja margen en el equipo de referencia. La separación entre la ventana y el proceso de corrida evita que la interfaz gráfica compita con el ciclo de control dentro del mismo proceso.

# 3. Objetivos y área de aplicación

## 3.1. Objetivo general

Ofrecer una plataforma reproducible para ejecutar y comparar controladores laterales de vehículos sobre un simulador de conducción externo, con la misma planta, el mismo perfil de velocidad, el mismo lazo longitudinal y el mismo registro para todos los controladores, y con trazabilidad completa de cada corrida.

## 3.2. Área de aplicación

El software se aplica en control automático, en investigación sobre seguimiento de trayectoria y vehículos autónomos, y en la validación de controladores sobre plantas que no fueron modeladas por quien diseña el controlador. También puede apoyar la docencia en control, porque permite observar en tiempo real el efecto de cambiar un controlador o un perfil de velocidad sin programar la dinámica del vehículo.

# 4. Funcionalidades principales

Las funcionalidades descritas en esta sección están implementadas en el código de `P00_control_lateral/`. La Tabla [[tab:func]] las agrupa por componente.

: Tabla [[tab:func]]. Funcionalidades principales y módulo que las implementa.

| Funcionalidad | Descripción | Módulo |
|---|---|---|
| Lectura del simulador | Lee las páginas de física, gráficos y datos estáticos de la memoria compartida y construye el estado del vehículo con posición, orientación, velocidad y posiciones de los ejes | `plataforma/memoria_ac.py` |
| Trazada y proyección | Carga la trazada de referencia, calcula orientación y curvatura suavizada, y proyecta la posición del vehículo, el eje trasero y el eje delantero para obtener error lateral y error angular | `plataforma/trazada.py` |
| Perfil de velocidad | Calcula la velocidad límite por curvatura y frenado y la escala por un factor según el perfil conservador, nominal o rápido | `plataforma/perfil.py` |
| Lazo longitudinal | Controlador PID con recorte de la integral y conformador de pedal con rampas de subida y bajada | `plataforma/longitudinal.py` |
| Controladores laterales | Pure Pursuit, Stanley y MPC cinemático con OSQP, con la misma interfaz | `controladores/` |
| Actuación | Entrada progresiva de la dirección a baja velocidad, límites de magnitud y tasa, y envío de dirección, acelerador y freno a vJoy | `plataforma/actuador.py` |
| Vuelta y abandono | Detección del cruce de meta por la posición normalizada, vuelta medida entre dos cruces, tiempo límite y abandono automático | `plataforma/vuelta.py`, `plataforma/abandono.py` |
| Regiones de curvatura | Clasificación de cada punto en región baja, media o alta según el radio | `plataforma/regiones.py` |
| Registro y procedencia | Telemetría de 70 columnas por ciclo, perfil usado, manifiesto con configuración, versiones y huellas SHA256 del código | `plataforma/registro.py`, `plataforma/procedencia.py` |
| Apertura del simulador | Plantilla de sesión con respaldo y restauración de la configuración del juego, apertura directa, clic en el menú y prueba de control por vJoy | `plataforma/juego_ac.py` |
| Lanzador | Ventana con pestañas para corridas individuales, planes de campaña, sintonía y piloto, verificación y ejecución de un script previo de MPC completo | `lanzador/` |
| Análisis | Métricas por vuelta y región, mapa de campaña con pruebas estadísticas, repetibilidad, selección de sintonía, identificación de plantas y de la cadena de dirección | `analisis/` |
| Pruebas | Siete archivos de pruebas automáticas que no requieren el simulador | `pruebas/` |

La trazada usada por la configuración base corresponde al circuito de Monza, con 3750 puntos, 5757.2 m de longitud y espaciado medio de 1.54 m. La Figura [[fig:trazada]] muestra la clasificación de sus puntos por región de curvatura, calculada con las mismas clases que usa el software durante una corrida.

![Figura [[fig:trazada]]. Trazada de Monza coloreada por región de curvatura, con el porcentaje de la longitud total en cada región. Generada por `documentacion/generar_figuras_doc.py` con la configuración base.](figuras/fig_trazada_regiones.png){width=11cm}

# 5. Adaptabilidad, extensibilidad y mantenibilidad

### Modularidad

El código se divide en cuatro paquetes con responsabilidades separadas. `plataforma/` contiene todo lo que es común a los controladores, `controladores/` contiene solo las leyes de control, `lanzador/` contiene la interfaz y la gestión de planes, y `analisis/` contiene el posprocesamiento. El proceso de corrida, `ejecutar_corrida.py`, une esas piezas y puede ejecutarse desde la línea de comandos sin la ventana.

### Incorporación de nuevos controladores

Todos los controladores heredan de `ControladorLateral`, definido en `controladores/base.py`. Reciben una estructura `EntradaLateral` con la trazada, la velocidad, la batalla, la orientación, las posiciones de los ejes, los errores en cada eje y el último ángulo aplicado, y devuelven una estructura `SalidaControlador` con el ángulo de rueda pedido y campos de diagnóstico. Añadir un controlador requiere una clase nueva con el método `calcular`, su registro en la fábrica `crear_controlador` de `controladores/__init__.py` y sus parámetros en la configuración. Los límites de dirección, el lazo longitudinal y el registro se aplican sin cambios.

### Incorporación de otras pistas y vehículos

La trazada se lee de un archivo CSV cuya ruta está en la configuración, y el vehículo esperado, los límites de batalla y los valores de respaldo también. En principio otra pista o vehículo requieren un archivo de trazada nuevo, una configuración nueva y una plantilla de sesión del simulador. Esa extensión no se ha probado. La constante de la cadena de dirección y los parámetros de los controladores se identificaron y sintonizaron para el Alfa Romeo Giulietta QV en Monza, y otro vehículo requeriría repetir esas mediciones.

### Configuración y parametrización

Los parámetros de una corrida están en archivos JSON. `configs/base.json` reúne el periodo de muestreo, la trazada, las regiones, los datos del vehículo, los límites de dirección, el perfil de velocidad, el lazo longitudinal, los parámetros de los controladores, el abandono, la vuelta y las métricas. `configs/juego_ac.json` reúne la ruta del simulador y los tiempos de la preparación. Una corrida puede usar otro archivo de configuración elegido desde la interfaz, y el manifiesto guarda su contenido y su huella.

### Facilidad de mantenimiento

Cada módulo empieza con una descripción de su propósito y de las decisiones que implementa, con la fecha de verificación cuando corresponde. Las pruebas automáticas cubren la plataforma, el lanzador, los planes, el análisis de campaña, el abandono y la preparación del simulador.

### Extensiones ya realizadas

Durante el desarrollo se añadieron, sobre la versión inicial, la entrada progresiva de la dirección, la detección del cruce por posición normalizada, el abandono automático, la apertura del simulador desde el lanzador, las tandas encadenadas de sintonía, piloto y campaña, y el campo *Sesiones seguidas*. Cada cambio quedó registrado con su motivo en `docs/DECISIONS.md` y en `docs/P00_PLAN_CONTROLADORES.md`.

# 6. Robustez, desempeño y consistencia

## 6.1. Robustez

El software incorpora varios mecanismos para que una falla no pierda datos ni deje el vehículo en un estado inseguro dentro del simulador.

* El registro se acumula en memoria y se escribe en disco al terminar la corrida por cualquier motivo, incluida la interrupción por el usuario o por el lanzador. Al salir, la dirección se centra y los pedales se liberan.
* El abandono automático cierra la corrida si el vehículo queda por debajo de 5 km/h durante más de 8 s, o con tres o más ruedas fuera de la pista durante más de 3 s, después de superar 30 km/h y fuera de los pits.
* El tiempo límite de la corrida es de 420 s según la configuración base.
* Si el solucionador del MPC no entrega una solución válida, se aplica la secuencia anterior desplazada un paso y el ciclo queda marcado como respaldo en el registro.
* La dirección se retiene por debajo de 5 km/h y entra de forma gradual hasta 30 km/h, igual para los tres controladores.
* Si los puntos de contacto de las ruedas no son coherentes, la posición de los ejes se obtiene con un traslado de respaldo configurado, y el ciclo queda marcado.

## 6.2. Desempeño

La Tabla [[tab:desemp]] reúne las mediciones de desempeño disponibles, con la fuente de cada una. Todas se obtuvieron en el equipo de referencia.

: Tabla [[tab:desemp]]. Mediciones de desempeño de la plataforma y su fuente.

| Medición | Valor | Fuente |
|---|---|---|
| Periodo medio del ciclo en verificación | 50.3 ms | `docs/STATE.md`, fase 3, 15 de septiembre de 2026 |
| Tiempo de cómputo de los controladores en verificación | por debajo de 0.9 ms | `docs/STATE.md`, fase 3 |
| Batalla medida con los puntos de contacto | 2.633 m | `docs/STATE.md`, fase 3 |
| Constante de la cadena de dirección | 0.403 rad, identificada con dos corridas | nota de `configs/base.json` |
| Sobrevelocidad máxima con el PID congelado | 4.1 km/h | `docs/STATE.md`, 16 de septiembre de 2026 |
| Tiempo máximo permitido al solucionador del MPC | 40 ms | `configs/base.json` |

## 6.3. Consistencia y reproducibilidad

En la fase piloto se corrieron diez vueltas por controlador con la misma configuración y reinicio de sesión entre vueltas. La desviación estándar del error cuadrático medio del error lateral fue de 0.0027 m con Pure Pursuit, 0.0009 m con Stanley y 0.0007 m con el MPC, según `P00_control_lateral/piloto_fase5/repetibilidad_piloto.json`. Esos valores describen la variación entre vueltas en una sola jornada y no entre jornadas separadas. Los planes de sintonía y de campaña se generan una sola vez con semilla, el del piloto sigue un orden fijo por bloques, y los tres se guardan antes de ejecutar y no se sobrescriben. Cada manifiesto guarda la configuración completa, las versiones de Python y de las librerías, la huella de la configuración y las huellas de todos los archivos de código, de modo que una corrida puede asociarse con el código exacto que la produjo.

# 7. Interfaz y usabilidad

La interfaz es una ventana de escritorio titulada Lanzador de pruebas en Assetto Corsa, con dos pestañas principales. *Prueba de controladores* reúne cinco pestañas internas, *Corrida*, *Plan de campaña*, *Tanda de sintonía*, *Piloto* y *Verificación fase 3*. *MPC completo* ejecuta un script previo de los autores. La pestaña *Corrida* permite elegir fase, controlador, perfil, sesión, etiqueta, identificador de plan, tiempo máximo y archivo de configuración, iniciar la corrida con *Iniciar corrida* y detenerla guardando el registro con *Detener y guardar*. La salida del proceso aparece en *Salida en vivo* y, al terminar, un resumen legible aparece en *Resumen de la última corrida*.

La ventana no controla el vehículo. Arma el comando, prepara el simulador y lanza el proceso de corrida. Las acciones que pueden comprometer un experimento, como generar un plan, correr una corrida fuera de orden o repetir una ya hecha, piden confirmación. El Manual de usuario describe cada pestaña con capturas.

# 8. Integridad y seguridad de la información

El software trabaja con archivos locales y no transmite información. La integridad de los experimentos se apoya en estos mecanismos.

* Cada corrida se guarda en una carpeta propia con un identificador que incluye fase, sesión, controlador, perfil, fecha y hora, de modo que una corrida nueva no sobrescribe otra.
* El manifiesto guarda la huella SHA256 de la configuración, de cada archivo de código del software y de los dos archivos de origen de los que se copiaron piezas, y marca si alguno cambió desde la copia.
* Los planes no se sobrescriben. Una corrida de campaña exige identificador de plan, y la interfaz exige ese identificador en las fases con plan.
* Antes de escribir la plantilla de sesión en la carpeta de configuración del simulador, el software respalda la carpeta completa. El botón *Restaurar configuración del juego* copia de vuelta el último respaldo.
* El análisis de campaña filtra las corridas por fase, de modo que las corridas de prueba no entran en el mapa de campaña.

# 9. Portabilidad y compatibilidad

## 9.1. Plataformas compatibles

El software solo es operativo con el simulador en Windows. El análisis de corridas ya guardadas y las pruebas automáticas usan solo Python, numpy y scipy, y en principio pueden ejecutarse en otro sistema operativo, aunque eso no se ha verificado.

## 9.2. Dependencias relevantes

La Tabla [[tab:dep]] separa la autoría de cada componente. Las librerías de Python y los programas externos no son obra de los autores del software.

: Tabla [[tab:dep]]. Componentes propios, obra previa de los autores y componentes de terceros.

| Componente | Origen | Relación con el software |
|---|---|---|
| Código de `P00_control_lateral/` | Obra original de los autores | Objeto del registro |
| Lectura de memoria compartida, construcción de la trazada, generador de perfil, PID, conformador de pedal y salida a vJoy | Obra previa de los mismos autores, copiada el 15 de septiembre de 2026 desde `mpc_monza_Completo_barrido.py`, con huella registrada en `plataforma/procedencia.py` | Integrada, adaptada y ampliada |
| Script `mpc_monza_Completo_barrido.py` y `scripts/08_graficas_corrida.py` | Obra previa de los mismos autores, ajena al estudio de los controladores | Se ejecutan sin modificación desde la pestaña *MPC completo* |
| Campos ampliados de la página de física | Estructura declarada según la librería comunitaria mdjarv/assettocorsasharedmemory | Declaración de campos, sin código copiado de esa librería |
| Assetto Corsa y Steam | Kunos Simulazioni y Valve | Programas externos necesarios, no se distribuyen |
| vJoy y pyvjoy | Terceros | Controlador de dispositivo y librería de acceso |
| numpy, scipy, osqp | Terceros, licencias de código abierto | Librerías |

## 9.3. Portabilidad

El código no fija rutas absolutas del repositorio, que se calculan a partir de la ubicación de cada archivo. La ruta de instalación del simulador y la carpeta de configuración del juego están en `configs/juego_ac.json`, y el módulo de apertura busca otras instalaciones de Assetto Corsa en las bibliotecas de Steam cuando la ruta configurada no existe.

# 10. Documentación y soporte técnico

El software se acompaña de este documento, del Manual técnico y del Manual de usuario. El archivo `P00_control_lateral/README.md` resume la estructura, la convención de signos y los comandos de uso. Cada módulo tiene una descripción inicial en el código. El registro de decisiones de implementación está en `docs/P00_PLAN_CONTROLADORES.md` y `docs/DECISIONS.md`.

### Soporte técnico

El contacto para soporte es {{CONTACTO}}.

# 11. Pruebas, validación y desempeño

Las pruebas automáticas se ejecutaron el 23 de septiembre de 2026 en el equipo de referencia con Python 3.14.7. La Tabla [[tab:pruebas]] muestra el resultado de cada archivo. Ninguna prueba requiere el simulador ni vJoy.

: Tabla [[tab:pruebas]]. Resultado de las pruebas automáticas.

| Archivo | Qué verifica | Correctas |
|---|---|---|
| `prueba_abandono.py` | Criterios de abandono automático, esperas, exclusión de pits y lectura de sus valores | 12 de 12 |
| `prueba_humo.py` | Proyección, perfil escalado, respuesta de los tres controladores en recta y en círculo, entrada progresiva, límites, signo de vJoy, ejes por contactos, cruce de meta y registro | 21 de 21 |
| `prueba_juego_ac.py` | Plantilla de sesión, escalado del botón, respaldo, restauración, posición de salida y huellas | 20 de 20 |
| `prueba_lanzador.py` | Plan de campaña con semilla, estado del plan, resumen, verificación, construcción de la ventana y sus pestañas, tandas e identificador de plan | 17 de 17 |
| `prueba_mapa_campana.py` | Corrección de Holm, intervalo bootstrap, Friedman, Wilcoxon, umbral de mejora, esfuerzo y factibilidad | 26 de 26 |
| `prueba_plan_piloto.py` | Plan del piloto por bloques y escalera de agarre | 26 de 26 |
| `prueba_plan_sintonia.py` | Plan de sintonía con semilla y rangos | 14 de 14 |
| Total | | 136 de 136 |

Además de las pruebas automáticas, la plataforma se validó en el simulador con corridas de verificación, cuyos puntos de control están implementados en la pestaña *Verificación fase 3*. Entre ellos están el modelo de vehículo leído, la disponibilidad de la física ampliada, la batalla medida, la desalineación entre ejes y orientación, la velocidad al primer cruce de meta, el porcentaje de ciclos con tiempo de cómputo bajo 50 ms, el periodo medio, las saturaciones, el signo entre el comando de vJoy y el ángulo de dirección del simulador, y la reidentificación de la constante de la cadena de dirección. Al 23 de septiembre de 2026 la carpeta `data/raw/p00/corridas` contiene 9 corridas de verificación, 61 de sintonía, 37 de piloto, 2 de prueba y 36 de campaña, estas últimas del primer bloque de la campaña.

# 12. Impacto y utilización

## 12.1. Impacto en investigación

El software es la herramienta de ejecución de un estudio en curso que compara Pure Pursuit, Stanley y un MPC cinemático en función de la velocidad y de la curvatura sobre Assetto Corsa. A la fecha de este documento la campaña de ese estudio no ha terminado y sus resultados no se reportan aquí.

## 12.2. Impacto en docencia

El software se desarrolló en el marco de un trabajo de grado. Su uso en cursos no se ha realizado todavía y queda como posibilidad, dado que permite comparar controladores en tiempo real sin programar la dinámica del vehículo.

## 12.3. Usuarios y utilización

Los usuarios actuales son los autores del software. Las evidencias de utilización son las corridas guardadas en el repositorio, cada una con su manifiesto.

# 13. Contribución al estado del arte

## 13.1. Problema abordado

Los estudios que comparan Pure Pursuit, Stanley y MPC suelen hacerlo en entornos de simulación propios, con modelos del vehículo escritos por los mismos autores o en simuladores de propósito específico [[ref:zhang2025,lee2023,bousskoul2025]]. Otros trabajos muestran que el costo de cómputo y el retardo pueden cambiar la conclusión cuando el controlador pasa de la simulación a un vehículo real [[ref:kong2024]]. Una comparación que no fija las piezas comunes, perfil de velocidad, lazo longitudinal, límites de dirección y registro, mezcla el efecto del controlador con el de esas piezas.

## 13.2. Herramientas existentes

Existen herramientas abiertas que usan Assetto Corsa como plataforma de investigación. Remonda y colaboradores presentan un entorno con interfaz de aprendizaje por refuerzo, líneas base de MPC y datos de conductores humanos [[ref:remonda2024]]. Bockman y colaboradores presentan un conjunto de paquetes con interfaz de control, datos de visión y una pila de MPC [[ref:bockman2024]]. Ramlall y colaboradores usan el mismo motor para un simulador con varios participantes y telemetría sincronizada [[ref:ramlall2025]]. Estas herramientas se orientan a la conducción autónoma de competición, al aprendizaje automático o a estudios con conductores humanos.

## 13.3. Limitación que atiende el software

{{NOMBRE}} no busca reemplazar esas herramientas. Su alcance es más acotado y se centra en la comparación controlada de controladores laterales clásicos y predictivos. Reúne en una sola plataforma tres controladores con interfaz común, un lazo longitudinal y un perfil de velocidad compartidos, límites de dirección iguales para todos, planes aleatorizados con semilla que no se sobrescriben, verificación de la plataforma y un registro con huellas del código en cada corrida. Las formulaciones de Stanley [[ref:hoffmann2007]] y del modelo bicicleta cinemático [[ref:kong2015]] son conocidas. El aporte del software está en la ejecución controlada y trazable sobre una planta externa, no en las leyes de control.

## 13.4. Carácter científico

El software permite formular y responder preguntas experimentales sobre controladores con una planta cuya dinámica no programó el investigador, con periodo de control medido y con registro completo. El lazo longitudinal se sintonizó con reglas analíticas conocidas [[ref:skogestad2003]] sobre una planta identificada con telemetría. Sus limitaciones, una sola pista y un solo vehículo configurados, un modelo de predicción cinemático sin dinámica de llantas y la dependencia de un simulador comercial, se describen en el Manual técnico.

# 14. Aportes de los autores

{{APORTES}}

# Referencias

[[REFERENCIAS]]
