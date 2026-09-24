::: {custom-style="Title"}
{{NOMBRE}}
:::

::: {custom-style="Subtitle"}
Manual técnico del programa
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

{{NOMBRE}} ejecuta experimentos de seguimiento de trayectoria con controladores laterales de vehículos sobre el simulador Assetto Corsa. En cada ciclo lee el estado del vehículo, calcula la dirección con el controlador elegido y el acelerador y el freno con un lazo longitudinal común, envía esos comandos por un dispositivo virtual y registra la telemetría. Incluye además la gestión de planes experimentales con semilla y el análisis de las corridas guardadas.

## 1.2. Contexto de desarrollo

El software se escribió para ejecutar un estudio que compara Pure Pursuit, Stanley y un control predictivo basado en modelo con modelo bicicleta cinemático, en función de la velocidad y de la curvatura, con el circuito de Monza y el Alfa Romeo Giulietta QV de Assetto Corsa. Varias piezas de la plataforma, la lectura de memoria compartida, la construcción de la trazada, el generador de perfil de velocidad, el PID, el conformador de pedal y la salida a vJoy, provienen de un script previo de los mismos autores, `Model Predictive Control/Python/mpc_monza_Completo_barrido.py`, escrito para pruebas anteriores ajenas a ese estudio. Esas piezas se copiaron el 15 de septiembre de 2026, se adaptaron y se ampliaron, y la huella del archivo de origen queda registrada en `plataforma/procedencia.py`.

## 1.3. Alcance del documento

Este manual describe la arquitectura, los componentes, los modelos y ecuaciones implementados, los métodos numéricos, el lazo de tiempo real, la gestión de experimentos, el análisis, los formatos de almacenamiento, las limitaciones y el despliegue. Todo lo descrito corresponde al código de `P00_control_lateral/` verificado el 23 de septiembre de 2026. Los nombres de archivos, clases, funciones y parámetros son los del código. El uso de la interfaz se describe en el Manual de usuario.

# 2. Arquitectura del sistema

## 2.1. Descripción de la arquitectura

El software tiene dos procesos. El primero es la ventana del lanzador, escrita con tkinter, que gestiona planes, prepara el simulador y muestra resultados. El segundo es el proceso de corrida, `ejecutar_corrida.py`, que contiene el ciclo de control. La ventana lanza el proceso de corrida con `subprocess.Popen`, lee su salida estándar en un hilo aparte y la muestra en pantalla. Esa separación hace que el ciclo de control de 50 ms no comparta proceso ni intérprete con la interfaz gráfica, de modo que un redibujado de la ventana no retrasa un ciclo. La detención se pide creando un archivo de detención que el proceso de corrida revisa cada cinco ciclos, lo que permite cerrar la corrida guardando el registro.

La Figura [[fig:arquitectura]] muestra los componentes y el sentido de la información.

![Figura [[fig:arquitectura]]. Arquitectura del software. Los bloques grises son programas de terceros.](figuras/fig_arquitectura.png){width=15.5cm}

## 2.2. Componentes funcionales

* Interfaz y gestión de experimentos, en `lanzador/`.
* Preparación del simulador, en `plataforma/juego_ac.py`.
* Plataforma común a los controladores, en `plataforma/`.
* Controladores laterales, en `controladores/`.
* Proceso de corrida, en `ejecutar_corrida.py`.
* Análisis y posprocesamiento, en `analisis/`.
* Configuración, en `configs/`.
* Pruebas automáticas, en `pruebas/`.

## 2.3. Estructura del proyecto

La Tabla [[tab:estructura]] lista los directorios y archivos del software con su responsabilidad.

: Tabla [[tab:estructura]]. Estructura del proyecto.

| Directorio o archivo | Responsabilidad |
|---|---|
| `abrir_lanzador.py` | Punto de entrada de la interfaz gráfica |
| `ejecutar_corrida.py` | Una corrida con un controlador y un perfil, ciclo de control y registro |
| `lanzador/app.py` | Ventana principal con sus pestañas |
| `lanzador/corridas.py` | Lectura de corridas guardadas, estado del plan, resumen y puntos de verificación |
| `lanzador/plan.py` | Plan de campaña aleatorizado con semilla |
| `lanzador/plan_sintonia.py` | Plan de sintonía por búsqueda aleatoria con semilla |
| `lanzador/plan_piloto.py` | Plan del piloto en dos bloques |
| `lanzador/proceso_consola.py` | Ejecución con consola oculta y detención con Ctrl+C real para el script previo |
| `plataforma/memoria_ac.py` | Lectura de memoria compartida y estado del vehículo |
| `plataforma/trazada.py` | Trazada de referencia, curvatura y proyección |
| `plataforma/perfil.py` | Perfil de velocidad y su escalado |
| `plataforma/longitudinal.py` | PID longitudinal y conformador de pedal |
| `plataforma/actuador.py` | Límites de dirección, entrada progresiva y salida a vJoy |
| `plataforma/vuelta.py` | Seguimiento de la vuelta medida |
| `plataforma/abandono.py` | Abandono automático |
| `plataforma/regiones.py` | Regiones de curvatura |
| `plataforma/registro.py` | Telemetría, perfil y manifiesto de cada corrida |
| `plataforma/procedencia.py` | Huellas de archivos de origen y de código |
| `plataforma/juego_ac.py` | Apertura y preparación de Assetto Corsa |
| `plataforma/angulos.py` | Envolvimiento de ángulos |
| `controladores/base.py` | Interfaz común de los controladores |
| `controladores/pure_pursuit.py`, `stanley.py`, `mpc_cinematico.py` | Leyes de control |
| `analisis/` | Métricas, mapa de campaña, repetibilidad, sintonía, plantas y cadena de dirección |
| `configs/` | Configuración base, del simulador, planes y plantilla de sesión |
| `pruebas/` | Siete archivos de pruebas automáticas |
| `requirements.txt` | Versiones de las librerías |

## 2.4. Flujo general de ejecución

Una corrida iniciada desde la ventana sigue este orden. La ventana valida los campos y arma los argumentos de `ejecutar_corrida.py`. En un hilo aparte prepara el simulador con `juego_ac.preparar`, cuyo detalle se muestra en la Figura [[fig:preparacion]]. La información de la preparación se escribe en un archivo JSON temporal. Después la ventana lanza el proceso de corrida con ese archivo como argumento, lee su salida línea por línea y, cuando el proceso imprime la marca de corrida guardada, lee el manifiesto y muestra el resumen.

![Figura [[fig:preparacion]]. Secuencia de preparación del simulador que ejecuta el lanzador antes de cada corrida.](figuras/fig_preparacion_ac.png){width=15.5cm}

# 3. Componentes del software

## 3.1. Módulo lanzador

### 3.1.1. app.py

Define la clase `Lanzador`, derivada de `tk.Tk`, con dos pestañas principales. *Prueba de controladores* contiene las pestañas internas *Corrida*, *Plan de campaña*, *Tanda de sintonía*, *Piloto* y *Verificación fase 3*. *MPC completo* ejecuta el script previo `mpc_monza_Completo_barrido.py` sin modificarlo ni pasarle argumentos. Los métodos principales son `iniciar`, que valida y lanza la preparación, `_lanzar_corrida`, que crea el proceso, `detener`, que crea el archivo de detención, `_fin_proceso`, que muestra el resumen, y los métodos de tandas `iniciar_tanda`, `iniciar_tanda_piloto` e `iniciar_tanda_campana`, que encadenan corridas de un plan. Las fases válidas son verificacion, sintonia, piloto, campana y prueba. Las fases campana, sintonia y piloto exigen identificador de plan.

### 3.1.2. corridas.py

Lee las carpetas de corridas sin modificarlas. `escanear` lista las corridas con su manifiesto, `estado_plan` marca cada fila del plan como pendiente o hecha, `resumen_legible` arma el texto del resumen y `verificacion_fase3` calcula los puntos de verificación de una corrida. Una corrida del plan cuenta como hecha si existe al menos una carpeta con su fase e identificador, haya completado o no la vuelta.

### 3.1.3. plan.py, plan_sintonia.py y plan_piloto.py

`plan.py` genera el plan de campaña, diez sesiones en las que cada sesión corre los tres controladores con los tres perfiles en orden aleatorio, con una semilla. El plan se guarda en `configs/plan_campana.json` y no se sobrescribe. `plan_sintonia.py` genera un plan de búsqueda aleatoria con semilla dentro de los rangos de `configs/rangos_sintonia.json`, con los pesos del MPC muestreados en escala logarítmica, y escribe una configuración por corrida en `configs/sintonia/`. `plan_piloto.py` genera dos bloques, una escalera descendente del factor de uso de agarre con los tres controladores en perfil rápido y un bloque de repetibilidad en perfil nominal, y escribe las configuraciones en `configs/piloto/`.

### 3.1.4. proceso_consola.py

El script previo de la pestaña *MPC completo* solo guarda su corrida cuando recibe una interrupción de teclado. Este módulo lo lanza con una consola propia oculta y genera un Ctrl+C real sobre esa consola con un proceso auxiliar.

## 3.2. Módulo plataforma

### 3.2.1. memoria_ac.py

La clase `MemoriaAC` abre las páginas de memoria compartida de física, gráficos y datos estáticos del simulador. Intenta primero una estructura de física ampliada que añade los puntos de contacto de las llantas, la velocidad angular local y otros campos. La declaración de esos campos sigue la librería comunitaria mdjarv/assettocorsasharedmemory y el código la marca como no verificada para la versión 1.16.4 del simulador, por lo que cada lectura se comprueba antes de usarse. `construir_estado` arma un `EstadoVehiculo` con la posición, la orientación de la carrocería, la velocidad, las posiciones de los ejes, la batalla medida y banderas de validez.

### 3.2.2. trazada.py

La clase `Trazada` carga el archivo CSV con las columnas x, z y longitud acumulada, calcula la orientación de cada cuerda, la tangente en cada vértice y la curvatura suavizada. `Proyector` proyecta un punto sobre la trazada con búsqueda local alrededor de la última proyección y devuelve índice, abscisa, error lateral, orientación interpolada y curvatura. Hay un proyector por cada punto del vehículo, posición del simulador, eje trasero y eje delantero. `punto_a_distancia` entrega el punto objetivo de Pure Pursuit.

### 3.2.3. perfil.py

`SpeedProfileGenerator` calcula la velocidad límite por curvatura y por frenado. `PerfilEscalado` multiplica ese perfil por el factor del perfil elegido y entrega la velocidad objetivo con anticipación. `construir_perfil` los crea a partir de la configuración.

### 3.2.4. longitudinal.py

`PIDLongitudinalController` calcula el comando de pedal entre menos uno y uno a partir del error de velocidad. `PedalShaper` limita la tasa de cambio del pedal con constantes distintas para subir y bajar el acelerador y el freno.

### 3.2.5. actuador.py

`InterfazDireccion` aplica los límites de magnitud y tasa, iguales para los tres controladores. `entrada_progresiva` escala la dirección a baja velocidad. `SalidaVJoy` convierte el ángulo de rueda a eje de vJoy y envía dirección, acelerador y freno. `SalidaSimulada` tiene la misma interfaz sin dispositivo y se usa en las pruebas.

### 3.2.6. vuelta.py y abandono.py

`SeguidorVuelta` distingue tres fases, salida, medida y terminada, a partir de los cruces de meta. `Abandono` decide si la corrida debe cerrarse por vehículo detenido o por ruedas fuera de la pista.

### 3.2.7. regiones.py

`asignar_region` clasifica cada valor de curvatura en región baja, media o alta con los radios de la configuración.

### 3.2.8. registro.py y procedencia.py

`RegistroCorrida` acumula las filas de telemetría en memoria y al terminar escribe la carpeta de la corrida. `procedencia.py` calcula huellas SHA256 de los archivos de origen, de la trazada y de cada archivo de código del software.

### 3.2.9. juego_ac.py

Abre y prepara el simulador. `aplicar_plantilla` compara la plantilla de `configs/sesion_ac` con la carpeta de configuración del juego y, si algún archivo difiere, respalda la carpeta completa antes de escribir. `lanzar` abre `acs.exe`, `esperar_en_vivo` espera la sesión en vivo, `VentanaAC.clic` pulsa el botón del volante del menú con coordenadas escaladas al tamaño de la ventana, `probar_control` envía un freno de prueba por vJoy y lo lee en la memoria compartida, y `restaurar_respaldo` copia de vuelta el último respaldo. `preparar` ejecuta la secuencia de la Figura [[fig:preparacion]] y devuelve la información que se copia al manifiesto. El módulo registra también si Custom Shaders Patch está presente en la carpeta del juego, porque puede cambiar el clima y la temperatura de pista.

## 3.3. Módulo controladores

`base.py` define `EntradaLateral`, `SalidaControlador` y la clase `ControladorLateral`. Cada controlador implementa `calcular(entrada)` y devuelve el ángulo de rueda pedido en radianes. `crear_controlador` en `__init__.py` crea el controlador por nombre, pure_pursuit, stanley o mpc_cinematico, con sus parámetros de la configuración. Pure Pursuit y el MPC usan los errores del eje trasero y Stanley los del eje delantero. Los campos `extra1` y `extra2` de la salida guardan diagnósticos propios de cada controlador, la distancia de anticipación y el ángulo al punto objetivo en Pure Pursuit, los términos de orientación y de error lateral en Stanley, y el tiempo del solucionador en milisegundos y el valor del costo en el MPC.

## 3.4. Módulo analisis

La Tabla [[tab:analisis]] resume los guiones de análisis. Todos se ejecutan desde la línea de comandos sobre corridas ya guardadas.

: Tabla [[tab:analisis]]. Guiones de análisis.

| Archivo | Función | Salida |
|---|---|---|
| `metricas_vuelta.py` | Métricas de la vuelta medida y por región de curvatura | `metricas.json` en la carpeta de la corrida |
| `mapa_campana.py` | Mapa por perfil y región, Friedman, Wilcoxon, Holm y bootstrap | archivo del mapa en `resultados_campana/` |
| `repetibilidad_piloto.py` | Repetibilidad, umbral de mejora práctica y tiempo límite | `piloto_fase5/repetibilidad_piloto.json` |
| `seleccion_sintonia.py` | Aplica el criterio de selección de la sintonía | `sintonia_fase4/seleccion_sintonia.json` |
| `planta_longitudinal.py` | Planta longitudinal por tramos y validación en lazo cerrado | `sintonia_pid/` |
| `planta_lateral.py` | Planta lateral cinemática con ganancia y retardo, para acotar rangos de sintonía | `sintonia_fase4/planta_lateral.json` |
| `identificar_direccion.py` | Constante de la cadena de dirección con intervalo por bloques | `results/p00/auditoria/identificacion_direccion.json` |
| `regiones_curvatura.py` | Cobertura de regiones sobre la trazada | `results/p00/auditoria/` |

# 4. Modelos implementados

## 4.1. Convención de signos y estado del vehículo

El plano de movimiento es el plano x z del simulador y los ángulos se miden con atan2 de z sobre x. La orientación de la carrocería ψ se obtiene sumando al heading del simulador un desfase de π/2 rad medido. Un ángulo de rueda δ positivo hace crecer ψ y corresponde a un comando positivo de vJoy. El error lateral $e_y$ es positivo cuando el punto está a la izquierda de la trazada y el error angular es la orientación del vehículo menos la orientación de la trazada.

Las posiciones de los ejes salen del punto medio de los contactos de las llantas delanteras y traseras. La batalla medida es la distancia entre esos dos puntos. Si la batalla sale del rango de 2.0 a 3.5 m, si el centro de los ejes se aleja más de 3.0 m de la posición del simulador o si la línea entre ejes difiere de ψ en más de 10 grados, se usa un traslado de respaldo con batalla de 2.7 m y el punto del simulador a 0.43 de la batalla desde el eje trasero. Esos valores de respaldo están marcados en la configuración como no verificados.

## 4.2. Trazada, errores de seguimiento y curvatura

La orientación de la cuerda que une los vértices i e i+1 de la trazada es

$$\theta_i = \operatorname{atan2}(z_{i+1} - z_i,\ x_{i+1} - x_i) \qquad ([[eq:theta]])$$

La curvatura sin suavizar es la variación de esa orientación entre cuerdas consecutivas dividida por la longitud de la cuerda, envuelta al intervalo de menos π a π,

$$\kappa_i^{0} = \frac{\operatorname{envolver}(\theta_{i+1} - \theta_i)}{\Delta s_i} \qquad ([[eq:kappa0]])$$

y la curvatura usada es su media móvil circular con una ventana de 15 m, que con el espaciado medio de 1.54 m son 11 puntos. Para un punto del vehículo con coordenadas x y z proyectado sobre la cuerda i, con φ igual a la orientación de la cuerda, el error lateral es

$$e_y = -\sin\phi\,(x - x_i) + \cos\phi\,(z - z_i) \qquad ([[eq:ey]])$$

La orientación de la trazada en el punto proyectado se interpola entre las tangentes de los vértices de la cuerda, y el error angular es

$$e_\psi = \operatorname{envolver}(\psi - \phi_{t}) \qquad ([[eq:epsi]])$$

donde $\phi_t$ es esa orientación interpolada. La interpolación evita un error de orientación de hasta κ Δs sobre 2 que aparecería al tomar la orientación de la cuerda en todo el segmento.

## 4.3. Regiones de curvatura

Con los radios de la configuración, 500 m y 100 m, cada punto se clasifica en región baja si |κ| es menor a 1/500 m⁻¹, en región media si |κ| está entre 1/500 y menos de 1/100 m⁻¹, y en región alta si |κ| es igual o mayor a 1/100 m⁻¹.

## 4.4. Perfil de velocidad

La velocidad límite por curvatura en cada punto es

$$v_{c,i} = \min\left(\sqrt{\frac{a_{y,\max}\, g_u}{\max(|\kappa_i|,\,10^{-5})}},\ v_{\max}\right) \qquad ([[eq:vcurva]])$$

con $a_{y,\max}$ igual a 6.5 m/s², factor de uso de agarre $g_u$ igual a 0.80 y $v_{\max}$ igual a 50 m/s en la configuración base. Después se recorre la trazada hacia atrás cinco veces aplicando

$$v_i \leftarrow \min\left(v_i,\ \sqrt{v_{i+1}^2 + 2\, a_{x,\max}\, g_b\, \Delta s_i}\right) \qquad ([[eq:vfreno]])$$

con $a_{x,\max}$ igual a 10.25 m/s² y factor de frenado $g_b$ igual a 0.6. El perfil de cada nivel es el perfil base multiplicado por un factor f, 0.8 en el conservador, 0.9 en el nominal y 1.0 en el rápido. La velocidad objetivo en el índice proyectado es el mínimo del perfil escalado en una ventana hacia adelante de longitud

$$d_p = d_0 + g_p\, v \qquad ([[eq:preview]])$$

con $d_0$ igual a 20 m y $g_p$ igual a 0.35 s.

## 4.5. Lazo longitudinal

El comando del PID a partir del error de velocidad $e_v$ en km/h es

$$u_k = \operatorname{sat}_{[-1,1]}\left(k_p\, e_{v,k} + k_i\, I_k + k_d\, \frac{e_{v,k} - e_{v,k-1}}{T_s}\right), \qquad I_k = \operatorname{sat}_{[-I_{\max}, I_{\max}]}\left(I_{k-1} + e_{v,k}\, T_s\right) \qquad ([[eq:pid]])$$

Los valores congelados son $k_p$ igual a 0.0985, $k_i$ igual a 0.0758, $k_d$ igual a 0 e $I_{\max}$ igual a 13.2. Se obtuvieron con la regla analítica de Skogestad [[ref:skogestad2003]] sobre la rama de freno de una planta identificada con telemetría, con $\tau_c$ igual a 0.25 s y retardo θ igual a 0.075 s,

$$k_p = \frac{1}{a\,(\tau_c + \theta)}, \qquad k_i = \frac{k_p}{4\,(\tau_c + \theta)}, \qquad I_{\max} = \frac{1}{k_i} \qquad ([[eq:simc]])$$

donde a es la ganancia de la planta identificada. El valor positivo del comando es acelerador y el negativo es freno. El conformador de pedal limita el cambio por ciclo a $T_s$ dividido por la constante correspondiente, 0.6 s para subir el acelerador, 0.35 s para bajarlo, 0.45 s para subir el freno y 0.3 s para soltarlo.

## 4.6. Pure Pursuit

La distancia de anticipación crece con la velocidad,

$$L_d = L_0 + k_v\, v \qquad ([[eq:ld]])$$

El punto objetivo es el primer punto de la trazada, avanzando desde la proyección del eje trasero, a distancia euclidiana $L_d$ del eje trasero, interpolado dentro del segmento. Con α igual al ángulo entre el eje longitudinal del vehículo y la línea al punto objetivo, y L igual a la batalla, el ángulo pedido es

$$\delta = \arctan\left(\frac{2\, L\, \sin\alpha}{L_d}\right) \qquad ([[eq:pp]])$$

Los parámetros congelados son $L_0$ igual a 2.182824 m y $k_v$ igual a 0.475382 s.

## 4.7. Stanley

La ley de Stanley [[ref:hoffmann2007]] se implementa sin anticipación de curvatura, con los errores del eje delantero. En la convención de signos del software, donde los errores son vehículo menos trazada, la ley queda

$$\delta = -e_{\psi,f} - \arctan\left(\frac{k\, e_{y,f}}{v + \varepsilon}\right) \qquad ([[eq:stanley]])$$

con k igual a 2.037435 y ε igual a 1.0 m/s.

## 4.8. MPC con modelo bicicleta cinemático

El modelo de predicción es lineal en los errores del eje trasero, con la velocidad medida constante en el horizonte y las aproximaciones sen $e_\psi$ igual a $e_\psi$ y tan δ igual a δ, siguiendo el modelo bicicleta cinemático [[ref:kong2015]],

$$\begin{bmatrix} e_y \\ e_\psi \end{bmatrix}_{k+1} = \begin{bmatrix} 1 & T_s v \\ 0 & 1 \end{bmatrix} \begin{bmatrix} e_y \\ e_\psi \end{bmatrix}_{k} + \begin{bmatrix} 0 \\ T_s v / L \end{bmatrix} \delta_k + \begin{bmatrix} 0 \\ -T_s v\, \kappa_k \end{bmatrix} \qquad ([[eq:modelo]])$$

donde $\kappa_k$ es la curvatura de la trazada a la distancia v $T_s$ k por delante de la proyección del eje trasero. El costo sobre un horizonte de N pasos es

$$J = \sum_{k=1}^{N} \left(Q_y\, e_{y,k}^2 + Q_\psi\, e_{\psi,k}^2\right) + \sum_{k=0}^{N-1} R_\delta \left(\delta_k - \arctan(L\kappa_k)\right)^2 + \sum_{k=0}^{N-1} R_{\Delta\delta} \left(\delta_k - \delta_{k-1}\right)^2 \qquad ([[eq:costo]])$$

donde $\delta_{-1}$ es el último ángulo aplicado. El segundo término penaliza la diferencia con el ángulo cinemático de la curva en lugar de δ², para no sesgar el controlador hacia afuera en curva. Las restricciones son

$$|\delta_k| \le \delta_{\max}, \qquad |\delta_k - \delta_{k-1}| \le \dot{\delta}_{\max}\, T_s \qquad ([[eq:restr]])$$

con $\delta_{\max}$ igual a 0.403 rad y tasa máxima de 30 grados por segundo. Los valores congelados son N igual a 20, $Q_y$ igual a 1, $Q_\psi$ igual a 33.167603, $R_\delta$ igual a 8.147265 y $R_{\Delta\delta}$ igual a 3.092056. Con $T_s$ de 0.05 s el horizonte cubre 1 s.

## 4.9. Entrada progresiva y límites de dirección

El ángulo pedido por cualquier controlador se multiplica por un factor que depende de la velocidad v en km/h,

$$f_d(v) = \begin{cases} 0 & v \le 5 \\ \dfrac{v - 5}{30 - 5} & 5 < v < 30 \\ 1 & v \ge 30 \end{cases} \qquad ([[eq:rampa]])$$

con 5 y 30 km/h como velocidades de la configuración. Con factor cero la dirección queda retenida. Después se limita la magnitud a $\delta_{\max}$ y el cambio respecto al último ángulo aplicado a la tasa máxima por el tiempo transcurrido desde el último envío, acotado a dos periodos nominales para que un ciclo largo no permita un salto grande.

## 4.10. Conversión al eje de vJoy

El ángulo aplicado se normaliza con la constante de la cadena de dirección $c_d$, igual a 0.403 rad, y el signo s de la configuración,

$$n = \operatorname{sat}_{[-1,1]}\left(\frac{s\,\delta}{c_d}\right), \qquad \text{eje} = \left\lfloor \frac{n + 1}{2}\, 32766 + 1 \right\rfloor \qquad ([[eq:vjoy]])$$

El acelerador y el freno se envían en los ejes Y y rotación Z con escala de 0 a 32767.

# 5. Métodos numéricos

## 5.1. Discretización y periodo de muestreo

El periodo nominal $T_s$ es de 50 ms. El ciclo no usa un temporizador fijo. Espera a que el simulador publique un paquete gráfico nuevo, lo procesa y duerme lo que falte para completar el periodo. El periodo real de cada ciclo se registra en la columna `periodo_ms`. El modelo del MPC se discretiza con el mismo $T_s$ por integración de Euler hacia adelante, como muestra la Ecuación [[eq:modelo]].

## 5.2. Solución del problema cuadrático del MPC

La predicción se escribe como $x = S_x x_0 + S_u \delta + c$ apilando la Ecuación [[eq:modelo]] sobre el horizonte, lo que deja un problema cuadrático denso en las N variables de dirección, con matriz

$$H = 2\left(S_u^\top Q\, S_u + R_\delta I + R_{\Delta\delta} D^\top D\right) \qquad ([[eq:hess]])$$

donde D es la matriz de diferencias primeras. Las restricciones de magnitud y tasa forman una matriz constante que apila la identidad sobre D. El problema se resuelve con OSQP [[ref:stellato2020]] con estos ajustes, arranque en caliente activado, pulido desactivado, tolerancias absoluta y relativa de 10⁻⁴, máximo de 4000 iteraciones y límite de tiempo de 40 ms. El patrón de dispersión de H es fijo, de modo que en cada ciclo solo se actualizan sus valores, el vector lineal y los límites. El arranque en caliente usa la secuencia óptima anterior desplazada un paso.

## 5.3. Respaldo ante falla del solucionador

Si el estado de OSQP no es solved o la solución contiene valores no finitos, se aplica la secuencia anterior desplazada un paso, se marca `ctrl_respaldo` igual a 1 y se incrementa un contador de fallos consecutivos. El registro guarda además el estado del solucionador, el número de iteraciones y el tiempo de solución.

## 5.4. Suavizados y proyección

La curvatura se suaviza con media móvil circular. El perfil de velocidad admite un suavizado que solo puede bajar el perfil, desactivado en la configuración base con ventana de 0 m. La proyección busca el vértice más cercano en una ventana de 60 puntos alrededor de la proyección anterior y vuelve a una búsqueda global si la distancia supera 20 m.

## 5.5. Fin de vuelta, tiempo límite y abandono

La vuelta medida empieza en el primer cruce de meta y termina en el segundo. Un cruce se detecta cuando la posición normalizada del simulador salta de un valor mayor a 0.8 a uno menor a 0.2. El contador de vueltas del simulador no cuenta el primer cruce tras la salida, verificado el 15 de septiembre de 2026, y se conserva solo como comprobación. En cada cruce se guardan la diferencia entre velocidad y objetivo y el máximo error lateral de los últimos 2 s. La corrida se detiene al completar la vuelta medida, al superar 420 s, al recibir la señal de detención o por abandono automático. El abandono se arma después de superar 30 km/h, no aplica en pits y se dispara con velocidad menor a 5 km/h durante más de 8 s o con tres o más ruedas fuera durante más de 3 s.

# 6. Lazo de control en tiempo real

La Figura [[fig:ciclo]] resume el orden de operaciones de cada ciclo en `ejecutar_corrida.py`.

![Figura [[fig:ciclo]]. Orden de operaciones de un ciclo de control.](figuras/fig_ciclo_control.png){width=15.5cm}

La lectura del estado ocurre cuando cambia el identificador de paquete gráfico, y en ese instante se toma la marca de tiempo del ciclo. El tiempo de cómputo del controlador se mide solo alrededor de `calcular` y se guarda en `tc_ms`. El retardo total entre la lectura y el envío a vJoy se guarda en `retardo_ms`. El envío ocurre apenas termina el cálculo, sin esperar al final del periodo. Mientras el paquete no cambia, el ciclo duerme 2 ms y revisa la señal de detención cada 200 repeticiones, para responder al lanzador aunque el simulador esté en pausa. Cada 20 ciclos el proceso imprime una línea con tiempo, fase de la vuelta, error lateral, ángulo aplicado, velocidad y objetivo, tiempo de cómputo, posición normalizada y estado de los contactos. Esa línea es la que aparece en *Salida en vivo*.

# 7. Gestión de experimentos

## 7.1. Fases y corridas

Cada corrida pertenece a una de cinco fases, verificacion, sintonia, piloto, campana o prueba, y lleva sesión, controlador, perfil, etiqueta e identificador de plan opcionales. El identificador de la carpeta se forma con la fase, la sesión con dos dígitos, el controlador, el perfil, la fecha y la hora.

## 7.2. Planes con semilla

Los planes de campaña y de sintonía se generan una sola vez con semilla. El plan del piloto no es aleatorio, sigue un orden fijo por bloques. Los tres se guardan en JSON antes de ejecutar y no se sobrescriben. El plan de campaña contiene diez sesiones con las nueve combinaciones de controlador y perfil en orden aleatorio en cada una. La Tabla [[tab:plan]] muestra la estructura de una fila de plan tal como la presenta la interfaz.

: Tabla [[tab:plan]]. Columnas de las tablas de planes en la interfaz.

| Plan | Columnas |
|---|---|
| Campaña | id_plan, sesion, posicion, controlador, perfil, estado, intentos, carpeta |
| Sintonía | id_plan, controlador, parametros, estado, carpeta |
| Piloto | id_plan, bloque, agarre, controlador, perfil, estado, carpeta |

## 7.3. Tandas encadenadas

Las pestañas de sintonía, piloto y campaña pueden correr en tanda las corridas pendientes de su plan, en el orden del plan. La tanda reinicia la sesión del simulador antes de cada vuelta cerrando y abriendo el juego con la plantilla, y registra su avance en un archivo en `data/raw/p00/tandas`. En la campaña la tanda corre por omisión una sola sesión, y el campo *Sesiones seguidas* permite encadenar hasta diez. Cuando se encadenan varias sesiones el lanzador lo advierte y lo deja registrado, porque las sesiones quedan sin separación en el tiempo.

## 7.4. Correr fuera de orden o repetir

Cargar una corrida del plan que no es la siguiente, o repetir una ya hecha, pide confirmación. Una vuelta fallida cuenta como hecha y no se repite en silencio.

# 8. Análisis y posprocesamiento

## 8.1. Métricas por vuelta y región

`metricas_vuelta.py` usa solo los ciclos de la fase medida. La región de cada ciclo sale de la curvatura en la proyección de la posición del simulador. Para la vuelta completa y para cada región calcula, entre otras, las métricas de la Tabla [[tab:metricas]].

: Tabla [[tab:metricas]]. Métricas calculadas por `metricas_vuelta.py`.

| Clave en metricas.json | Significado |
|---|---|
| `rmse_e_y_cg_m`, `mae_e_y_cg_m`, `max_abs_e_y_cg_m` | Error lateral en la posición del simulador |
| `rmse_e_y_tras_m` y afines | Error lateral en el eje trasero |
| `rmse_e_psi_cg_rad` | Error angular |
| `rms_tasa_delta_rad_s`, `variacion_total_delta_rad`, `pico_abs_delta_rad` | Esfuerzo de dirección |
| `rmse_v_kmh` y afines | Error de velocidad |
| `tc_ms_media`, `tc_ms_p95`, `tc_ms_max`, `pct_tc_mayor_50ms` | Tiempo de cómputo |
| `pct_sat_magnitud`, `pct_sat_tasa`, `pct_respaldo_controlador` | Saturaciones y respaldo |
| `eventos_salida_pista` | Salidas con más de dos ruedas fuera |

El error cuadrático medio del error lateral sobre los n ciclos de un grupo es

$$\mathrm{RMSE}_{e_y} = \sqrt{\frac{1}{n}\sum_{j=1}^{n} e_{y,j}^2} \qquad ([[eq:rmse]])$$

y la tasa de dirección se calcula como la diferencia de ángulos aplicados entre ciclos dividida por la diferencia de tiempos medida.

## 8.2. Mapa de campaña y pruebas estadísticas

`mapa_campana.py` agrupa las vueltas de una fase por perfil y región, tres por tres, con la vuelta como unidad de análisis. Implementa la prueba de Friedman sobre los tres controladores con la sesión como bloque, la prueba de Wilcoxon pareada del MPC contra cada geométrico, la corrección de Holm en dos familias de nueve celdas y un intervalo bootstrap del 95 por ciento de la diferencia que remuestrea sesiones completas. El criterio del mapa tiene tres capas, factibilidad, decisión de mejora con el umbral leído del piloto y esfuerzo reportado con bandas de 1.5 y 3 para la razón de tasas de dirección. Ningún umbral se decide en el código, todos se leen de la configuración y de la salida del piloto.

## 8.3. Gráficas

Las figuras de un estudio se generan fuera del software con `scripts/16_p00_figuras.py` y las tablas con `scripts/17_p00_tablas.py`, que leen las salidas del análisis y escriben figuras a 600 dpi. El botón *Generar gráficas* de la pestaña *MPC completo* ejecuta `scripts/08_graficas_corrida.py` sobre la última corrida de ese script previo.

# 9. Almacenamiento y formatos

Cada corrida produce una carpeta en `data/raw/p00/corridas` con tres archivos. `telemetria.csv` tiene una fila por ciclo y 70 columnas agrupadas como muestra la Tabla [[tab:columnas]]. `perfil.csv` tiene una fila por punto de la trazada con índice, abscisa, coordenadas, orientación, curvatura suavizada, velocidad límite y velocidad del perfil en km/h. `manifiesto.json` guarda el identificador, las marcas de inicio y fin, el número de ciclos, los metadatos de la corrida, la configuración completa, la ruta y huella de la configuración, las versiones de Python y de las librerías, el estado de los archivos de origen, las huellas del código, los datos estáticos del simulador, el resumen y las notas. `metricas_vuelta.py` añade `metricas.json`.

: Tabla [[tab:columnas]]. Grupos de columnas de telemetria.csv.

| Grupo | Columnas |
|---|---|
| Tiempo y bucle | ciclo, t_s, periodo_ms, tc_ms, retardo_ms, packet_graficos, packet_fisica, packets_repetidos |
| Posición y orientación | x, y, z, psi_rad, heading_ac_rad, rumbo_velocidad_rad, deriva_rad |
| Ejes | contactos_ok, motivo_contactos, x_tras, z_tras, x_del, z_del, L_medida_m, L_usada_m, desalineacion_ejes_rad |
| Proyecciones | idx, s, e_y, e_psi y kappa en la posición, el eje trasero y el eje delantero |
| Velocidad | v_kmh, v_ref_kmh, v_perfil_kmh |
| Dirección | delta_pedido_rad, delta_aplicado_rad, factor_direccion, direccion_retenida, sat_magnitud, sat_tasa, steer_norm, sat_eje, steer_angle_ac |
| Controlador | ctrl_estado, ctrl_iteraciones, ctrl_respaldo, ctrl_extra1, ctrl_extra2 |
| Longitudinal | u_pid, u_pedal, gas_cmd, freno_cmd, gas_ac, freno_ac |
| Vehículo y vuelta | acc_g_x, acc_g_y, acc_g_z, tasa_guinada_local, ruedas_fuera, vueltas_completadas, posicion_normalizada, en_pits, fase_vuelta |

Los planes se guardan en `configs/plan_campana.json`, `configs/plan_sintonia.json` y `configs/plan_piloto.json`. Los de campaña y sintonía guardan la semilla, la fecha de generación, el método y la versión de numpy, y el del piloto guarda la fecha de generación, los parámetros y el agarre fijado. Los respaldos de la configuración del simulador se guardan en `data/raw/p00/respaldos_cfg_ac` y la información de cada preparación en `data/raw/p00/preparaciones_ac`.

# 10. Restricciones y limitaciones

* El software depende de Assetto Corsa, de Steam y de vJoy en Windows, programas que no son de los autores.
* La configuración incluida cubre una pista, Monza, y un vehículo, el Alfa Romeo Giulietta QV. La constante de dirección y los parámetros de los controladores se midieron y sintonizaron para esa combinación.
* El modelo de predicción del MPC es cinemático y lineal en los errores, sin dinámica de llantas ni transferencia de carga, y supone velocidad constante en el horizonte.
* El periodo de control depende de la frecuencia con la que el simulador publica paquetes gráficos.
* Parte de la estructura de física ampliada y el orden de las ruedas están marcados como no verificados en el código. Las lecturas se comprueban antes de usarse.
* El signo del eje de guiñada local del simulador no está verificado, por lo que la reidentificación de la constante de dirección usa el valor absoluto de la pendiente.
* El clic sobre el menú del simulador usa coordenadas de la ventana y puede fallar al reabrir el juego. En ese caso el lanzador espera hasta 120 s a que el usuario pulse el volante del menú.
* La presencia de Custom Shaders Patch puede cambiar el clima y la temperatura de pista fijados por la plantilla. El software lo registra pero no lo impide.

# 11. Despliegue

1. Instalar Steam, Assetto Corsa y vJoy, y configurar vJoy como dispositivo 1.
2. Instalar Python 3.14 y, desde la raíz del repositorio, las librerías con `python -m pip install -r P00_control_lateral/requirements.txt`.
3. Revisar en `configs/juego_ac.json` la ruta de instalación del simulador y la carpeta de configuración del juego.
4. Crear en la carpeta del juego el archivo `steam_appid.txt` con el número 244210. Sin ese archivo `acs.exe` no abre directo y Steam abre el lanzador oficial, que reescribe la carpeta de configuración.
5. Verificar que la plantilla de `configs/sesion_ac` asigna vJoy como control, con Monza, el Giulietta QV y sesión de vuelta rápida.
6. Ejecutar las pruebas automáticas, por ejemplo `python P00_control_lateral/pruebas/prueba_humo.py`.
7. Abrir el lanzador con `python P00_control_lateral/abrir_lanzador.py` desde la raíz del repositorio.

Una corrida también puede ejecutarse sin la ventana, con el simulador abierto y el vehículo en pista, con `python P00_control_lateral/ejecutar_corrida.py --controlador stanley --perfil conservador --sesion 0 --fase prueba`.

# 12. Tecnologías utilizadas

: Tabla [[tab:tec]]. Tecnologías utilizadas.

| Tecnología | Versión verificada | Uso |
|---|---|---|
| Python | 3.14.7 | Lenguaje |
| tkinter | incluida en Python | Interfaz gráfica |
| numpy | 2.5.3 | Cálculo numérico |
| scipy | 1.18.1 | Matrices dispersas y pruebas estadísticas |
| osqp | 1.1.3 | Solución del problema cuadrático del MPC |
| pyvjoy | 1.0.1 | Acceso al dispositivo vJoy |
| ctypes y mmap | incluidas en Python | Lectura de memoria compartida |
| Assetto Corsa | leída del simulador en cada corrida | Planta vehicular |
| vJoy | instalada en el equipo de referencia | Dispositivo de juego virtual |

# 13. Glosario

Batalla. Distancia entre el eje delantero y el eje trasero del vehículo.

Constante de la cadena de dirección. Ángulo de rueda que toma el vehículo con el eje de vJoy en su extremo.

Corrida. Ejecución de un controlador con un perfil durante una vuelta de salida y una vuelta medida.

Error angular. Diferencia entre la orientación del vehículo y la de la trazada en el punto proyectado.

Error lateral. Distancia con signo entre un punto del vehículo y la trazada.

Huella SHA256. Resumen criptográfico de un archivo que cambia si el archivo cambia.

Manifiesto. Archivo JSON que describe una corrida y su procedencia.

Memoria compartida. Mecanismo de Windows por el que Assetto Corsa publica el estado del vehículo.

Perfil de velocidad. Velocidad objetivo en cada punto de la trazada.

Posición normalizada. Fracción de la vuelta recorrida según el simulador, entre 0 y 1.

Región de curvatura. Clasificación de un tramo de la trazada según su radio.

Trazada. Secuencia de puntos que el vehículo debe seguir.

Vuelta medida. Tramo entre el primer y el segundo cruce de meta, sobre el que se calculan las métricas.

vJoy. Controlador de Windows que crea un dispositivo de juego virtual.

# Referencias

[[REFERENCIAS]]
