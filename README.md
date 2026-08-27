# Volante direct drive con Force Feedback

Proyecto de desarrollo de un **volante direct-drive con Force Feedback (FFB)** de bajo costo, junto con la implementación de un **controlador MPC (Model Predictive Control)** orientado a dirección autónoma, usando Assetto Corsa como entorno de simulación y pruebas.

## Equipo

- **Integrantes:** Juan Camilo Díaz López y Jesus Alberto Lastra Robles
- **Profesor:** Francisco Javier Burgos Flórez
- **Carrera:** Ingeniería Mecatrónica
- **Institución:** Universidad Nacional de Colombia, Sede de La Paz

## Descripción del proyecto

El proyecto combina dos frentes de trabajo:

1. **Hardware del volante (Force Feedback):** construcción de un volante direct-drive reutilizando motores de hoverboard, controlados mediante un MKS ODrive Mini, con retroalimentación de fuerza (FFB) validada en el simulador Assetto Corsa.
2. **Control MPC para dirección autónoma:** diseño e implementación de un controlador predictivo (Model Predictive Control) para dirección autónoma, con modelado/tuning en Matlab e implementación en tiempo real mediante Python, interactuando con la telemetría de Assetto Corsa.

> **Nota sobre el alcance del MPC:** a diferencia de la mayoría de trabajos de control predictivo aplicados a conducción autónoma, que modelan la dinámica completa del vehículo (chasis, neumáticos, tracción, etc.), en este proyecto la planta modelada para el MPC es **únicamente el volante** (el sistema direct-drive con force feedback). Esto se debe a que nuestro sistema físico es el volante en sí, no el vehículo completo, por lo que el alcance del controlador se limita a la **dirección autónoma** (control del ángulo del volante), y no a la dinámica lateral/longitudinal del auto.

## Estructura del repositorio

```
Volante-direct-drive-con-Force-Feedback/
├── .gitignore                              Archivos que Git debe ignorar (metadata de Windows/macOS, temporales, etc.)
├── README.md                               Este archivo
├── Firmware/
│   ├── ffbeast-wheel-RC.24.1.4.Full/      Firmware comprimido (contiene ffbeast-wheel-hex y ffbeast-wheel-ui)
│   └── README                              Instrucciones de instalación y configuración del firmware
├── Model Predictive Control/
│   ├── AC Codes/                           Scripts para leer/interactuar con Assetto Corsa (telemetría)
│   │   ├── mpc_localization                Localización del carro sobre la trazada (Frenet)
│   │   ├── parse_fast_lane                 Extracción/procesado de la trazada (fast lane) de la pista
│   │   └── README
│   ├── Matlab/                             Modelado, diseño y validación del controlador MPC
│   │   ├── Modelo
│   │   ├── MPC View 2D
│   │   ├── MPC View 3D
│   │   ├── MPCDesignerSessionPython
│   │   ├── Validacion
│   │   └── README
│   └── Python/                             Implementación del MPC en tiempo real / interfaz con el simulador
│       ├── mpc_monza_FINAL                 Script principal: MPC real corriendo en Monza
│       ├── monza_fast_lane                 Trazada de referencia de Monza (entrada del MPC)
│       ├── mpc_session_*                   Logs de sesiones de manejo (salida del MPC)
│       └── README
└── Prototipo/
    ├── calculos_engranajes_volante         Script de MATLAB con los cálculos de resistencia de engranajes
    ├── Imagenes/                            Registro fotográfico del prototipo
    │   ├── Desgaste engranajes ABS.jpg     Desgaste (manchas blancas) por golpeteo de dientes del engranaje-piñón
    │   ├── Desgaste engranajes ABS 2.jpg   Segundo ángulo del desgaste del engranaje-piñón
    │   ├── Estructura Volante.jpg          Vista interna: encoder, MKS ODrive, motor y rejillas de ventilación
    │   ├── Prototipo completo.jpg          Estructura completa con volante provisional para pruebas
    │   └── README                          Descripción detallada de cada fotografía
    └── README                              Avance del diseño físico del volante
```

## Documentación

- **Firmware** — Instalación de STM32CubeProgrammer, modo bootloader del MKS ODrive y configuración con la app de setup de FFBeast.
- **AC Codes** — Scripts de interfaz con Assetto Corsa: localización del carro sobre la trazada y procesado de la fast lane.
- **Matlab (MPC Simulado)** — Modelado, diseño y validación del controlador MPC. La planta modelada corresponde al volante (sistema direct-drive), no al vehículo completo, ya que el alcance del proyecto es la dirección autónoma y no la dinámica integral del auto.
- **Python (MPC en tiempo real)** — Implementación del MPC corriendo en tiempo real sobre Monza, control del volante (simulado o físico vía FFBeast) e interfaz con Assetto Corsa.
- **Prototipo** — Historial de diseño y fabricación del volante: componentes, relación de engranajes, cálculos de resistencia (Ecuación de Lewis), comparación de materiales (PC, ABS, PLA) y registro fotográfico del desarrollo (ver `Prototipo/Imagenes`).

## Archivo .gitignore

El repositorio incluye un `.gitignore` en la raíz para evitar que se suban archivos que no aportan al proyecto:

- **Metadata de Windows/macOS:** `desktop.ini`, `Thumbs.db`, `ehthumbs.db`, `.DS_Store` — archivos que el propio sistema operativo genera en cada carpeta para guardar preferencias de visualización, y que no tienen relación con el código o los diseños.
- **Archivos temporales de edición:** `*.tmp`, `*.bak`, `*~`.
- **Entornos virtuales y cachés de Python:** `venv/`, `env/`, `__pycache__/`, `*.pyc` — en caso de que alguien cree un entorno virtual dentro del repo al trabajar en `Model Predictive Control/Python`.
- **Archivos autogenerados de MATLAB:** `*.asv` (copias de autoguardado) y `slprj/` (carpeta de compilación de Simulink), generados al trabajar en `Model Predictive Control/Matlab`.

Si ya subiste alguno de estos archivos antes de agregar el `.gitignore` (por ejemplo, un `desktop.ini` dentro de `Firmware`), el `.gitignore` no lo elimina automáticamente: hay que borrarlo manualmente del repositorio una sola vez (desde GitHub o con `git rm --cached <archivo>`), y a partir de ahí Git dejará de rastrearlo.

## Estado actual

- Primer prototipo del volante (ABS) fabricado y validado en Assetto Corsa. (Completado)
- Firmware configurado y funcionando sobre FFBeast + MKS ODrive Mini. (Completado)
- Cálculo estructural de engranajes completado; policarbonato seleccionado como material para el nuevo prototipo. (Completado)
- Rediseño del volante en policarbonato. (En curso)
- Desarrollo del controlador MPC para dirección autónoma. (En curso)

## Próximos pasos

- Finalizar el nuevo prototipo del volante en policarbonato.
- Integrar el controlador MPC con el hardware del volante para pruebas de dirección autónoma.
- Documentar resultados de las pruebas de MPC en Assetto Corsa.
