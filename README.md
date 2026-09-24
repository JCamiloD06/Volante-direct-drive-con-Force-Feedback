# Volante direct drive con Force Feedback

Proyecto de desarrollo de un volante direct drive con Force Feedback (FFB) de bajo costo, junto con controladores de dirección autónoma probados en Assetto Corsa como entorno de simulación.

## Equipo

Integrantes: Juan Camilo Díaz López y Jesus Alberto Lastra Robles
Profesor: Francisco Javier Burgos Flórez
Carrera: Ingeniería Mecatrónica
Institución: Universidad Nacional de Colombia, Sede de La Paz

## Descripción del proyecto

El proyecto tiene tres frentes de trabajo.

1. Hardware del volante. Volante direct drive construido con motores de hoverboard, controlados con un MKS ODrive Mini y firmware FFBeast, con Force Feedback validado en Assetto Corsa. Carpetas `Prototipo/` y `Firmware/`.
2. MPC sobre la planta del volante. Controlador predictivo cuya planta modelada es el volante, diseñado en Matlab y ejecutado en tiempo real en Python con la telemetría de Assetto Corsa, más un control longitudinal independiente de acelerador y freno. Carpeta `Model Predictive Control/`.
3. Plataforma de comparación de controladores laterales. Software que ejecuta Pure Pursuit, Stanley y un MPC con modelo bicicleta cinemático del vehículo bajo las mismas condiciones, con lanzador gráfico, planes aleatorizados con semilla, registro trazable de corridas y análisis por región de curvatura. Carpeta `P00_control_lateral/`.

Los frentes 2 y 3 tienen alcances distintos. En el frente 2 la planta es el volante. En el frente 3 el modelo del MPC es el vehículo, y el objetivo es comparar leyes de control lateral con la misma plataforma.

## Estructura del repositorio

```
Volante-direct-drive-con-Force-Feedback/
├── README.md
├── LICENSE
├── .gitignore
├── Firmware/                         Frente 1, firmware FFBeast (terceros)
├── Prototipo/                        Frente 1, diseño físico y fotografías
├── Model Predictive Control/         Frente 2
│   ├── AC Codes/
│   ├── Matlab/
│   └── Python/
│       ├── mpc_monza_Lateral.py
│       ├── mpc_monza_Completo.py
│       ├── mpc_monza_Completo_barrido.py   Nuevo, lo ejecuta la pestaña MPC completo del frente 3
│       ├── monza_fast_lane.csv             Trazada compartida por los frentes 2 y 3
│       └── ...
├── P00_control_lateral/              Frente 3, nuevo
│   ├── abrir_lanzador.py             Punto de entrada
│   ├── ejecutar_corrida.py
│   ├── lanzador/                     Interfaz gráfica
│   ├── plataforma/                   Simulador, trazada, perfil, vJoy, registro
│   ├── controladores/                Pure Pursuit, Stanley, MPC cinemático
│   ├── analisis/                     Métricas y mapa por velocidad y curvatura
│   ├── configs/                      Parámetros y plantilla de sesión de AC
│   ├── pruebas/                      Pruebas automáticas sin simulador
│   ├── documentacion/                Descripción, manual técnico y manual de usuario
│   ├── requirements.txt
│   └── README.md
└── scripts/                          Frente 3, nuevo
    ├── 08_graficas_corrida.py
    ├── 16_p00_figuras.py
    └── 17_p00_tablas.py
```

## Documentación

* Firmware. Instalación de STM32CubeProgrammer, modo bootloader del MKS ODrive y configuración con la aplicación de FFBeast.
* AC Codes. Interfaz con Assetto Corsa, localización del carro sobre la trazada y procesado de la fast lane.
* Matlab. Modelado, diseño y validación del MPC sobre la planta del volante.
* Python. MPC en tiempo real sobre Monza y control longitudinal.
* P00_control_lateral. Instalación, uso del lanzador, convención de signos y pruebas, en su `README.md` y en `documentacion/`.

## Uso rápido del frente 3

Desde la raíz del repositorio.

```
pip install -r P00_control_lateral/requirements.txt
python P00_control_lateral/abrir_lanzador.py
```

Requiere Windows, Assetto Corsa con Monza y el Alfa Romeo Giulietta QV, y vJoy. Las corridas se guardan en `data/raw/p00/corridas`, que no se versiona.

## Licencia

Ver `LICENSE`.
