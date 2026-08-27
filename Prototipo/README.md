# README — Proyecto Volante Force Feedback (FFBeast)

Este documento resume el avance del proyecto de construcción de un volante de Force Feedback (FFB) casero basado en FFBeast, desde el semestre pasado hasta el estado actual.

## 1. Origen del proyecto

El proyecto inició el semestre pasado con el objetivo de construir un volante de Force Feedback funcional para simuladores de carreras, usando componentes accesibles y de bajo costo.

## 2. Componentes utilizados

- **Motor:** motores extraídos de una hoverboard reciclada.
- **Driver:** MKS ODrive Mini, encargado de controlar el motor y generar la fuerza de retroalimentación.
- **Encoder:** encoder rotativo de 360 pulsos por revolución (PPR).
- **Relación de engranajes:** 2:1, implementada con el fin de aumentar la resolución efectiva del encoder.
- **Fuente de alimentación:** fuente de 480 W, 24 V, 20 A.
- **Material estructural (prototipo inicial):** ABS, elegido por ser el material más apto disponible en ese momento considerando rigidez y resistencia térmica.

## 3. Diseño y fabricación del primer prototipo

- Se diseñó y fabricó el primer prototipo del volante en ABS.
- Se implementó la relación de engranajes 2:1 entre el motor/encoder y el eje del volante.
- El backlash (holgura) entre los engranajes resultó muy mínimo, prácticamente despreciable, lo cual fue un resultado positivo para la precisión del sistema.

## 4. Configuración de firmware y software

- Se realizó la configuración del firmware siguiendo el proceso de programación vía STM32CubeProgrammer (ver README de firmware para el detalle del procedimiento).
- Se ajustaron los parámetros del volante mediante la app de setup de FFBeast (perfiles de efectos, periferia, etc.).

## 5. Pruebas realizadas

- Se probó el volante en el simulador **Assetto Corsa** como banco de pruebas.
- El prototipo funcionó exitosamente, validando el concepto general del sistema (motor + driver + encoder + engranajes + firmware).

## 6. Hallazgos y aspectos a mejorar

- **Rigidez estructural:** el primer prototipo en ABS no era mala en cuanto a rigidez, pero se identificó como un punto de mejora para siguientes versiones.
- **Ubicación de la fuente de alimentación:** se descartó la idea de integrar la fuente dentro de la carcasa del volante. Se decidió dejarla externa, de forma similar a los cargadores de laptop, para lograr un diseño más modular.

## 7. Cálculos de diseño de engranajes

Para validar la resistencia de los engranajes ante la selección de material, se realizó un análisis de esfuerzo a flexión en los dientes utilizando la **Ecuación de Lewis**. El script de MATLAB con estos cálculos está disponible en `calculos_engranajes_volante.m`.

### 7.1 Datos del motor (hoverboard)

| Parámetro | Valor |
|-----------|-------|
| Torque | 10 Nm |
| Potencia | 250 W (Nm/s) |
| Velocidad | 15 km/h (4.1667 m/s) |

### 7.2 Geometría de los engranajes

| Parámetro | Valor |
|-----------|-------|
| Diámetro engranaje conducido (dE) | 66 mm |
| Dientes engranaje conducido (zE) | 44 |
| Diámetro piñón (dP) | 33 mm |
| Dientes piñón (zP) | 22 |
| Módulo (m) | 1.5 |
| Ancho de cara (b) | 10 mm |
| Relación de transmisión | 2:1 |
| Distancia entre centros (dc) | ≈ 50 mm |

### 7.3 Factores de diseño

- **Fuerza tangencial:** Ft = Potencia / Velocidad = 250 / 4.1667 = **59.99952 N**
- **Factor de servicio (Cs):** 1.8 (uso de 8–10 h/día, choque pesado)
- **Factor de velocidad (Cv):** Cv = 4.58 / (4.58 + V) = **0.523626**
- **Fuerza máxima de diseño:** Fmax = Cs·Ft / Cv = (1.8 × 59.99952) / 0.523626 = **206.252432 N**
- **Factor de forma de Lewis (Y):** evaluado sobre el piñón (zP = 22 dientes):
  Y = π(0.154 − 0.912/22) = **0.353571973194924**

### 7.4 Resistencia a la flexión de los materiales candidatos

| Material | σd (MPa / N/mm²) |
|----------|-------------------|
| Policarbonato (PC) | 111 |
| ABS | 70.5 |
| PLA | 103 |

### 7.5 Capacidad de carga del diente (Fbeam = b·m·σd·Y)

| Material | Fbeam (N) | ¿Cumple Fbeam ≥ Fmax (206.25 N)? |
|----------|-----------|-----------------------------------|
| Policarbonato (PC) | 588.69736 | ✅ Sí |
| ABS | 373.90236 | ✅ Sí |
| PLA | 546.26869 | ✅ Sí |

**Conclusión:** los tres materiales analizados cumplen el criterio estructural mínimo (Fbeam ≥ Fmax) para los engranajes del volante. El **policarbonato** ofrece el mayor margen de seguridad, seguido del **PLA** y luego el **ABS**, lo cual respalda la decisión de migrar hacia policarbonato en el nuevo prototipo.

### 7.6 Comparación integral de materiales (más allá de la resistencia a flexión)

El cálculo de Fbeam solo evalúa resistencia a flexión, pero en un volante FFB los engranajes también están sometidos a **cargas cíclicas de impacto** (picos de torque del motor) y a **calor cercano al motor y al driver**. Por eso se comparó también resistencia al impacto, resistencia térmica, rigidez, facilidad de fabricación y costo.

| Factor | Policarbonato (PC) | ABS | PLA |
|--------|---------------------|-----|-----|
| Resistencia a flexión (σd) | 111 MPa | 70.5 MPa | 103 MPa |
| Fbeam calculado | 588.7 N | 373.9 N | 546.3 N |
| Resistencia al impacto | Muy alta (tenaz, no quebradizo) | Media | Baja (quebradizo) |
| Resistencia térmica (HDT) | ~130–140 °C | ~95–100 °C | ~55–60 °C (muy baja) |
| Rigidez (módulo de flexión) | Media-alta (~2300 MPa) | Media (~2100–2300 MPa) | Alta pero frágil (~3000–4000 MPa) |
| Facilidad de impresión/mecanizado | Difícil (altas temperaturas, cerramiento) | Media (propenso a warping) | Muy fácil |
| Costo relativo | Alto | Medio | Bajo |

**Conclusión final — Policarbonato (PC) es el mejor material para esta aplicación:**

- Aunque el PLA tiene un σd alto y un Fbeam calculado mayor que el ABS, es **frágil**: ante impactos repetidos o picos de torque tiende a fisurarse en vez de deformarse, y su **baja resistencia térmica (~55–60 °C)** lo hace propenso a reblandecerse y deformarse (creep) por el calor del motor, degradando con el tiempo el backlash ya logrado.
- El ABS es un punto intermedio razonable (por eso se usó en el primer prototipo), pero tiene el σd más bajo de los tres, dándole el menor margen de seguridad estructural.
- El PC combina alta resistencia al impacto, buena resistencia térmica y el Fbeam más alto de los tres, con el mayor margen de seguridad frente a Fmax. Su única desventaja real es que es más difícil de imprimir y más costoso, pero para un componente estructural sometido a esfuerzos cíclicos cerca de una fuente de calor, esa dificultad de fabricación se justifica frente al riesgo de falla por fatiga o fluencia térmica.

## 8. Estado actual del proyecto

- Actualmente el equipo se encuentra **rediseñando el volante** para una nueva versión del prototipo.
- Se está explorando el uso de **policarbonato** como material estructural, por ser más resistente que el ABS, buscando mejorar la rigidez general del sistema.

## 9. Resumen de avances (línea de tiempo)

| Etapa | Descripción | Estado |
|-------|-------------|--------|
| Selección de componentes | Motor de hoverboard, MKS ODrive Mini, encoder 360 PPR, fuente 480W/24V/20A | ✅ Completado |
| Relación de engranajes 2:1 | Implementada para aumentar resolución del encoder | ✅ Completado |
| Primer prototipo (ABS) | Diseñado y fabricado | ✅ Completado |
| Configuración de firmware | Programación y setup vía FFBeast | ✅ Completado |
| Pruebas en simulador | Validado en Assetto Corsa | ✅ Completado |
| Cálculo de resistencia de engranajes (Lewis) | PC, ABS y PLA cumplen el criterio Fbeam ≥ Fmax | ✅ Completado |
| Mejora de rigidez estructural | Identificada como pendiente | 🔄 En progreso |
| Fuente de alimentación externa | Rediseño modular tipo cargador de laptop | 🔄 En progreso |
| Nuevo prototipo en policarbonato | Exploración de material más resistente | 🔄 En progreso |

---

**Próximos pasos sugeridos:**
- Finalizar el diseño del nuevo prototipo en policarbonato.
- Validar mejoras en rigidez estructural respecto al prototipo en ABS.
- Definir el diseño final del gabinete/carcasa para la fuente externa.
