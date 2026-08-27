# README — Cómo actualizar el firmware del volante (FFBeast)

Esta guía explica el proceso para programar el firmware del volante FFBeast usando STM32CubeProgrammer y luego configurarlo con la app de setup.

## Requisitos previos

- **STM32CubeProgrammer** instalado (descárgalo desde la página oficial de STMicroelectronics).
- La carpeta de firmware descomprimida, por ejemplo `ffbeast-wheel-RC.24.1.4.Full`, que contiene dos subcarpetas:
  - `ffbeast-wheel-hex` → contiene el archivo `.hex` a programar.
  - `ffbeast-wheel-ui` → contiene la app `ffbeast-wheel-setup-RC.24.1.4` para configurar el volante.
- Cable USB para conectar la placa del volante (MKS ODrive) a la PC.

## Paso 1: Instalar STM32CubeProgrammer

Descarga e instala STM32CubeProgrammer desde el sitio de ST. Es la herramienta que se usa para grabar el archivo `.hex` en el microcontrolador de la placa.

## Paso 2: Entrar en modo bootloader (DFU) de la placa MKS ODrive

1. Desconecta el volante de la PC (si estaba conectado).
2. Mantén presionado el botón de **BOOT** de la placa MKS ODrive.
3. Sin soltar el botón, conecta el cable USB a la PC (o presiona también el botón de reset si la placa lo requiere).
4. Suelta el botón una vez conectado. La placa debería quedar en modo de programación (DFU/bootloader), lista para recibir el firmware.

> Nota: la ubicación exacta del botón de BOOT puede variar según la revisión de la placa. Verifica en la serigrafía de tu MKS ODrive cuál es el botón correcto.

## Paso 3: Cargar el archivo .hex

1. Abre **STM32CubeProgrammer**.
2. Selecciona el modo de conexión correspondiente (USB/DFU) y presiona **Connect** para que detecte la placa en modo bootloader.
3. Ve a la sección de **Erasing & Programming**.
4. Haz clic en **Browse** y selecciona el archivo `.hex` ubicado en:
   ```
   ffbeast-wheel-RC.24.1.4.Full\ffbeast-wheel-hex\
   ```
5. Presiona **Start Programming** y espera a que el proceso termine sin errores.

## Paso 4: Reiniciar el volante en modo normal

1. Desconecta el cable USB.
2. Vuelve a conectar el volante normalmente (sin mantener presionado el botón de BOOT).
3. La placa ahora debería arrancar con el nuevo firmware cargado.

## Paso 5: Configurar el volante con la app de setup

1. Ve a la carpeta:
   ```
   ffbeast-wheel-RC.24.1.4.Full\ffbeast-wheel-ui\
   ```
2. Ejecuta la aplicación **ffbeast-wheel-setup-RC.24.1.4**.
3. Desde ahí podrás configurar todos los aspectos del volante: Force Feedback, perfiles de efectos (`wheel_effects_profiles`), periferia (`wheel_periphery_profiles`), botones, encoder, etc.
4. Guarda la configuración una vez ajustados los parámetros deseados.

## Resumen rápido

| Paso | Acción |
|------|--------|
| 1 | Instalar STM32CubeProgrammer |
| 2 | Entrar en modo BOOT en la placa MKS ODrive |
| 3 | Cargar el `.hex` desde `ffbeast-wheel-hex` |
| 4 | Reiniciar el volante en modo normal |
| 5 | Configurar con `ffbeast-wheel-setup` desde `ffbeast-wheel-ui` |

---

**Notas adicionales:**
- Si STM32CubeProgrammer no detecta la placa, revisa los drivers USB (DFU/ST-Link) y que el modo BOOT se haya activado correctamente.
- Si la app de setup no abre o no reconoce el volante, verifica que el firmware se haya grabado sin errores y que el volante esté conectado en modo normal (no en bootloader).
