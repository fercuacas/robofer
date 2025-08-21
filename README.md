# Robofer

Este paquete proporciona nodos y archivos *launch* para manejar el sistema de ojos.
A continuación se muestran ejemplos de uso tanto en simulación como con el
hardware real.

## Simulación

1. Ejecuta el sistema de ojos en modo simulación:
   ```bash
   ros2 launch robofer eyes_system.launch.py sim:=true fps:=60
   ```
2. En otra terminal, carga el workspace y lanza el nodo de teclado:
   ```bash
   source ~/Documents/fer/install/setup.bash
   ros2 run robofer keyboard_buttons_node
   ```

## Hardware real

Para correr el sistema de ojos sobre el hardware ST7735 ejecuta:

```bash
ros2 launch robofer eyes_system.launch.py sim:=false \
  spi_device:=/dev/spidev1.0 spi_hz:=24000000 spi_chunk:=2048 \
  use_manual_cs:=true gpiochip_c:=gpiochip0 \
  dc_offset:=75 rst_offset:=78 cs_offset:=233 \
  madctl:=0 invert:=false self_test:=true \
  btn1_offset:=66 btn2_offset:=67 btn3_offset:=71 btn4_offset:=72
```

Estos comandos asumen que el workspace ya fue compilado y que has
sourceado el `setup.bash` correspondiente.
