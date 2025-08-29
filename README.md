# Robofer Eyes — README

Sistema de ojos animados y menú superpuesto para un robot basado en una Orange Pi
Zero 3 y ROS 2. Incluye renderizado de ojos en escala de grises mediante OpenCV,
salida a una pantalla física ST7735 (SPI, RGB565) o simulación en PC a través de
una ventana OpenCV, y un menú overlay que se activa con botones o teclado para
navegar por modos, ajustes de Wi-Fi y apagado del sistema.

## 1) Hardware objetivo

* **MCU principal:** Orange Pi Zero 3 (SoC Allwinner H618, 3.3 V en GPIO).
* **Pantalla:** TFT ST7735 128×160, bus SPI.
  * Pines de control: DC, RST, CS (CS manual por libgpiod o CS del driver SPI).
* **Botones:** 4 sensores táctiles TTP223 (arriba/abajo/atrás/OK) o simulación por
  teclado (W-A-S-D). Cada TTP223 se alimenta con 3.3 V y entrega señal digital a
  un GPIO.

⚠️ Importante: niveles lógicos 3.3 V. No usar 5 V en GPIO.

## 2) Software / Entorno

* ROS 2 Humble (probado). Compatible con Foxy con cambios menores de
  dependencias.
* C++17.
* Librerías:
  * OpenCV (render 8UC1 y drawing).
  * libgpiod (GPIO desde espacio de usuario).
  * spidev (acceso a SPI desde Linux userspace).
* **Build system:** colcon + ament.
* **Sistemas probados:**
  * Orange Pi Zero 3 con Ubuntu/Armbian (Server) + ROS 2 Humble.
  * PC (Ubuntu) para simulación (OpenCV + teclado).

## 3) Arquitectura de paquetes y ficheros

```
robofer/
├─ include/robofer/
│  ├─ Eyes.hpp            # lógica de ojos
│  ├─ Display.hpp         # interfaz de display + factory
│  └─ UiMenu.hpp         # controlador del menú (overlay)
├─ src/
│  ├─ Eyes.cpp            # implementación de ojos
│  ├─ Display.cpp         # DisplaySim (OpenCV) + DisplayST7735 (SPI/gpiod)
│  ├─ UiMenu.cpp         # estructura de menú + dibujo y navegación
│  ├─ EyesUnifiedNode.cpp   # nodo único: ojos + menú + display
│  ├─ ButtonsNode.cpp             # nodo de botones físicos (TTP223 → /ui/button)
│  └─ KeyboardButtonsNode.cpp    # nodo teclado (WASD → /ui/button, simulación)
└─ launch/
   └─ eyes_system.launch.py   # launch unificado (sim | real)
```

### Componentes principales

* **RoboEyes (Eyes.hpp/.cpp):** genera un `cv::Mat` 8UC1 con los ojos
  (parpadeo, estados, animaciones).
* **Display (Display.hpp/.cpp):**
  * `DisplaySim`: ventana OpenCV (escalado).
  * `DisplayST7735`: panel físico (SPI, RGB565, centrado del frame).
* **Menú (UiMenu.hpp/.cpp):**
  * Árbol raíz: Modos, Wi-Fi, Apagar.
  * Modos: Angry, Sad(Frown), Happy.
  * Timeout de visibilidad: 5 s desde la última pulsación.
  * Dibujo como overlay sobre el frame de ojos.

### Nodos

* **eyes_unified_node:** fusiona ojos + menú + display; se suscribe a `/ui/button`.
* **buttons_node:** escucha 4 GPIO (TTP223) y publica en `/ui/button` (0–3).
* **keyboard_buttons_node:** en simulación, publica en `/ui/button` con W-A-S-D.

## 4) Interfaz ROS

| Nodo                  | Publica    | Tipo             | Descripción                                    |
|-----------------------|-----------|------------------|------------------------------------------------|
| `buttons_node`        | `/ui/button` | `std_msgs/Int32` | Eventos: 0=UP, 1=DOWN, 2=BACK, 3=OK            |
| `keyboard_buttons_node` | `/ui/button` | `std_msgs/Int32` | Igual que arriba (desde teclado)               |
| `eyes_unified_node`   | —         | —                | Renderiza y envía a display                    |

En versiones anteriores se usaba `/eyes/mood_id`. Ahora el menú controla el
estado internamente a través de `/ui/button`.

### Parámetros clave

**eyes_unified_node**

* `backend`: `"sim"` o `"st7735"`.
* `eyes_width` / `eyes_height`: tamaño base de los ojos (por defecto 160×128).
* `fps`: frames por segundo (p. ej. 30 o 60).
* `menu_timeout_ms`: overlay activo tras última tecla (por defecto 5000).

**DisplayST7735** (cuando `backend:=st7735`)

* `spi_device` (por defecto `/dev/spidev1.0`)
* `spi_hz` (`24000000`)
* `use_manual_cs` (`true`)
* `gpiochip_c` (`gpiochip0`)
* `dc_offset=75`, `rst_offset=78`, `cs_offset=233` (ejemplo OPI Zero 3)
* `spi_chunk=2048`
* `madctl=0xA0`, `invert=false`, `self_test=true`

**buttons_node**

* `gpiochip`: `gpiochip0`
* `rising_on_press`: `true` (TTP223 normalmente “alto al tocar”)
* `debounce_ms`: 120–200 (por defecto 150)
* `btn{1..4}_offset`: offsets GPIO; -1 deshabilita.
* `btn{1..4}_code`: mapeo a 0=UP, 1=DOWN, 2=BACK, 3=OK (por defecto 0..3)

**keyboard_buttons_node** (simulación)

* `key_up/down/back/ok`: teclas (por defecto w/s/a/d)
* `repeat_ms`: 0 sin autorepetición (configurable)

## 5) GPIO offsets (Allwinner H618)

Calculados por banco:

PA0..31 = 0..31, PB = 32..63, PC = 64..95, PD = 96..127,
PE = 128..159, PF = 160..191, PG = 192..223, PH = 224..255.

Ejemplos utilizados:

* `PC11` → 64 + 11 = 75 (DC)
* `PC14` → 78 (RST)
* `PH9` → 224 + 9 = 233 (CS manual)

Sugerencias para botones (libres típicamente):

* `PC2=66`, `PC3=67`, `PC7=71`, `PC8=72` (4 botones).
* Alternativa: `PG8=200`, `PG10=202`, `PH7=231`, `PH10=234`.

Comandos útiles para comprobar disponibilidad:

```bash
gpioinfo gpiochip0
gpioget gpiochip0 66   # leer pin puntual
```

## 6) Instalación (sistema)

En Orange Pi / Ubuntu:

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
  python3-colcon-common-extensions \
  ros-humble-desktop ros-humble-ros-base \
  libopencv-dev libgpiod-dev
```

Ajusta `ros-humble-*` por tu distro/ROS. Sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Clona y compila:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <tu_repo> robofer
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 7) Ejecución con launch unificado

### Simulación (OpenCV + teclado WASD)

```bash
ros2 launch robofer eyes_system.launch.py sim:=true fps:=60
```

Ventana OpenCV con los ojos; el menú es visible al pulsar `w`, `a`, `s`, `d`.

### Simulación completa (ojos + máquina de estados)

Lanza ojos y máquina de estados en modo simulado. El nodo de teclado se
ejecuta aparte.

```bash
ros2 launch robofer robofer_sim.launch.py
```

En otra terminal:

```bash
ros2 run robofer keyboard_buttons_node
```

### Hardware real (ST7735 + TTP223)

```bash
ros2 launch robofer eyes_system.launch.py sim:=false \
  spi_device:=/dev/spidev1.0 spi_hz:=24000000 spi_chunk:=2048 \
  use_manual_cs:=true gpiochip_c:=gpiochip0 \
  dc_offset:=75 rst_offset:=78 cs_offset:=233 \
  madctl:=160 invert:=false self_test:=true \
  btn1_offset:=66 btn2_offset:=67 btn3_offset:=71 btn4_offset:=72
```

Estos comandos asumen que el workspace ya fue compilado y que se ha
`sourceado` el `setup.bash` correspondiente.

## 8) Flujo de interacción (menú)

El overlay del menú aparece cuando se pulsa un botón y permanece visible 5 s
desde la última pulsación.

**Navegación:**

* **UP/DOWN:** moverse entre items.
* **BACK:** subir un nivel.
* **OK:** entrar a submenú o ejecutar acción.

**Árbol inicial:**

* Modos
  * Angry → cambia humor a ANGRY
  * Sad → cambia a FROWN ("triste/ceño")
  * Happy → cambia a HAPPY
* Wi-Fi → vacío por ahora (placeholder).
* Apagar → ejecuta `sudo poweroff`.

Para `sudo poweroff` sin password:

```bash
sudo visudo
# Añade (cambia <usuario>):
<usuario> ALL=(ALL) NOPASSWD: /sbin/poweroff, /usr/sbin/poweroff
```

## 9) Troubleshooting

* **La pantalla no enciende / se ve en blanco:**
  * Verifica `spi_device`, `spi_hz` (24 MHz funciona bien), `madctl`, `invert` y
    `self_test`.
  * Comprueba GPIOs de DC/RST/CS y que `use_manual_cs:=true` si usas CS por GPIO.
  * Permisos SPI: añade al grupo `spi` o ejecuta con permisos.
* **Botones no responden:**
  * Comprueba offsets con `gpioinfo/gpioget`.
  * Ajusta `rising_on_press` (si tu TTP223 se configuró como low-active).
  * Sube `debounce_ms` a 200–300 ms si hay falsos positivos.
* **Simulación no abre ventana:**
  * Falta backend gráfico (en servers). Instala `libgtk2.0-dev` o `libgtk-3-dev`
    según OpenCV.
* **No apaga:**
  * Revisa `sudoers` y la ruta de `poweroff` (`which poweroff`).

## 10) Roadmap

* Confirmación "¿Seguro apagar?" en el menú.
* Submenú Wi-Fi: escaneo, selección de red, credenciales.
* Perfiles de mood más ricos (Idle/Curious configurables desde menú).
* Persistencia de ajustes (YAML/param server).
* Telemetría (fps, tiempos de SPI) en un panel secundario.

## 11) Licencia

El código de RoboEyes original se basa en el proyecto de FluxGarage (GPLv3). Esta
implementación C++/ROS respeta la licencia original. El resto del código de
integración (nodos/menú/display) sigue la licencia que decidas para este repo
(indícala en `LICENSE`).

## 12) Comandos útiles (chuleta)

```bash
# Build limpio
cd ~/ros2_ws && rm -rf build install log && colcon build --symlink-install

# Sourcing
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Probar botones por teclado (solo sim)
ros2 run robofer keyboard_buttons_node

# Inyectar eventos a mano
ros2 topic pub /ui/button std_msgs/Int32 "{data: 0}"   # UP
ros2 topic pub /ui/button std_msgs/Int32 "{data: 3}"   # OK
```
