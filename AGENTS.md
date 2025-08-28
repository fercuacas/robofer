# AGENTS.md — Build & Test fijos (ROBOFER)

## Entorno (siempre antes)
```bash
set -euo pipefail

# 1) Carga ROS 2 (Jazzy)
source /opt/ros/jazzy/setup.bash

# 2) Quita pyenv del PATH para que 'python3' sea el del sistema
export PATH="$(printf "%s" "$PATH" | tr ':' '\n' | grep -v -E '\.pyenv/shims' | paste -sd: -)"
hash -r

# 3) (debug opcional)
which python3
python3 -V
python3 -c "import em"  # debe funcionar aquí

Build (usa EXACTAMENTE esta línea)
\colcon build 2>&1 | head -n 200

Test (usa EXACTAMENTE esta línea)
\colcon test 2>&1 | head -n 200
```

Notas para el agente

No uses otros comandos de build o test.

El backslash (\) delante de colcon fuerza el binario real, ignorando alias.

Ejecuta desde la raíz del repo (con src/ si es workspace).


---

## (Opcional, pero recomendable) Parchea tu Setup script para “matar” pyenv globalmente

Añade **al final** de tu script de Setup (después de `echo "[setup] Jazzy listo …"` o justo antes):

```bash
# [NEW] Desactiva pyenv en el contenedor para todas las shells
if [ -d "$HOME/.pyenv" ]; then mv "$HOME/.pyenv" "$HOME/.pyenv.disabled" || true; fi
printf '%s\n' 'export PATH=$(printf "%s" "$PATH" | tr ":" "\n" | grep -v -E "\.pyenv/shims" | paste -sd: -)' | sudo tee /etc/profile.d/90-nopyenv.sh >/dev/null```

Con esto, aunque olvides el saneo en algún script, no aparecerán los shims.

Verificación rápida (cuando lances Cloud)

Primero, en el paso “Entorno” que te puse arriba, deben salir:

which python3 → /usr/bin/python3

python3 -c "import em" → sin error

Luego, tus dos líneas exactas:

\colcon build 2>&1 | head -n 200
\colcon test  2>&1 | head -n 200


Si aún vieras /root/.pyenv/shims/python3 en el log, es que el bloque “Entorno” no se ejecutó (o el markdown estaba mal). Con el AGENTS.md que te paso arriba y el saneo del PATH, el error de No module named 'em' desaparece.
