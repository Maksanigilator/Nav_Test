#!/usr/bin/env bash
# Виртуальный экран и noVNC. Вызывается из run.sh, когда у терминала нет DISPLAY.
set -euo pipefail

export DISPLAY="${DISPLAY:-:99}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"

NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_PORT="${VNC_PORT:-5900}"
disp_num="${DISPLAY#:}"
rm -f "/tmp/.X${disp_num}-lock" "/tmp/.X11-unix/X${disp_num}"
mkdir -p /tmp/novnc /tmp/.X11-unix

setsid Xvfb "$DISPLAY" -screen 0 1600x900x24 -ac +extension GLX +render -noreset \
  >/tmp/novnc/xvfb.log 2>&1 &

ready=0
for _ in $(seq 1 50); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 0.1
done
if [[ "$ready" -ne 1 ]]; then
  echo "Xvfb не стартовал" >&2
  cat /tmp/novnc/xvfb.log >&2 || true
  exit 1
fi

setsid openbox >/tmp/novnc/openbox.log 2>&1 &
x11vnc -display "$DISPLAY" -localhost -nopw -forever -shared \
  -rfbport "$VNC_PORT" -bg -o /tmp/novnc/x11vnc.log
setsid /usr/share/novnc/utils/novnc_proxy --vnc "localhost:${VNC_PORT}" --listen "$NOVNC_PORT" \
  >/tmp/novnc/novnc.log 2>&1 &

# vnc.html оставляет заставку «Loading», пока не поднимет свой интерфейс
# (в части браузеров это не случается вовсе). vnc_lite.html рисует экран сразу.
ip=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "RViz в браузере: http://127.0.0.1:${NOVNC_PORT}/vnc_lite.html?scale=1"
if [[ -n "${ip}" ]]; then
  echo "С другого компьютера: http://${ip}:${NOVNC_PORT}/vnc_lite.html?scale=1"
fi

exec "$@"
