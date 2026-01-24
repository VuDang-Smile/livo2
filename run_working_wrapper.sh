#!/bin/bash
# Wrapper script để tự động detect XAUTHORITY và các environment variables cần thiết
# Script này được gọi từ systemd service

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Detect DISPLAY nếu chưa có
if [ -z "$DISPLAY" ]; then
    if [ -S "/tmp/.X11-unix/X0" ]; then
        export DISPLAY=":0"
    elif [ -S "/tmp/.X11-unix/X1" ]; then
        export DISPLAY=":1"
    else
        export DISPLAY=":0"
    fi
fi

# Detect XAUTHORITY - ưu tiên từ environment, sau đó tìm file
if [ -z "$XAUTHORITY" ] || [ ! -f "$XAUTHORITY" ]; then
    # Thử tìm XAUTHORITY từ các vị trí phổ biến
    if [ -f "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    else
        # Wayland session - tìm file mutter-Xwaylandauth mới nhất
        USER_ID=$(id -u)
        WAYLAND_AUTH=$(ls -t /run/user/$USER_ID/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
        if [ -n "$WAYLAND_AUTH" ] && [ -f "$WAYLAND_AUTH" ]; then
            export XAUTHORITY="$WAYLAND_AUTH"
        fi
    fi
fi

# Detect DBUS_SESSION_BUS_ADDRESS
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    USER_ID=$(id -u)
    if [ -S "/run/user/$USER_ID/bus" ]; then
        export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$USER_ID/bus"
    fi
fi

# Detect XDG_RUNTIME_DIR
if [ -z "$XDG_RUNTIME_DIR" ]; then
    USER_ID=$(id -u)
    if [ -d "/run/user/$USER_ID" ]; then
        export XDG_RUNTIME_DIR="/run/user/$USER_ID"
    fi
fi

# Chạy script gốc
exec "$SCRIPT_DIR/run_working.sh"
