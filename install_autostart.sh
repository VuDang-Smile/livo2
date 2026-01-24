#!/bin/bash
# Script to install autostart for run_working.sh
# This script will create systemd user service to automatically run run_working.sh on boot

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_NAME="livo2-working.service"
SERVICE_FILE="$HOME/.config/systemd/user/$SERVICE_NAME"
RUN_SCRIPT="$SCRIPT_DIR/run_working.sh"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Install Autostart for Livo2 Working${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if run_working.sh exists
if [ ! -f "$RUN_SCRIPT" ]; then
    echo -e "${RED}[ERROR] File not found: $RUN_SCRIPT${NC}"
    exit 1
fi

# Check executable permissions
if [ ! -x "$RUN_SCRIPT" ]; then
    echo -e "${YELLOW}[WARNING] run_working.sh is not executable. Adding permissions...${NC}"
    chmod +x "$RUN_SCRIPT"
fi

# Auto-detect DISPLAY
if [ -z "$DISPLAY" ]; then
    # Try common values
    if [ -S "/tmp/.X11-unix/X0" ]; then
        DETECTED_DISPLAY=":0"
    elif [ -S "/tmp/.X11-unix/X1" ]; then
        DETECTED_DISPLAY=":1"
    else
        DETECTED_DISPLAY=":0"
    fi
else
    DETECTED_DISPLAY="$DISPLAY"
fi

# Auto-detect XAUTHORITY from current environment
if [ -z "$XAUTHORITY" ]; then
    # Try to find XAUTHORITY from common locations
    if [ -f "$HOME/.Xauthority" ]; then
        DETECTED_XAUTHORITY="$HOME/.Xauthority"
    else
        # Wayland session - find mutter-Xwaylandauth file
        WAYLAND_AUTH=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
        if [ -n "$WAYLAND_AUTH" ] && [ -f "$WAYLAND_AUTH" ]; then
            DETECTED_XAUTHORITY="$WAYLAND_AUTH"
        else
            DETECTED_XAUTHORITY="$HOME/.Xauthority"
        fi
    fi
else
    DETECTED_XAUTHORITY="$XAUTHORITY"
fi

# Detect DBUS_SESSION_BUS_ADDRESS
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    DETECTED_DBUS="unix:path=/run/user/$(id -u)/bus"
else
    DETECTED_DBUS="$DBUS_SESSION_BUS_ADDRESS"
fi

# Detect XDG_RUNTIME_DIR
if [ -z "$XDG_RUNTIME_DIR" ]; then
    DETECTED_XDG_RUNTIME="/run/user/$(id -u)"
else
    DETECTED_XDG_RUNTIME="$XDG_RUNTIME_DIR"
fi

# Use wrapper script to auto-detect environment variables
WRAPPER_SCRIPT="$SCRIPT_DIR/run_working_wrapper.sh"

# Check if wrapper script exists
if [ ! -f "$WRAPPER_SCRIPT" ]; then
    echo -e "${RED}[ERROR] Wrapper script not found: $WRAPPER_SCRIPT${NC}"
    exit 1
fi

# Ensure wrapper script has executable permissions
if [ ! -x "$WRAPPER_SCRIPT" ]; then
    echo -e "${YELLOW}[WARNING] Wrapper script is not executable. Adding permissions...${NC}"
    chmod +x "$WRAPPER_SCRIPT"
fi

# Display installation information
echo -e "${GREEN}Installation Information:${NC}"
echo "  - Service name: $SERVICE_NAME"
echo "  - Service file: $SERVICE_FILE"
echo "  - Wrapper script: $WRAPPER_SCRIPT"
echo "  - Script to run: $RUN_SCRIPT"
echo "  - Working directory: $SCRIPT_DIR"
echo "  - DISPLAY: $DETECTED_DISPLAY (will auto-detect when running)"
echo "  - XAUTHORITY: will auto-detect when running (current: $DETECTED_XAUTHORITY)"
echo "  - DBUS_SESSION_BUS_ADDRESS: $DETECTED_DBUS"
echo "  - XDG_RUNTIME_DIR: $DETECTED_XDG_RUNTIME"
echo ""

# Check if service already exists
if [ -f "$SERVICE_FILE" ]; then
    echo -e "${YELLOW}[WARNING] Service file already exists at: $SERVICE_FILE${NC}"
    echo -e "${YELLOW}Do you want to overwrite the existing file? (y/n)${NC}"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo -e "${RED}Installation cancelled.${NC}"
        exit 0
    fi
    echo ""
fi

# Confirm with user
echo -e "${YELLOW}Do you want to continue with autostart installation? (y/n)${NC}"
read -r response
if [[ ! "$response" =~ ^[Yy]$ ]]; then
    echo -e "${RED}Installation cancelled.${NC}"
    exit 0
fi

echo ""
echo -e "${BLUE}[1/4] Creating systemd user directory...${NC}"
mkdir -p "$HOME/.config/systemd/user"

echo -e "${BLUE}[2/4] Creating systemd service file...${NC}"
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Livo2 Working GUI Application
After=graphical-session.target

[Service]
Type=simple
Environment="DISPLAY=$DETECTED_DISPLAY"
Environment="DBUS_SESSION_BUS_ADDRESS=$DETECTED_DBUS"
Environment="XDG_RUNTIME_DIR=$DETECTED_XDG_RUNTIME"
WorkingDirectory=$SCRIPT_DIR
ExecStart=$WRAPPER_SCRIPT
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=default.target
EOF

echo -e "${GREEN}✓ Service file created at: $SERVICE_FILE${NC}"

echo -e "${BLUE}[3/4] Reloading systemd daemon...${NC}"
systemctl --user daemon-reload
echo -e "${GREEN}✓ Systemd daemon reloaded${NC}"

echo -e "${BLUE}[4/4] Enabling service to auto-start on login...${NC}"
systemctl --user enable "$SERVICE_NAME"
echo -e "${GREEN}✓ Service enabled${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Installation completed successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Useful commands:${NC}"
echo "  - Start service now: ${GREEN}systemctl --user start $SERVICE_NAME${NC}"
echo "  - Stop service: ${GREEN}systemctl --user stop $SERVICE_NAME${NC}"
echo "  - Check status: ${GREEN}systemctl --user status $SERVICE_NAME${NC}"
echo "  - View logs: ${GREEN}journalctl --user -u $SERVICE_NAME -f${NC}"
echo "  - Disable autostart: ${GREEN}systemctl --user disable $SERVICE_NAME${NC}"
echo ""

# Ask if user wants to test now
echo -e "${YELLOW}Do you want to start the service now to test? (y/n)${NC}"
read -r test_response
if [[ "$test_response" =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${BLUE}Starting service...${NC}"
    systemctl --user start "$SERVICE_NAME" || true
    sleep 2
    echo ""
    echo -e "${BLUE}Service status:${NC}"
    systemctl --user status "$SERVICE_NAME" --no-pager || true
    echo ""
    echo -e "${GREEN}Service has been started. Check if the application is displayed.${NC}"
    echo -e "${YELLOW}If there are issues, check logs with: journalctl --user -u $SERVICE_NAME -f${NC}"
fi

echo ""
echo -e "${GREEN}Done!${NC}"
