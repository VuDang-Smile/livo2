#!/bin/bash
# Script to uninstall autostart for run_working.sh
# This script will remove systemd user service that automatically runs run_working.sh on boot

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

echo -e "${RED}========================================${NC}"
echo -e "${RED}  Uninstall Autostart for Livo2 Working${NC}"
echo -e "${RED}========================================${NC}"
echo ""

# Check if service exists
if [ ! -f "$SERVICE_FILE" ]; then
    echo -e "${YELLOW}[WARNING] Service file does not exist at: $SERVICE_FILE${NC}"
    echo -e "${YELLOW}It seems autostart is not installed.${NC}"
    exit 0
fi

# Check if service is running
if systemctl --user is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo -e "${YELLOW}[NOTICE] Service is running.${NC}"
    echo -e "${YELLOW}Do you want to stop the service before uninstalling? (y/n)${NC}"
    read -r stop_response
    if [[ "$stop_response" =~ ^[Yy]$ ]]; then
        echo -e "${BLUE}Stopping service...${NC}"
        systemctl --user stop "$SERVICE_NAME" || true
        echo -e "${GREEN}✓ Service stopped${NC}"
    fi
    echo ""
fi

# Display information
echo -e "${GREEN}Uninstallation Information:${NC}"
echo "  - Service name: $SERVICE_NAME"
echo "  - Service file: $SERVICE_FILE"
echo ""

# Confirm with user
echo -e "${YELLOW}Are you sure you want to uninstall autostart? (y/n)${NC}"
read -r response
if [[ ! "$response" =~ ^[Yy]$ ]]; then
    echo -e "${RED}Uninstallation cancelled.${NC}"
    exit 0
fi

echo ""
echo -e "${BLUE}[1/3] Disabling service...${NC}"
systemctl --user disable "$SERVICE_NAME" 2>/dev/null || true
echo -e "${GREEN}✓ Service disabled${NC}"

echo -e "${BLUE}[2/3] Reloading systemd daemon...${NC}"
systemctl --user daemon-reload
echo -e "${GREEN}✓ Systemd daemon reloaded${NC}"

echo -e "${BLUE}[3/3] Removing service file...${NC}"
rm -f "$SERVICE_FILE"
echo -e "${GREEN}✓ Service file removed${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Uninstallation completed successfully!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Service has been completely uninstalled.${NC}"
echo -e "${BLUE}The application will no longer start automatically on boot.${NC}"
echo ""
echo -e "${GREEN}Done!${NC}"
