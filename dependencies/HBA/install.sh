#!/bin/bash

# Script install/build HBA standalone
# Script này sẽ tự động build HBA standalone tool

set -e

# Màu sắc cho output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Lấy đường dẫn script và project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
HBA_DIR="$SCRIPT_DIR"
BUILD_SCRIPT="$PROJECT_ROOT/scripts/build_hba_standalone.sh"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    HBA Standalone Installation${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Kiểm tra script build có tồn tại không
if [ ! -f "$BUILD_SCRIPT" ]; then
    echo -e "${RED}Error: Không tìm thấy script build tại: $BUILD_SCRIPT${NC}"
    exit 1
fi

# Kiểm tra CMakeLists.standalone.txt
if [ ! -f "$HBA_DIR/CMakeLists.standalone.txt" ]; then
    echo -e "${RED}Error: Không tìm thấy CMakeLists.standalone.txt trong $HBA_DIR${NC}"
    exit 1
fi

echo -e "${BLUE}HBA Directory:${NC} $HBA_DIR"
echo -e "${BLUE}Build Script:${NC} $BUILD_SCRIPT"
echo ""

# Chạy script build
echo -e "${YELLOW}Đang build HBA standalone...${NC}"
echo ""

chmod +x "$BUILD_SCRIPT"

if bash "$BUILD_SCRIPT"; then
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}HBA Standalone đã được build thành công!${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}Binary location:${NC} $HBA_DIR/build_standalone/hba_standalone"
    echo ""
    exit 0
else
    echo ""
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Build HBA thất bại!${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    exit 1
fi

