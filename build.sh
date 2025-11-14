#!/bin/bash

# Script build cho ROS2 workspace
# Hỗ trợ build driver và các packages trong ws/src

set -e

# Màu sắc cho output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Đường dẫn
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRIVE_WS_DIR="$SCRIPT_DIR/drive_ws"
WS_DIR="$SCRIPT_DIR/ws"
DRIVER_BUILD_SCRIPT="$DRIVE_WS_DIR/src/livox_ros_driver2/build.sh"
DRIVER_SETUP_SCRIPT="$DRIVE_WS_DIR/install/setup.sh"

# Danh sách packages trong ws/src
declare -a PACKAGES=(
    "fast_livo"
    "direct_visual_lidar_calibration"
    "theta_driver"
    "vikit_common"
    "vikit_ros"
    "livox_msg_converter"
)

# Mảng lưu trạng thái chọn của từng package (0 = chưa chọn, 1 = đã chọn)
declare -a SELECTED=(
    0 0 0 0 0 0
)

# Hàm hiển thị menu
show_menu() {
    local current_selection=$1
    clear
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    ROS2 Workspace Build Script${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Chọn packages để build:${NC}"
    echo ""
    
    for i in "${!PACKAGES[@]}"; do
        local marker=" "
        local pkg_name="${PACKAGES[$i]}"
        
        if [ "${SELECTED[$i]}" -eq 1 ]; then
            marker="${GREEN}[*]${NC}"
        else
            marker="${RED}[ ]${NC}"
        fi
        
        # Highlight package hiện tại
        if [ "$i" -eq "$current_selection" ]; then
            echo -e "  $marker ${BLUE}>>> ${pkg_name} <<<${NC}"
        else
            echo -e "  $marker ${pkg_name}"
        fi
    done
    
    echo ""
    echo -e "${YELLOW}Hướng dẫn:${NC}"
    echo "  - Dùng phím ↑/↓ để di chuyển"
    echo "  - Nhấn Space để chọn/bỏ chọn package hiện tại"
    echo "  - Nhấn số (1-${#PACKAGES[@]}) để chọn/bỏ chọn package tương ứng"
    echo "  - Nhấn Enter để bắt đầu build"
    echo "  - Nhấn 'q' để thoát"
    echo ""
}

# Hàm toggle selection
toggle_selection() {
    local index=$1
    if [ "${SELECTED[$index]}" -eq 1 ]; then
        SELECTED[$index]=0
    else
        SELECTED[$index]=1
    fi
}

# Hàm xử lý input từ người dùng
handle_menu_input() {
    local current_selection=0
    
    while true; do
        show_menu $current_selection
        
        # Đọc input
        IFS= read -rsn1 key
        
        # Xử lý escape sequences (arrow keys)
        if [ "$key" = $'\x1b' ]; then
            read -rsn1 -t 0.1 key
            if [ "$key" = "[" ]; then
                read -rsn1 -t 0.1 key
                case "$key" in
                    A) # Up arrow
                        current_selection=$((current_selection - 1))
                        if [ $current_selection -lt 0 ]; then
                            current_selection=$((${#PACKAGES[@]} - 1))
                        fi
                        continue
                        ;;
                    B) # Down arrow
                        current_selection=$((current_selection + 1))
                        if [ $current_selection -ge ${#PACKAGES[@]} ]; then
                            current_selection=0
                        fi
                        continue
                        ;;
                esac
            fi
        fi
        
        case "$key" in
            [1-9])
                local index=$((key - 1))
                if [ $index -lt ${#PACKAGES[@]} ]; then
                    toggle_selection $index
                    current_selection=$index
                fi
                ;;
            " ")
                # Space bar - toggle current selection
                toggle_selection $current_selection
                ;;
            "")
                # Enter key
                local has_selected=0
                for selected in "${SELECTED[@]}"; do
                    if [ "$selected" -eq 1 ]; then
                        has_selected=1
                        break
                    fi
                done
                
                if [ "$has_selected" -eq 1 ]; then
                    return 0
                else
                    echo -e "${RED}Vui lòng chọn ít nhất một package!${NC}"
                    sleep 1
                fi
                ;;
            [qQ])
                echo -e "${YELLOW}Đã hủy build.${NC}"
                exit 0
                ;;
            *)
                # Ignore other keys
                ;;
        esac
    done
}

# Hàm build driver
build_driver() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Bắt đầu build livox_ros_driver2...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ ! -f "$DRIVER_BUILD_SCRIPT" ]; then
        echo -e "${RED}Lỗi: Không tìm thấy file build.sh tại: $DRIVER_BUILD_SCRIPT${NC}"
        exit 1
    fi
    
    cd "$DRIVE_WS_DIR/src/livox_ros_driver2"
    bash build.sh jazzy
    cd "$SCRIPT_DIR"
    
    echo -e "${GREEN}Build driver hoàn tất!${NC}"
    echo ""
}

# Hàm source setup.sh
source_setup() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Source drive_ws/install/setup.sh...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ ! -f "$DRIVER_SETUP_SCRIPT" ]; then
        echo -e "${YELLOW}Cảnh báo: Không tìm thấy file setup.sh tại: $DRIVER_SETUP_SCRIPT${NC}"
        echo -e "${YELLOW}Có thể driver chưa được build.${NC}"
        echo ""
    else
        source "$DRIVER_SETUP_SCRIPT"
        echo -e "${GREEN}Đã source setup.sh thành công!${NC}"
        echo ""
    fi
}

# Hàm build packages
build_packages() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Bắt đầu build các packages đã chọn...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # Tạo danh sách packages cần build
    local packages_to_build=()
    for i in "${!PACKAGES[@]}"; do
        if [ "${SELECTED[$i]}" -eq 1 ]; then
            packages_to_build+=("${PACKAGES[$i]}")
        fi
    done
    
    if [ ${#packages_to_build[@]} -eq 0 ]; then
        echo -e "${RED}Không có package nào được chọn để build!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}Packages sẽ được build:${NC}"
    for pkg in "${packages_to_build[@]}"; do
        echo -e "  - ${GREEN}$pkg${NC}"
    done
    echo ""
    
    # Chuyển đến thư mục ws
    cd "$WS_DIR"
    
    # Build với colcon
    local packages_arg=""
    for pkg in "${packages_to_build[@]}"; do
        if [ -z "$packages_arg" ]; then
            packages_arg="--packages-select $pkg"
        else
            packages_arg="$packages_arg --packages-select $pkg"
        fi
    done
    
    echo -e "${BLUE}Chạy lệnh: colcon build $packages_arg --symlink-install${NC}"
    echo ""
    
    colcon build $packages_arg --symlink-install
    
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Build hoàn tất!${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# Main function
main() {
    echo -e "${BLUE}ROS2 Workspace Build Script${NC}"
    echo ""
    
    # Bước 1: Hỏi có build driver không
    echo -e "${YELLOW}Bước 1: Build livox_ros_driver2?${NC}"
    read -p "Có build driver không? (y/n): " build_driver_choice
    
    if [[ "$build_driver_choice" =~ ^[Yy]$ ]]; then
        build_driver
    else
        echo -e "${YELLOW}Bỏ qua build driver.${NC}"
        echo ""
    fi
    
    # Bước 2: Source setup.sh
    source_setup
    
    # Bước 3: Chọn packages
    echo -e "${YELLOW}Bước 3: Chọn packages để build${NC}"
    handle_menu_input
    
    # Bước 4: Build packages
    echo -e "${YELLOW}Bước 4: Build packages${NC}"
    build_packages
}

# Chạy main function
main

