#!/bin/bash

# Script build cho ROS2 workspace
# Hỗ trợ build các packages trong ws/src

set -e

# Màu sắc cho output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Đường dẫn
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ws"
ROS2_SETUP_SCRIPT="/opt/ros/jazzy/setup.bash"
DRIVE_WS_SETUP_SCRIPT="$SCRIPT_DIR/dependencies/drive_ws/install/setup.sh"

# Danh sách packages trong ws/src
declare -a PACKAGES=(
    "fast_livo"
    "fast_lio_localization"
    "direct_visual_lidar_calibration"
    "theta_driver"
    "vikit_common"
    "vikit_ros"
    "livox_msg_converter"
)

# Mảng lưu trạng thái chọn của từng package (0 = chưa chọn, 1 = đã chọn)
declare -a SELECTED=(
    0 0 0 0 0 0 0
)

# Hàm hiển thị menu
show_menu() {
    local current_selection=$1
    local clear_screen=${2:-1}  # Mặc định clear, nhưng có thể tắt
    
    if [ "$clear_screen" -eq 1 ]; then
        clear
    else
        # Không clear, chỉ in dòng phân cách
        echo ""
        echo -e "${BLUE}========================================${NC}"
    fi
    
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
    local first_display=1  # Flag để biết lần đầu hiển thị menu
    
    while true; do
        # Lần đầu không clear để giữ log, các lần sau clear để refresh menu
        if [ $first_display -eq 1 ]; then
            show_menu $current_selection 0  # Không clear
            first_display=0
        else
            show_menu $current_selection 1  # Clear để refresh
        fi
        
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

# Hàm source ROS2 base setup (cần thiết cho colcon build khi chạy "as program")
source_ros2_base() {
    # Kiểm tra xem ROS2 đã được source chưa (kiểm tra biến môi trường ROS_DISTRO)
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Source ROS2 Jazzy base setup...${NC}"
        echo -e "${BLUE}========================================${NC}"
        
        if [ ! -f "$ROS2_SETUP_SCRIPT" ]; then
            echo -e "${RED}Lỗi: Không tìm thấy ROS2 Jazzy tại: $ROS2_SETUP_SCRIPT${NC}"
            echo -e "${RED}Vui lòng cài đặt ROS2 Jazzy trước!${NC}"
            exit 1
        else
            source "$ROS2_SETUP_SCRIPT"
            echo -e "${GREEN}Đã source ROS2 Jazzy base setup thành công!${NC}"
            echo ""
        fi
    else
        echo -e "${GREEN}ROS2 environment đã được source (ROS_DISTRO=$ROS_DISTRO)${NC}"
        echo ""
    fi
}

# Hàm source livox driver 2 setup
source_livox_driver() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Source livox_ros_driver2 setup...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ ! -f "$DRIVE_WS_SETUP_SCRIPT" ]; then
        echo -e "${YELLOW}Cảnh báo: Không tìm thấy file setup.sh tại: $DRIVE_WS_SETUP_SCRIPT${NC}"
        echo -e "${YELLOW}Có thể livox_ros_driver2 chưa được build.${NC}"
        echo -e "${YELLOW}Script sẽ tiếp tục nhưng một số packages có thể cần driver này.${NC}"
        echo ""
    else
        source "$DRIVE_WS_SETUP_SCRIPT"
        echo -e "${GREEN}Đã source livox_ros_driver2 setup thành công!${NC}"
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
    local packages_arg="--packages-select"
    for pkg in "${packages_to_build[@]}"; do
        packages_arg="$packages_arg $pkg"
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
    
    # Source ROS2 base setup (cần thiết cho colcon build, đặc biệt khi chạy "as program")
    source_ros2_base
    
    # Source livox driver 2 setup
    source_livox_driver
    
    # Chọn packages
    echo -e "${YELLOW}Chọn packages để build${NC}"
    handle_menu_input
    
    # Build packages
    echo -e "${YELLOW}Build packages${NC}"
    build_packages
    
    # Tạm dừng trước khi thoát
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${YELLOW}Nhấn phím bất kỳ để thoát...${NC}"
    read -n 1 -s
}

# Chạy main function
main

