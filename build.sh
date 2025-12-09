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
ROS2_SETUP_SCRIPT="/opt/ros/jazzy/setup.bash"

# Danh sách packages trong ws/src
declare -a PACKAGES=(
    "theta_driver"
    "livox_msg_converter"
)

# Mảng lưu trạng thái chọn của từng package (0 = chưa chọn, 1 = đã chọn)
declare -a SELECTED=(
    0 0
)

# Biến lưu trạng thái có build Livox driver không (0 = không, 1 = có)
BUILD_LIVOX_DRIVER=0

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

# Hàm hỏi có build Livox driver không
ask_build_livox_driver() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Build Livox Driver${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Bạn có muốn build Livox driver (drive_ws) không?${NC}"
    echo ""
    echo -e "  ${GREEN}[Y]${NC} - Có, build Livox driver"
    echo -e "  ${RED}[N]${NC} - Không, bỏ qua build driver"
    echo ""
    echo -e "${YELLOW}Nhập lựa chọn (Y/n): ${NC}"
    
    while true; do
        read -r response
        case "$response" in
            [Yy]|[Yy][Ee][Ss]|"")
                BUILD_LIVOX_DRIVER=1
                echo -e "${GREEN}Đã chọn: Build Livox driver${NC}"
                echo ""
                return 0
                ;;
            [Nn]|[Nn][Oo])
                BUILD_LIVOX_DRIVER=0
                echo -e "${YELLOW}Đã chọn: Bỏ qua build Livox driver${NC}"
                echo ""
                return 0
                ;;
            *)
                echo -e "${RED}Lựa chọn không hợp lệ. Vui lòng nhập Y (có) hoặc N (không): ${NC}"
                ;;
        esac
    done
}

# Hàm build driver
build_driver() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Bắt đầu build Livox driver (drive_ws)...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ ! -d "$DRIVE_WS_DIR" ]; then
        echo -e "${RED}Lỗi: Không tìm thấy drive_ws tại: $DRIVE_WS_DIR${NC}"
        echo -e "${YELLOW}Bỏ qua build driver.${NC}"
        echo ""
        return 1
    fi
    
    if [ ! -d "$DRIVE_WS_DIR/src/livox_ros_driver2" ]; then
        echo -e "${RED}Lỗi: Không tìm thấy livox_ros_driver2 tại: $DRIVE_WS_DIR/src/livox_ros_driver2${NC}"
        echo -e "${YELLOW}Bỏ qua build driver.${NC}"
        echo ""
        return 1
    fi
    
    # Lưu thư mục hiện tại
    local current_dir=$(pwd)
    
    # Chuyển đến thư mục livox_ros_driver2 và chạy build.sh
    cd "$DRIVE_WS_DIR/src/livox_ros_driver2"
    
    echo -e "${GREEN}Đang build livox_ros_driver2 cho ROS2 Jazzy...${NC}"
    echo ""
    
    # Chạy build.sh với argument "jazzy"
    bash build.sh jazzy
    local build_result=$?
    
    # Quay lại thư mục gốc
    cd "$current_dir"
    
    if [ $build_result -eq 0 ]; then
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}Build Livox driver thành công!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        return 0
    else
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}Build Livox driver thất bại!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        return 1
    fi
}

# Hàm source ROS2 base setup
source_ros2_base() {
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
}

# Hàm source setup.sh
source_setup() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Source drive_ws setup.sh...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    local drive_ws_setup="$DRIVE_WS_DIR/install/setup.sh"
    
    if [ ! -f "$drive_ws_setup" ]; then
        echo -e "${YELLOW}Cảnh báo: Không tìm thấy drive_ws/install/setup.sh tại: $drive_ws_setup${NC}"
        echo -e "${YELLOW}Có thể drive_ws chưa được build hoặc build thất bại.${NC}"
        echo -e "${YELLOW}Bỏ qua source drive_ws setup.${NC}"
        echo ""
        return 1
    else
        source "$drive_ws_setup"
        echo -e "${GREEN}Đã source drive_ws setup.sh thành công!${NC}"
        echo ""
        return 0
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
    
    # Bước 0: Source ROS2 base setup (bắt buộc)
    source_ros2_base
    
    # Bước 1: Hỏi có build Livox driver không
    echo -e "${YELLOW}Bước 1: Xác nhận build Livox driver${NC}"
    ask_build_livox_driver
    
    # Bước 2: Build driver (nếu được chọn)
    if [ "$BUILD_LIVOX_DRIVER" -eq 1 ]; then
        echo -e "${YELLOW}Bước 2: Build Livox driver (drive_ws)${NC}"
        build_driver
        
        # Bước 3: Source drive_ws setup.sh
        echo -e "${YELLOW}Bước 3: Source drive_ws setup.sh${NC}"
        source_setup
    else
        echo -e "${YELLOW}Bước 2: Bỏ qua build Livox driver${NC}"
        echo ""
        # Vẫn thử source drive_ws setup.sh nếu đã được build trước đó
        echo -e "${YELLOW}Bước 3: Kiểm tra và source drive_ws setup.sh (nếu có)${NC}"
        source_setup
    fi
    
    # Bước 4: Chọn packages
    echo -e "${YELLOW}Bước 4: Chọn packages để build${NC}"
    handle_menu_input
    
    # Bước 5: Build packages
    echo -e "${YELLOW}Bước 5: Build packages${NC}"
    build_packages
    
    # Tạm dừng trước khi thoát
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${YELLOW}Nhấn phím bất kỳ để thoát...${NC}"
    read -n 1 -s
}

# Chạy main function
main

