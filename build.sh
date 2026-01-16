#!/bin/bash

# Script to build ROS2 workspace
# Supports building packages in ws/src

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ws"
ROS2_SETUP_SCRIPT="/opt/ros/jazzy/setup.bash"
DRIVE_WS_SETUP_SCRIPT="$SCRIPT_DIR/dependencies/drive_ws/install/setup.sh"

# List of packages in ws/src + HBA standalone
declare -a PACKAGES=(
    "fast_livo"
    "fast_lio_localization"
    "hba_standalone"
    "direct_visual_lidar_calibration"
    "theta_driver"
    "vikit_common"
    "vikit_ros"
    "livox_msg_converter"
)

# Array to store selection state of each package (0 = not selected, 1 = selected)
declare -a SELECTED=(
    0 0 0 0 0 0 0 0
)

# Function to display menu
show_menu() {
    local current_selection=$1
    local clear_screen=${2:-1}  # Default clear, but can be disabled
    
    if [ "$clear_screen" -eq 1 ]; then
        clear
    else
        # Don't clear, just print separator line
        echo ""
        echo -e "${BLUE}========================================${NC}"
    fi
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    ROS2 Workspace Build Script${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${YELLOW}Select packages to build:${NC}"
    echo ""
    
    for i in "${!PACKAGES[@]}"; do
        local marker=" "
        local pkg_name="${PACKAGES[$i]}"
        
        if [ "${SELECTED[$i]}" -eq 1 ]; then
            marker="${GREEN}[*]${NC}"
        else
            marker="${RED}[ ]${NC}"
        fi
        
        # Highlight current package
        if [ "$i" -eq "$current_selection" ]; then
            echo -e "  $marker ${BLUE}>>> ${pkg_name} <<<${NC}"
        else
            echo -e "  $marker ${pkg_name}"
        fi
    done
    
    echo ""
    # Show Select All / Deselect All option
    if all_selected; then
        echo -e "  ${GREEN}[*]${NC} ${GREEN}Select All${NC} (Press 'a' to deselect all)"
    else
        echo -e "  ${RED}[ ]${NC} ${YELLOW}Select All${NC} (Press 'a' to select all)"
    fi
    
    echo ""
    echo -e "${YELLOW}Instructions:${NC}"
    echo "  - Use ↑/↓ keys to navigate"
    echo "  - Press Space to toggle current package"
    echo "  - Press number (1-${#PACKAGES[@]}) to toggle corresponding package"
    echo "  - Press 'a' to select/deselect all packages"
    echo "  - Press Enter to start build"
    echo "  - Press 'q' to exit"
    echo ""
}

# Function to toggle selection
toggle_selection() {
    local index=$1
    if [ "${SELECTED[$index]}" -eq 1 ]; then
        SELECTED[$index]=0
    else
        SELECTED[$index]=1
    fi
}

# Function to select all packages
select_all() {
    for i in "${!PACKAGES[@]}"; do
        SELECTED[$i]=1
    done
}

# Function to deselect all packages
deselect_all() {
    for i in "${!PACKAGES[@]}"; do
        SELECTED[$i]=0
    done
}

# Function to check if all packages are selected
all_selected() {
    for selected in "${SELECTED[@]}"; do
        if [ "$selected" -eq 0 ]; then
            return 1
        fi
    done
    return 0
}

# Function to handle user input
handle_menu_input() {
    local current_selection=0
    local first_display=1  # Flag to know first menu display
    
    while true; do
        # First time don't clear to keep log, subsequent times clear to refresh menu
        if [ $first_display -eq 1 ]; then
            show_menu $current_selection 0  # Don't clear
            first_display=0
        else
            show_menu $current_selection 1  # Clear to refresh
        fi
        
        # Read input
        IFS= read -rsn1 key
        
        # Handle escape sequences (arrow keys)
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
            [aA])
                # 'a' key - toggle select all / deselect all
                if all_selected; then
                    deselect_all
                else
                    select_all
                fi
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
                    echo -e "${RED}Please select at least one package!${NC}"
                    sleep 1
                fi
                ;;
            [qQ])
                echo -e "${YELLOW}Build cancelled.${NC}"
                exit 0
                ;;
            *)
                # Ignore other keys
                ;;
        esac
    done
}

# Function to source ROS2 base setup (required for colcon build when running "as program")
source_ros2_base() {
    # Check if ROS2 has been sourced (check ROS_DISTRO environment variable)
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Source ROS2 Jazzy base setup...${NC}"
        echo -e "${BLUE}========================================${NC}"
        
        if [ ! -f "$ROS2_SETUP_SCRIPT" ]; then
            echo -e "${RED}Error: ROS2 Jazzy not found at: $ROS2_SETUP_SCRIPT${NC}"
            echo -e "${RED}Please install ROS2 Jazzy first!${NC}"
            exit 1
        else
            source "$ROS2_SETUP_SCRIPT"
            echo -e "${GREEN}Successfully sourced ROS2 Jazzy base setup!${NC}"
            echo ""
        fi
    else
        echo -e "${GREEN}ROS2 environment already sourced (ROS_DISTRO=$ROS_DISTRO)${NC}"
        echo ""
    fi
}

# Function to source livox driver 2 setup
source_livox_driver() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Source livox_ros_driver2 setup...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if [ ! -f "$DRIVE_WS_SETUP_SCRIPT" ]; then
        echo -e "${YELLOW}Warning: setup.sh file not found at: $DRIVE_WS_SETUP_SCRIPT${NC}"
        echo -e "${YELLOW}livox_ros_driver2 may not have been built.${NC}"
        echo -e "${YELLOW}Script will continue but some packages may require this driver.${NC}"
        echo ""
    else
        source "$DRIVE_WS_SETUP_SCRIPT"
        echo -e "${GREEN}Successfully sourced livox_ros_driver2 setup!${NC}"
        echo ""
    fi
}

# Function to prepare Log directories for FAST-LIVO2
prepare_log_directories() {
    # Check if fast_livo will be built
    local build_fast_livo=0
    for i in "${!PACKAGES[@]}"; do
        if [ "${SELECTED[$i]}" -eq 1 ] && [ "${PACKAGES[$i]}" = "fast_livo" ]; then
            build_fast_livo=1
            break
        fi
    done
    
    # Only create Log directory if building fast_livo
    if [ $build_fast_livo -eq 1 ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Preparing Log directories for FAST-LIVO2...${NC}"
        echo -e "${BLUE}========================================${NC}"
        
        # Path to Log directory
        local log_dir="$WS_DIR/src/FAST-LIVO2/Log"
        local pcd_dir="$log_dir/PCD"
        local colmap_dir="$log_dir/Colmap/sparse/0"
        
        # Create necessary directories
        mkdir -p "$pcd_dir"
        mkdir -p "$colmap_dir"
        mkdir -p "$log_dir"
        
        # Set permissions (755 for directories, 644 for files if any)
        chmod -R 755 "$log_dir" 2>/dev/null || true
        
        # Check write permissions
        if [ -w "$pcd_dir" ]; then
            echo -e "${GREEN}✅ Created and set permissions for Log directory${NC}"
            echo -e "   - PCD directory: $pcd_dir"
            echo -e "   - Colmap directory: $colmap_dir"
        else
            echo -e "${YELLOW}⚠️  Warning: Cannot write to Log directory${NC}"
            echo -e "   Try running: chmod -R 755 $log_dir"
        fi
        echo ""
    fi
}

# Function to build packages
build_packages() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Starting build of selected packages...${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # Create list of packages to build
    local packages_to_build=()
    local build_hba=0
    
    for i in "${!PACKAGES[@]}"; do
        if [ "${SELECTED[$i]}" -eq 1 ]; then
            if [ "${PACKAGES[$i]}" = "hba_standalone" ]; then
                build_hba=1
            else
                packages_to_build+=("${PACKAGES[$i]}")
            fi
        fi
    done
    
    if [ ${#packages_to_build[@]} -eq 0 ] && [ $build_hba -eq 0 ]; then
        echo -e "${RED}No packages selected to build!${NC}"
        return 1
    fi

    # 1. Build HBA standalone if selected
    if [ $build_hba -eq 1 ]; then
        echo -e "${YELLOW}>>> Building HBA standalone...${NC}"
        local hba_script="$SCRIPT_DIR/scripts/build_hba_standalone.sh"
        if [ -f "$hba_script" ]; then
            # Run existing HBA build script
            set +e
            bash "$hba_script"
            local hba_status=$?
            set -e
            if [ $hba_status -ne 0 ]; then
                echo -e "${RED}HBA build failed!${NC}"
                return $hba_status
            fi
        else
            echo -e "${RED}Error: HBA build script not found at $hba_script${NC}"
            return 1
        fi
        echo ""
    fi

    # 2. Build ROS2 packages with colcon
    local build_status=0
    if [ ${#packages_to_build[@]} -gt 0 ]; then
        echo -e "${GREEN}ROS2 Packages to be built:${NC}"
        for pkg in "${packages_to_build[@]}"; do
            echo -e "  - ${GREEN}$pkg${NC}"
        done
        echo ""
        
        # Change to ws directory
        cd "$WS_DIR" || return 1
        
        # Build with colcon
        local packages_arg="--packages-select"
        for pkg in "${packages_to_build[@]}"; do
            packages_arg="$packages_arg $pkg"
        done
        
        echo -e "${BLUE}Running command: colcon build $packages_arg --symlink-install${NC}"
        echo ""
        
        # Temporarily disable set -e to catch build errors and display full error messages
        set +e
        colcon build $packages_arg --symlink-install 2>&1
        build_status=$?
        # Don't re-enable set -e here to avoid early exit
    fi
    
    echo ""
    if [ $build_status -eq 0 ]; then
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}Build completed successfully!${NC}"
        echo -e "${GREEN}========================================${NC}"
    else
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}Build failed!${NC}"
        echo -e "${RED}Please check the error messages above.${NC}"
        echo -e "${RED}========================================${NC}"
    fi
    
    return $build_status
}

# Main function
main() {
    echo -e "${BLUE}ROS2 Workspace Build Script${NC}"
    echo ""
    
    # Source ROS2 base setup (required for colcon build, especially when running "as program")
    source_ros2_base
    
    # Source livox driver 2 setup
    source_livox_driver
    
    # Auto-select all packages
    echo -e "${YELLOW}Auto-selecting all packages to build...${NC}"
    select_all
    echo -e "${GREEN}All packages selected:${NC}"
    for i in "${!PACKAGES[@]}"; do
        echo -e "  - ${GREEN}${PACKAGES[$i]}${NC}"
    done
    echo ""
    
    # Prepare Log directories (before building)
    prepare_log_directories
    
    # Build packages
    echo -e "${YELLOW}Build packages${NC}"
    # Disable set -e to avoid immediate exit on build failure and display full errors
    set +e
    build_packages
    local build_result=$?
    # Keep set +e to ensure "press enter to exit" is always executed
    
    # Pause before exiting (always display whether build succeeded or failed)
    echo ""
    echo -e "${BLUE}========================================${NC}"
    if [ $build_result -eq 0 ]; then
        echo -e "${GREEN}Press Enter to exit...${NC}"
    else
        echo -e "${RED}Press Enter to exit...${NC}"
    fi
    read -r
    
    # Return exit code corresponding to build result
    exit $build_result
}

# Run main function
main

