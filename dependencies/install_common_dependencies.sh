#!/bin/bash

# Common ROS2 Dependencies Installation Script
# Installs common ROS2 dependencies required for development

# Don't use set -e to handle errors better
# set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Variables to track installation status
INSTALL_SUCCESS=false
INSTALL_FAILED=false

# List of dependencies to install
# Add more dependencies here as needed
DEPENDENCIES=(
    "ros-jazzy-ament-cmake-auto"
    "ros-jazzy-pcl-ros"
    "libgstreamer1.0-dev"
    "libgstreamer-plugins-base1.0-dev"
    "libomp-dev"
    "libboost-all-dev"
    "libglm-dev"
    "libglfw3-dev"
    "libpng-dev"
    "libjpeg-dev"
    "libpcl-dev"
    "libgoogle-glog-dev"
    "python3-pip"
)

# Python packages to install via pip
# Note: Install order matters - numpy must be installed before ros2-numpy
# For Python 3.12, numpy < 1.24 may not have wheels, so we'll handle it specially
PYTHON_PACKAGES=(
    "transforms3d"  # Alternative to tf_transformations (provides quaternion_from_euler)
    "open3d"
)

# Numpy will be handled separately based on Python version and ros2-numpy requirements

# Packages that need special handling
# numpy and ros2-numpy will be handled together to resolve version conflicts
PYTHON_PACKAGES_SPECIAL=(
    "numpy"  # Will be handled with ros2-numpy to resolve version
    "ros2-numpy"  # May require numpy==1.24.2, will handle separately
    "tf_transformations"  # Try from PyPI, fallback to git if not available
)

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to display result message and confirm before exit
confirm_exit() {
    local exit_code=$1
    local message=$2
    
    echo ""
    echo -e "${BLUE}========================================${NC}"
    
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}    SUCCESS!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo -e "${GREEN}$message${NC}"
    else
        echo -e "${RED}    FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        echo -e "${RED}$message${NC}"
    fi
    
    echo ""
    echo -e "${YELLOW}Press Enter to exit...${NC}"
    read -r
    exit $exit_code
}

# Function to check if a package is already installed
check_package_installed() {
    local package=$1
    
    if dpkg -l | grep -q "^ii.*${package}"; then
        return 0  # Already installed
    fi
    
    return 1  # Not installed
}

# Function to install a single dependency
install_dependency() {
    local package=$1
    
    print_info "Installing ${package}..."
    
    if ! sudo apt update; then
        print_error "Failed to update apt packages."
        return 1
    fi
    
    if ! sudo apt install -y "${package}"; then
        print_error "Failed to install ${package}."
        return 1
    fi
    
    print_success "${package} installed successfully."
    return 0
}

# Function to install all dependencies
install_all_dependencies() {
    local failed_packages=()
    local installed_count=0
    local skipped_count=0
    
    print_info "Checking and installing dependencies..."
    echo ""
    
    for package in "${DEPENDENCIES[@]}"; do
        echo -e "${BLUE}----------------------------------------${NC}"
        print_info "Processing: ${package}"
        
        if check_package_installed "${package}"; then
            print_success "${package} is already installed. Skipping..."
            skipped_count=$((skipped_count + 1))
        else
            if install_dependency "${package}"; then
                installed_count=$((installed_count + 1))
            else
                failed_packages+=("${package}")
                INSTALL_FAILED=true
            fi
        fi
        echo ""
    done
    
    # Summary
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Installation Summary${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    print_info "Total packages: ${#DEPENDENCIES[@]}"
    print_success "Installed: ${installed_count}"
    print_info "Already installed (skipped): ${skipped_count}"
    
    if [ ${#failed_packages[@]} -gt 0 ]; then
        print_error "Failed: ${#failed_packages[@]}"
        echo ""
        print_error "Failed packages:"
        for pkg in "${failed_packages[@]}"; do
            echo -e "  ${RED}- ${pkg}${NC}"
        done
        echo ""
        return 1
    else
        INSTALL_SUCCESS=true
        echo ""
        return 0
    fi
}

# Function to install Python packages via pip
install_python_packages() {
    print_info "Installing Python packages via pip..."
    echo ""
    
    # Check if pip is available
    if ! command -v pip3 &> /dev/null && ! command -v python3 -m pip &> /dev/null; then
        print_error "pip3 is not available. Please install python3-pip first."
        return 1
    fi
    
    # Determine pip command
    if command -v pip3 &> /dev/null; then
        PIP_CMD="pip3"
    else
        PIP_CMD="python3 -m pip"
    fi
    
    local failed_packages=()
    local installed_count=0
    local skipped_count=0
    
    for package in "${PYTHON_PACKAGES[@]}"; do
        echo -e "${BLUE}----------------------------------------${NC}"
        print_info "Processing Python package: ${package}"
        
        # Special handling for numpy with version constraint
        if [[ "$package" == "numpy<1.24" ]]; then
            # Check if numpy is installed and its version
            if $PIP_CMD show numpy &> /dev/null; then
                local numpy_version=$($PIP_CMD show numpy | grep "^Version:" | awk '{print $2}')
                print_info "Current numpy version: ${numpy_version}"
                
                # Check if version is < 1.24 using Python
                # Compare version strings: split by '.' and compare numerically
                local version_check=$(python3 <<EOF
import sys
try:
    current = tuple(map(int, "${numpy_version}".split('.')))
    target = (1, 24)
    if current < target:
        sys.exit(0)
    else:
        sys.exit(1)
except Exception as e:
    # If parsing fails, assume needs upgrade
    sys.exit(1)
EOF
)
                
                # Check if version_check has a valid value (handle empty or non-numeric)
                if [ -z "$version_check" ] || ! [[ "$version_check" =~ ^[0-9]+$ ]]; then
                    version_check=1  # Default to failure if empty or invalid
                fi
                
                if [ "$version_check" -eq 0 ]; then
                    print_success "numpy ${numpy_version} is already installed and compatible (< 1.24). Skipping..."
                    skipped_count=$((skipped_count + 1))
                else
                    print_warning "numpy ${numpy_version} is installed but >= 1.24. Downgrading to < 1.24..."
                    # Uninstall current numpy first to avoid build issues
                    print_info "Uninstalling current numpy..."
                    $PIP_CMD uninstall -y numpy 2>/dev/null || true
                    
                    # Install numpy<1.24 using only binary wheels (no build from source)
                    print_info "Installing numpy<1.24 (using pre-built wheels only)..."
                    local install_success=false
                    
                    # Try with --only-binary to avoid building from source
                    if $PIP_CMD install --break-system-packages --only-binary :all: --no-cache-dir "numpy<1.24" 2>&1 | grep -q "Successfully installed"; then
                        install_success=true
                    elif $PIP_CMD install --break-system-packages --only-binary :all: "numpy<1.24" 2>&1 | grep -q "Successfully installed"; then
                        install_success=true
                    elif $PIP_CMD install --break-system-packages --prefer-binary "numpy<1.24" 2>&1 | grep -q "Successfully installed"; then
                        install_success=true
                    fi
                    
                    if [ "$install_success" = true ]; then
                        print_success "numpy downgraded successfully (using pre-built wheels)."
                        installed_count=$((installed_count + 1))
                    else
                        print_error "Failed to downgrade numpy. Trying alternative approach..."
                        # Last resort: try installing specific version that has wheels
                        if $PIP_CMD install --break-system-packages --only-binary :all: "numpy==1.23.5" 2>&1 | grep -q "Successfully installed"; then
                            print_success "numpy downgraded to 1.23.5 (using pre-built wheels)."
                            installed_count=$((installed_count + 1))
                        else
                            print_error "Failed to downgrade numpy. You may need to manually fix numpy version."
                            failed_packages+=("${package}")
                            INSTALL_FAILED=true
                        fi
                    fi
                fi
            else
                print_info "Installing numpy<1.24..."
                # Use only binary wheels to avoid build issues with Python 3.12
                local install_success=false
                
                # Try with --only-binary to avoid building from source
                if $PIP_CMD install --break-system-packages --only-binary :all: --no-cache-dir "numpy<1.24" 2>&1 | grep -q "Successfully installed"; then
                    install_success=true
                elif $PIP_CMD install --break-system-packages --only-binary :all: "numpy<1.24" 2>&1 | grep -q "Successfully installed"; then
                    install_success=true
                elif $PIP_CMD install --break-system-packages --prefer-binary "numpy<1.24" 2>&1 | grep -q "Successfully installed"; then
                    install_success=true
                fi
                
                if [ "$install_success" = true ]; then
                    print_success "numpy<1.24 installed successfully (using pre-built wheels)."
                    installed_count=$((installed_count + 1))
                else
                    print_error "Failed to install numpy<1.24. Trying specific version..."
                    # Try installing specific version that has wheels
                    if $PIP_CMD install --break-system-packages --only-binary :all: "numpy==1.23.5" 2>&1 | grep -q "Successfully installed"; then
                        print_success "numpy 1.23.5 installed successfully (using pre-built wheels)."
                        installed_count=$((installed_count + 1))
                    else
                        print_error "Failed to install numpy<1.24."
                        failed_packages+=("${package}")
                        INSTALL_FAILED=true
                    fi
                fi
            fi
        else
            # For other packages, extract package name without version constraint for checking
            local package_name=$(echo "$package" | sed 's/[<>=!].*//')
            
            # Check if package is already installed
            if $PIP_CMD show "${package_name}" &> /dev/null; then
                print_success "${package_name} is already installed. Skipping..."
                skipped_count=$((skipped_count + 1))
            else
                print_info "Installing ${package}..."
                # Try with --user first, then --break-system-packages if needed
                if $PIP_CMD install --user "${package}" 2>&1 | grep -q "externally-managed-environment"; then
                    print_warning "externally-managed-environment detected. Using --break-system-packages..."
                    if $PIP_CMD install --break-system-packages "${package}"; then
                        print_success "${package} installed successfully (with --break-system-packages)."
                        installed_count=$((installed_count + 1))
                    else
                        print_error "Failed to install ${package} even with --break-system-packages."
                        failed_packages+=("${package}")
                        INSTALL_FAILED=true
                    fi
                elif ! $PIP_CMD install --user "${package}" 2>/dev/null; then
                    # If --user failed, try --break-system-packages
                    print_warning "Installation with --user failed. Trying with --break-system-packages..."
                    if $PIP_CMD install --break-system-packages "${package}"; then
                        print_success "${package} installed successfully (with --break-system-packages)."
                        installed_count=$((installed_count + 1))
                    else
                        print_error "Failed to install ${package}."
                        failed_packages+=("${package}")
                        INSTALL_FAILED=true
                    fi
                else
                    print_success "${package} installed successfully."
                    installed_count=$((installed_count + 1))
                fi
            fi
        fi
        echo ""
    done
    
    # Summary
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Python Packages Installation Summary${NC}"
        echo -e "${BLUE}========================================${NC}"
        echo ""
        print_info "Total Python packages: ${#PYTHON_PACKAGES[@]}"
        print_success "Installed: ${installed_count}"
        print_info "Already installed (skipped): ${skipped_count}"
        
        if [ ${#failed_packages[@]} -gt 0 ]; then
            print_error "Failed: ${#failed_packages[@]}"
            echo ""
            print_error "Failed packages:"
            for pkg in "${failed_packages[@]}"; do
                echo -e "  ${RED}- ${pkg}${NC}"
            done
            echo ""
            return 1
        fi
    fi
    
    return 0
}

# Function to install special Python packages (those with version conflicts or need git)
install_special_python_packages() {
    if [ ${#PYTHON_PACKAGES_SPECIAL[@]} -eq 0 ]; then
        return 0
    fi
    
    print_info "Installing special Python packages..."
    echo ""
    
    # Check if pip is available
    if ! command -v pip3 &> /dev/null && ! command -v python3 -m pip &> /dev/null; then
        print_error "pip3 is not available. Please install python3-pip first."
        return 1
    fi
    
    # Determine pip command
    if command -v pip3 &> /dev/null; then
        PIP_CMD="pip3"
    else
        PIP_CMD="python3 -m pip"
    fi
    
    local failed_packages=()
    local installed_count=0
    
    for package in "${PYTHON_PACKAGES_SPECIAL[@]}"; do
        echo -e "${BLUE}----------------------------------------${NC}"
        print_info "Processing special package: ${package}"
        
        # Extract package name for checking
        local package_name=$(echo "$package" | sed 's/[<>=!].*//')
        
        # Check if package is already installed
        if $PIP_CMD show "${package_name}" &> /dev/null; then
            print_success "${package_name} is already installed. Skipping..."
            continue
        fi
        
        # Special handling for tf_transformations - try multiple sources
        if [[ "$package" == "tf_transformations" ]]; then
            print_info "Installing tf_transformations..."
            
            # Try 1: PyPI (if exists)
            print_info "  Trying PyPI..."
            if $PIP_CMD install --break-system-packages "tf_transformations" 2>&1 | grep -q "Successfully installed"; then
                print_success "tf_transformations installed from PyPI."
                installed_count=$((installed_count + 1))
                continue
            fi
            
            # Try 2: Install from ROS2 geometry2 (tf2_py)
            print_info "  Trying ROS2 geometry2/tf2_py from git..."
            if $PIP_CMD install --break-system-packages "git+https://github.com/ros/geometry2.git#subdirectory=tf2_py" 2>&1 | grep -q "Successfully installed"; then
                print_success "tf_transformations installed from git (geometry2/tf2_py)."
                installed_count=$((installed_count + 1))
                continue
            fi
            
            # Try 3: Install tf2_ros Python package (may provide tf_transformations)
            print_info "  Trying tf2_ros Python package..."
            if $PIP_CMD install --break-system-packages "tf2_ros" 2>&1 | grep -q "Successfully installed"; then
                # Check if tf_transformations is now available
                if python3 -c "import tf_transformations" 2>/dev/null; then
                    print_success "tf_transformations available via tf2_ros."
                    installed_count=$((installed_count + 1))
                    continue
                fi
            fi
            
            # Try 4: Install from alternative source
            print_info "  Trying alternative: ros-tf-transformations..."
            if $PIP_CMD install --break-system-packages "git+https://github.com/ros/geometry.git#subdirectory/tf" 2>&1 | grep -q "Successfully installed"; then
                print_success "tf_transformations installed from geometry/tf."
                installed_count=$((installed_count + 1))
                continue
            fi
            
            # Final fallback: Use transforms3d (already installed) and create compatibility layer
            print_warning "Could not install tf_transformations from any source."
            print_info "Note: transforms3d is already installed and provides quaternion_from_euler."
            print_info "You may need to update code to use 'transforms3d' instead of 'tf_transformations'."
            print_info "Or create a compatibility wrapper: import transforms3d as tf_transformations"
            
            # Don't fail - transforms3d can be used as replacement
            print_warning "Skipping tf_transformations (using transforms3d as alternative)."
            continue
        fi
        
        # Special handling for ros2-numpy - it will install numpy with correct version
        if [[ "$package" == "ros2-numpy" ]]; then
            print_info "Installing ros2-numpy..."
            
            # Check Python version
            local python_version=$(python3 --version 2>&1 | awk '{print $2}' | cut -d. -f1,2)
            print_info "Python version: ${python_version}"
            
            # For Python 3.12, numpy 1.24.2 may not have wheels, try 1.24.4 first
            local current_numpy_version=""
            if $PIP_CMD show numpy &> /dev/null; then
                current_numpy_version=$($PIP_CMD show numpy | grep "^Version:" | awk '{print $2}')
                print_info "Current numpy version: ${current_numpy_version}"
            fi
            
            # Try to install numpy 1.24.4 first (has better Python 3.12 support than 1.24.2)
            if [ -z "$current_numpy_version" ] || [[ "$current_numpy_version" != "1.24"* ]]; then
                print_info "Installing numpy 1.24.4 (compatible with ros2-numpy and Python 3.12)..."
                $PIP_CMD uninstall -y numpy 2>/dev/null || true
                
                # Try numpy 1.24.4 which may have wheels for Python 3.12
                if $PIP_CMD install --break-system-packages --prefer-binary "numpy==1.24.4" 2>&1 | grep -q "Successfully installed"; then
                    print_success "numpy 1.24.4 installed successfully."
                elif $PIP_CMD install --break-system-packages "numpy==1.24.4" 2>&1 | grep -q "Successfully installed"; then
                    print_success "numpy 1.24.4 installed successfully (may have built from source)."
                else
                    print_warning "Failed to install numpy 1.24.4. Will let ros2-numpy handle numpy installation."
                fi
            fi
            
            # Install ros2-numpy - use --no-deps to avoid numpy conflict, then install dependencies manually
            print_info "Installing ros2-numpy..."
            local install_output=$(mktemp)
            
            # First try normal installation
            if $PIP_CMD install --break-system-packages "ros2-numpy" 2>&1 | tee "$install_output" | grep -q "Successfully installed"; then
                print_success "ros2-numpy installed successfully."
                installed_count=$((installed_count + 1))
                rm -f "$install_output"
                continue
            fi
            
            # If failed, try with --no-deps and install manually
            if grep -q "numpy" "$install_output" && (grep -q "build" "$install_output" || grep -q "wheel" "$install_output"); then
                print_warning "ros2-numpy installation failed due to numpy. Trying alternative approach..."
                
                # Install ros2-numpy without dependencies
                if $PIP_CMD install --break-system-packages --no-deps "ros2-numpy" 2>&1 | grep -q "Successfully installed"; then
                    print_success "ros2-numpy installed (without numpy dependency)."
                    print_info "Note: You may need numpy 1.24.x for ros2-numpy to work properly."
                    installed_count=$((installed_count + 1))
                else
                    print_error "Failed to install ros2-numpy even without dependencies."
                    failed_packages+=("${package}")
                    INSTALL_FAILED=true
                fi
            else
                print_error "Failed to install ros2-numpy. Check the error messages above."
                failed_packages+=("${package}")
                INSTALL_FAILED=true
            fi
            
            rm -f "$install_output"
            continue
        fi
        
        # Default installation for other special packages
        print_info "Installing ${package}..."
        if $PIP_CMD install --break-system-packages "${package}" 2>/dev/null; then
            print_success "${package} installed successfully."
            installed_count=$((installed_count + 1))
        else
            print_error "Failed to install ${package}."
            failed_packages+=("${package}")
            INSTALL_FAILED=true
        fi
        echo ""
    done
    
    # Summary
    if [ ${#failed_packages[@]} -gt 0 ]; then
        print_warning "Some special packages failed to install:"
        for pkg in "${failed_packages[@]}"; do
            echo -e "  ${YELLOW}- ${pkg}${NC}"
        done
        echo ""
        return 1
    fi
    
    return 0
}

# Function to verify all dependencies are installed
verify_installation() {
    print_info "Verifying installation..."
    echo ""
    
    local all_installed=true
    local missing_packages=()
    
    for package in "${DEPENDENCIES[@]}"; do
        if check_package_installed "${package}"; then
            print_success "${package} ✓"
        else
            print_error "${package} ✗"
            missing_packages+=("${package}")
            all_installed=false
        fi
    done
    
    echo ""
    
    # Verify Python packages
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        print_info "Verifying Python packages..."
        echo ""
        
        # Determine pip command
        if command -v pip3 &> /dev/null; then
            PIP_CMD="pip3"
        else
            PIP_CMD="python3 -m pip"
        fi
        
        # Verify regular Python packages
        for package in "${PYTHON_PACKAGES[@]}"; do
            # Extract package name without version constraint
            local package_name=$(echo "$package" | sed 's/[<>=!].*//')
            if $PIP_CMD show "${package_name}" &> /dev/null; then
                print_success "${package_name} ✓"
            else
                print_error "${package_name} ✗"
                missing_packages+=("${package}")
                all_installed=false
            fi
        done
        echo ""
        
        # Verify numpy separately (handled by ros2-numpy or special packages)
        if $PIP_CMD show numpy &> /dev/null; then
            local numpy_version=$($PIP_CMD show numpy | grep "^Version:" | awk '{print $2}')
            # Accept numpy 1.24.x (best), < 1.24 (OK), or >= 1.25 (warning but not fail)
            local version_check=$(python3 <<EOF
import sys
try:
    current = tuple(map(int, "${numpy_version}".split('.')))
    if current[0] == 1 and current[1] == 24:
        sys.exit(0)  # Best: 1.24.x
    elif current[0] == 1 and current[1] < 24:
        sys.exit(0)  # OK: < 1.24
    else:
        sys.exit(1)  # >= 1.25, may have issues
except Exception as e:
    sys.exit(1)
EOF
)
            if [ "$version_check" -eq 0 ]; then
                print_success "numpy ${numpy_version} (acceptable) ✓"
            else
                print_warning "numpy ${numpy_version} (>= 1.25, may cause issues with ros2-numpy) ⚠"
                # Don't fail verification, just warn
            fi
        else
            print_warning "numpy not found (may be installed by ros2-numpy) ⚠"
        fi
        echo ""
        
        # Verify special Python packages
        if [ ${#PYTHON_PACKAGES_SPECIAL[@]} -gt 0 ]; then
            print_info "Verifying special Python packages..."
            echo ""
            
            for package in "${PYTHON_PACKAGES_SPECIAL[@]}"; do
                local package_name=$(echo "$package" | sed 's/[<>=!].*//')
                
                # Special handling for tf_transformations - it's optional (transforms3d is alternative)
                if [[ "$package_name" == "tf_transformations" ]]; then
                    if $PIP_CMD show "${package_name}" &> /dev/null || python3 -c "import tf_transformations" 2>/dev/null; then
                        print_success "${package_name} ✓"
                    else
                        # Check if transforms3d is available as alternative
                        if $PIP_CMD show transforms3d &> /dev/null; then
                            print_warning "${package_name} ✗ (but transforms3d is available as alternative) ⚠"
                            # Don't fail verification - transforms3d can be used
                        else
                            print_warning "${package_name} ✗ (optional, transforms3d recommended) ⚠"
                            # Don't fail verification - it's optional
                        fi
                    fi
                # Special handling for numpy - already verified above
                elif [[ "$package_name" == "numpy" ]]; then
                    # Already verified above, skip
                    continue
                else
                    if $PIP_CMD show "${package_name}" &> /dev/null; then
                        print_success "${package_name} ✓"
                    else
                        print_error "${package_name} ✗"
                        missing_packages+=("${package}")
                        all_installed=false
                    fi
                fi
            done
            echo ""
        fi
    fi
    
    if [ "$all_installed" = true ]; then
        return 0
    else
        return 1
    fi
}

# Main function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Common ROS2 Dependencies Installer${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check sudo privileges
    if ! sudo -n true 2>/dev/null; then
        print_info "This script requires sudo privileges. Please enter your password when prompted."
    fi
    
    # Display dependencies to be installed
    print_info "Dependencies to install:"
    for package in "${DEPENDENCIES[@]}"; do
        echo -e "  ${BLUE}- ${package}${NC}"
    done
    
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        echo ""
        print_info "Python packages to install:"
        for package in "${PYTHON_PACKAGES[@]}"; do
            echo -e "  ${BLUE}- ${package}${NC}"
        done
    fi
    
    if [ ${#PYTHON_PACKAGES_SPECIAL[@]} -gt 0 ]; then
        echo ""
        print_info "Special Python packages to install:"
        for package in "${PYTHON_PACKAGES_SPECIAL[@]}"; do
            echo -e "  ${BLUE}- ${package}${NC}"
        done
        echo ""
        print_info "Note: numpy will be installed with version < 1.24 first, then ros2-numpy may upgrade it if needed"
    fi
    echo ""
    
    # Install all dependencies
    if ! install_all_dependencies; then
        INSTALL_FAILED=true
        echo ""
        print_error "Some dependencies failed to install!"
        echo ""
        confirm_exit 1 "Dependency installation completed with errors!"
    fi
    
    # Install Python packages
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Installing Python Packages${NC}"
        echo -e "${BLUE}========================================${NC}"
        echo ""
        
        if ! install_python_packages; then
            INSTALL_FAILED=true
            echo ""
            print_warning "Some Python packages failed to install, but continuing..."
            echo ""
        fi
    fi
    
    # Install special Python packages (after regular packages)
    if [ ${#PYTHON_PACKAGES_SPECIAL[@]} -gt 0 ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Installing Special Python Packages${NC}"
        echo -e "${BLUE}========================================${NC}"
        echo ""
        
        if ! install_special_python_packages; then
            INSTALL_FAILED=true
            echo ""
            print_warning "Some special Python packages failed to install, but continuing..."
            echo ""
        fi
    fi
    
    # Verify installation
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Verifying Installation${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    if verify_installation; then
        INSTALL_SUCCESS=true
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}    INSTALLATION SUCCESSFUL!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        print_success "All dependencies have been installed successfully!"
        echo ""
        print_info "Installed apt packages:"
        for package in "${DEPENDENCIES[@]}"; do
            echo -e "  ${GREEN}✓ ${package}${NC}"
        done
        
        if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
            echo ""
            print_info "Installed Python packages:"
            for package in "${PYTHON_PACKAGES[@]}"; do
                echo -e "  ${GREEN}✓ ${package}${NC}"
            done
        fi
        echo ""
        confirm_exit 0 "All dependencies have been installed successfully!"
    else
        INSTALL_FAILED=true
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}    VERIFICATION FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        print_error "Some dependencies could not be verified. Please check the logs above."
        echo ""
        confirm_exit 1 "Dependency verification failed!"
    fi
}

# Run main function
main
