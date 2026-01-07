#!/bin/bash

# Script to find backend server in LAN and update /etc/hosts
# This script scans the ARP table, checks health endpoint on port 8000,
# and updates /etc/hosts with lidar.tm

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Wait for user to press Enter before exiting
wait_for_exit() {
    echo ""
    read -p "Press Enter to exit..."
    exit 1
}

# Check if running as root, if not, request sudo and re-run
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        print_info "Requesting sudo privileges..."
        exec sudo "$0" "$@"
    fi
}

# Get local IP and subnet (prefer real LAN interface, not docker/bridge)
get_local_network() {
    # Try to get default interface (IPv4)
    local interface
    interface=$(ip -4 route show default 2>/dev/null | awk '{print $5}' | head -n1)

    # If default interface is docker/bridge or empty, pick first global non-virtual iface
    if [ -z "$interface" ] || [[ "$interface" == "docker0" ]] || [[ "$interface" == br-* ]] || [[ "$interface" == veth* ]]; then
        interface=$(ip -o -4 addr show scope global 2>/dev/null | awk '!/docker0/ && !/br-/ && !/veth/ {print $2; exit}')
    fi
    
    if [ -z "$interface" ]; then
        print_error "Could not determine network interface"
        wait_for_exit
    fi
    
    # Get local IP on that interface
    local_ip=$(ip -o -4 addr show dev "$interface" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)
    
    if [ -z "$local_ip" ]; then
        print_error "Could not determine local IP address"
        wait_for_exit
    fi
    
    # Extract subnet (e.g., 192.168.1.0/24)
    subnet=$(ip -o -4 addr show dev "$interface" 2>/dev/null | awk '{print $4}' | head -n1)
    
    print_info "Using interface: $interface"
    print_info "Local IP: $local_ip"
    print_info "Subnet: $subnet"
}

# Get list of IPs from ARP table
get_arp_ips() {
    print_info "Scanning ARP table for active hosts..."
    
    # Read ARP table and extract IPs (excluding localhost and broadcast)
    arp_ips=$(arp -a 2>/dev/null | grep -v "incomplete" | awk '{print $2}' | tr -d '()' | grep -v "^$" | sort -u)
    
    # Alternative: use /proc/net/arp if arp -a doesn't work
    if [ -z "$arp_ips" ]; then
        print_info "Trying /proc/net/arp..."
        arp_ips=$(cat /proc/net/arp 2>/dev/null | awk 'NR>1 {print $1}' | grep -v "^0.0.0.0$" | sort -u)
    fi
    
    if [ -z "$arp_ips" ]; then
        print_warning "No IPs found in ARP table. Trying to ping local subnet..."
        # Fallback: try to discover by pinging common IPs in subnet
        local base_ip=$(echo "$local_ip" | cut -d. -f1-3)
        arp_ips=""
        for i in {1..254}; do
            ip="${base_ip}.${i}"
            if [ "$ip" != "$local_ip" ]; then
                # Quick ping to populate ARP table
                ping -c 1 -W 1 "$ip" >/dev/null 2>&1 &
            fi
        done
        sleep 2
        arp_ips=$(arp -a 2>/dev/null | grep -v "incomplete" | awk '{print $2}' | tr -d '()' | grep -v "^$" | sort -u)
    fi
    
    if [ -z "$arp_ips" ]; then
        print_error "Could not find any IPs in LAN"
        wait_for_exit
    fi
    
    ip_count=$(echo "$arp_ips" | wc -l)
    print_info "Found $ip_count IP(s) in ARP table"
}

# Check health endpoint on an IP
check_backend_health() {
    local ip=$1
    local timeout=2
    
    # Try to connect via nginx reverse proxy first (port 80 with Host header)
    local response=$(curl -s --max-time $timeout -H "Host: backend.lidar.tm" "http://${ip}/health" 2>/dev/null)
    
    # If that fails, try direct connection to port 8000
    if [ $? -ne 0 ] || [ -z "$response" ]; then
        response=$(curl -s --max-time $timeout "http://${ip}:8000/health" 2>/dev/null)
    fi
    
    if [ $? -ne 0 ] || [ -z "$response" ]; then
        return 1
    fi
    
    # Check if response contains "livo-backend" service name or "healthy" status
    if echo "$response" | grep -q '"service":\s*"livo-backend"' || \
       (echo "$response" | grep -q '"status":\s*"healthy"' && echo "$response" | grep -q '"service"'); then
        return 0
    fi
    
    return 1
}

# Find backend server
find_backend() {
    print_info "Searching for backend server..."
    
    backend_ip=""
    checked=0
    
    for ip in $arp_ips; do
        # Skip localhost and invalid IPs
        if [ "$ip" == "127.0.0.1" ] || [ "$ip" == "::1" ] || [ -z "$ip" ]; then
            continue
        fi
        
        checked=$((checked + 1))
        print_info "Checking $ip... ($checked/$ip_count)"
        
        if check_backend_health "$ip"; then
            backend_ip="$ip"
            print_success "Found backend server at $ip"
            return 0
        fi
    done
    
    print_error "Backend server not found in LAN"
    return 1
}

# Check if backend is running on this machine (using local IP)
check_local_backend() {
    print_info "Checking backend on this machine ($local_ip)..."

    if check_backend_health "$local_ip"; then
        backend_ip="$local_ip"
        print_success "Backend server is running on this machine ($local_ip)"
        return 0
    fi

    print_info "No healthy backend detected on this machine. Falling back to LAN discovery..."
    return 1
}

# Backup hosts file
backup_hosts() {
    local timestamp=$(date +%Y%m%d_%H%M%S)
    local backup_file="/etc/hosts.backup.$timestamp"
    
    cp /etc/hosts "$backup_file"
    print_info "Backed up /etc/hosts to $backup_file"
}

# Update /etc/hosts
update_hosts() {
    local ip=$1
    
    print_info "Updating /etc/hosts..."
    
    # Remove old entries for backend.lidar.tm, frontend.lidar.tm and lidar.tm
    sed -i '/backend\.lidar\.tm/d' /etc/hosts
    sed -i '/frontend\.lidar\.tm/d' /etc/hosts
    sed -i '/lidar\.tm/d' /etc/hosts
    
    # Add new entries for both domains pointing to same IP
    echo "$ip    frontend.lidar.tm" >> /etc/hosts
    echo "$ip    backend.lidar.tm" >> /etc/hosts
    
    print_success "Updated /etc/hosts:"
    print_info "  frontend.lidar.tm -> $ip"
    print_info "  backend.lidar.tm -> $ip"
}

# Main function
main() {
    echo ""
    echo "=========================================="
    echo "  LAN Backend Discovery Script"
    echo "=========================================="
    echo ""
    
    # Check sudo (pass all arguments)
    check_sudo "$@"
    
    # Get network info first so we know local_ip
    get_local_network
    
    # First, try to find backend on this machine (using local_ip)
    if check_local_backend; then
        # Backup hosts file
        backup_hosts
        
        # Update hosts file
        update_hosts "$backend_ip"
        
        echo ""
        echo "=========================================="
        print_success "Setup complete!"
        echo "=========================================="
        echo ""
        echo "You can now access:"
        echo "  Frontend: http://frontend.lidar.tm"
        echo "  Backend:  http://backend.lidar.tm"
        echo ""
        exit 0
    fi
    
    # If not on this machine, try to discover in LAN
    # Get ARP IPs
    get_arp_ips
    
    # Find backend
    if ! find_backend; then
        wait_for_exit
    fi
    
    # Backup hosts file
    backup_hosts
    
    # Update hosts file
    update_hosts "$backend_ip"
    
    echo ""
    echo "=========================================="
    print_success "Setup complete!"
    echo "=========================================="
    echo ""
    echo "You can now access:"
    echo "  Frontend: http://frontend.lidar.tm"
    echo "  Backend:  http://backend.lidar.tm"
    echo ""
}

# Run main function with all arguments
main "$@"

