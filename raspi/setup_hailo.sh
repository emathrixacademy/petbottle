#!/bin/bash

#############################################
# Raspberry Pi 5 + Hailo-8L Setup Script
# This script sets up a fresh Raspberry Pi 5
# with Hailo-8L AI accelerator (26 TOPS)
#############################################

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Raspberry Pi 5
check_hardware() {
    log_info "Checking hardware compatibility..."
    
    if ! grep -q "Raspberry Pi 5" /proc/cpuinfo && ! grep -q "BCM2712" /proc/cpuinfo; then
        log_error "This script is designed for Raspberry Pi 5"
        exit 1
    fi
    
    log_info "Raspberry Pi 5 detected ✓"
}

# Update system
update_system() {
    log_info "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y
    log_info "System updated ✓"
}

# Install essential packages
install_essentials() {
    log_info "Installing essential packages..."
    sudo apt install -y \
        git \
        cmake \
        build-essential \
        pkg-config \
        wget \
        curl \
        python3-pip \
        python3-dev \
        python3-venv \
        libopencv-dev \
        python3-opencv \
        v4l-utils
    
    log_info "Essential packages installed ✓"
}

# Enable camera and necessary interfaces
enable_interfaces() {
    log_info "Enabling required interfaces..."
    
    # Enable camera
    sudo raspi-config nonint do_camera 0
    
    # Enable I2C
    sudo raspi-config nonint do_i2c 0
    
    # Enable SPI
    sudo raspi-config nonint do_spi 0
    
    log_info "Interfaces enabled ✓"
}

# Install Hailo software
install_hailo() {
    log_info "Installing Hailo software stack..."
    
    # Add Hailo repository
    wget -qO - https://hailo-csdata.s3.eu-west-2.amazonaws.com/hailo-ppa/hailo-public.gpg.key | sudo apt-key add -
    echo "deb https://hailo-csdata.s3.eu-west-2.amazonaws.com/hailo-ppa/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/hailo.list
    
    sudo apt update
    
    # Install Hailo packages
    sudo apt install -y hailort
    sudo apt install -y python3-hailo-platform
    
    # Verify installation
    if hailortcli fw-control identify &>/dev/null; then
        log_info "Hailo-8L detected and verified ✓"
    else
        log_warn "Hailo device not detected. Please check hardware connection."
    fi
}

# Setup Python virtual environment
setup_python_env() {
    log_info "Setting up Python virtual environment..."
    
    cd ~
    python3 -m venv hailo-env
    source hailo-env/bin/activate
    
    pip install --upgrade pip
    pip install numpy opencv-python pillow
    pip install hailo-platform
    
    deactivate
    
    log_info "Python environment created at ~/hailo-env ✓"
}

# Install Hailo Tappas (optional but recommended)
install_tappas() {
    log_info "Would you like to install Hailo TAPPAS (Hailo's application suite)? [y/N]"
    read -r response
    
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        log_info "Installing TAPPAS..."
        
        cd ~
        git clone https://github.com/hailo-ai/tappas.git
        cd tappas
        ./install.sh
        
        log_info "TAPPAS installed ✓"
    else
        log_info "Skipping TAPPAS installation"
    fi
}

# Configure static IP for ESP32 robot WiFi network
configure_robot_wifi() {
    log_info "Configuring static IP for PetBottle_Robot WiFi..."

    # Create NetworkManager connection with static IP 192.168.4.4
    sudo nmcli connection add \
        type wifi \
        con-name "PetBottle_Robot" \
        ssid "PetBottle_Robot" \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "petbottle123" \
        ipv4.method manual \
        ipv4.addresses "192.168.4.4/24" \
        ipv4.gateway "192.168.4.1" \
        connection.autoconnect yes \
        connection.autoconnect-priority 100

    log_info "Static IP 192.168.4.4 configured for PetBottle_Robot WiFi ✓"
    log_info "ESP32 dashboard: http://192.168.4.1"
    log_info "Pi video stream: http://192.168.4.4:5000/video_feed"
}

# Configure memory split
configure_memory() {
    log_info "Configuring GPU memory split..."
    
    # Set GPU memory to 256MB (recommended for AI workloads with camera)
    sudo raspi-config nonint do_memory_split 256
    
    log_info "GPU memory configured ✓"
}

# Create test script
create_test_script() {
    log_info "Creating test script..."
    
    cat > ~/test_hailo.py << 'EOF'
#!/usr/bin/env python3
"""Simple script to test Hailo-8L connection"""

try:
    from hailo_platform import VDevice, HailoSchedulingAlgorithm
    
    print("Testing Hailo-8L connection...")
    
    with VDevice() as device:
        print(f"✓ Hailo device detected: {device.get_device_info()}")
        print(f"✓ Device architecture: {device.get_architecture()}")
        print("✓ Hailo-8L is working correctly!")
        
except Exception as e:
    print(f"✗ Error: {e}")
    print("Please check your Hailo installation and hardware connection.")
EOF
    
    chmod +x ~/test_hailo.py
    log_info "Test script created at ~/test_hailo.py ✓"
}

# Create activation helper
create_activation_helper() {
    log_info "Creating environment activation helper..."
    
    cat >> ~/.bashrc << 'EOF'

# Hailo environment activation alias
alias hailo='source ~/hailo-env/bin/activate'
EOF
    
    log_info "Added 'hailo' alias to activate environment ✓"
}

# Final instructions
print_instructions() {
    echo ""
    echo "=========================================="
    log_info "Setup Complete!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "1. Reboot your Raspberry Pi: sudo reboot"
    echo "2. After reboot, activate the Hailo environment:"
    echo "   source ~/hailo-env/bin/activate"
    echo "   (or just type: hailo)"
    echo "3. Test the Hailo device:"
    echo "   python3 ~/test_hailo.py"
    echo "4. Check device with: hailortcli fw-control identify"
    echo ""
    echo "Documentation:"
    echo "- Hailo docs: https://hailo.ai/developer-zone/"
    echo "- Python environment: ~/hailo-env/"
    echo "- TAPPAS (if installed): ~/tappas/"
    echo ""
}

# Main execution
main() {
    log_info "Starting Raspberry Pi 5 + Hailo-8L Setup..."
    echo ""
    
    check_hardware
    update_system
    install_essentials
    enable_interfaces
    install_hailo
    setup_python_env
    install_tappas
    configure_robot_wifi
    configure_memory
    create_test_script
    create_activation_helper
    
    print_instructions
    
    log_warn "A reboot is required to complete the setup."
    echo -n "Would you like to reboot now? [y/N] "
    read -r response
    
    if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        log_info "Rebooting..."
        sudo reboot
    else
        log_info "Please reboot manually when ready: sudo reboot"
    fi
}

# Run main function
main
