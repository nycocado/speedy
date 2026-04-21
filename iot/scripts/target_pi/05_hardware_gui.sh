#!/bin/bash
# ==============================================================================
# SPEEDY IOT SYSTEM
# Script:  05_hardware_gui.sh
# Purpose: Hardware Permissions, Foxglove Bridge, and GUI Installation
# ==============================================================================
set -e

echo "====================================================================="
echo "[SPEEDY-PI] STAGE 5: Hardware & Telemetry Setup"
echo "====================================================================="

echo "[INFO] Adding user to hardware groups (dialout, video, i2c)..."
sudo usermod -aG dialout,video,i2c $USER

echo "[INFO] Installing Foxglove Bridge for remote telemetry..."
sudo apt install ros-jazzy-foxglove-bridge -y

echo "---------------------------------------------------------------------"
echo "[PROMPT] Optional Graphical Interface Installation"
echo "---------------------------------------------------------------------"
echo -n "Install XFCE4 desktop environment (xubuntu-desktop)? [y/N]: "
read -r response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]] || [[ "$response" =~ ^([sS][iI][mM]|[sS])$ ]]; then
    echo "[INFO] Installing Xubuntu Desktop environment..."
    sudo apt install xubuntu-desktop -y
    
    echo "[INFO] Configuring .xinitrc for startx..."
    echo "exec startxfce4" > ~/.xinitrc
    
    sudo systemctl set-default multi-user.target
    echo "[INFO] GUI installed. You can start it locally using 'startx'."
else
    echo "[INFO] GUI installation skipped."
fi

echo "---------------------------------------------------------------------"
echo "[SUCCESS] SETUP COMPLETE."
echo "[ACTION REQUIRED] Reboot the Raspberry Pi (sudo reboot) to apply hardware group permissions."
echo "---------------------------------------------------------------------"
