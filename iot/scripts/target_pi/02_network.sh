#!/bin/bash
# ==============================================================================
# SPEEDY IOT SYSTEM
# Script:  02_network.sh
# Purpose: Network and Remote Access Configuration (SSH/mDNS/Ethernet)
# ==============================================================================
set -e

echo "====================================================================="
echo "[SPEEDY-PI] STAGE 2: Network Configuration"
echo "====================================================================="

echo "[INFO] Installing and enabling Avahi Daemon (mDNS for speedy.local)..."
sudo apt install avahi-daemon -y
sudo systemctl enable --now avahi-daemon

echo "[INFO] Ensuring SSH service is enabled and running..."
sudo systemctl enable --now ssh

echo "[INFO] Configuring Ethernet (eth0) for dynamic DHCP via Netplan..."
cat <<EOF | sudo tee /etc/netplan/99-eth0.yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
EOF
sudo netplan apply

echo "---------------------------------------------------------------------"
echo "[SUCCESS] Stage 2 complete. Device accessible via 'speedy.local' or IP."
echo "---------------------------------------------------------------------"
