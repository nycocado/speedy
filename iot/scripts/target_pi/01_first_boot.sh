#!/bin/bash
# ==============================================================================
# SPEEDY IOT SYSTEM
# Script:  01_first_boot.sh
# Purpose: System Base Update (Ubuntu Server 24.04)
# ==============================================================================
set -e

echo "====================================================================="
echo "[SPEEDY-PI] STAGE 1: Preparing Base System"
echo "====================================================================="

echo "[INFO] Disabling unattended-upgrades to prevent apt locks..."
sudo systemctl stop unattended-upgrades || true
sudo systemctl disable unattended-upgrades || true

echo "[INFO] Updating package lists and performing full system upgrade..."
sudo apt update
sudo apt full-upgrade -y

echo "[INFO] Updating Raspberry Pi EEPROM firmware..."
sudo apt install rpi-eeprom -y
sudo rpi-eeprom-update

echo "[INFO] Removing unused dependencies..."
sudo apt autoremove -y

echo "---------------------------------------------------------------------"
echo "[SUCCESS] Stage 1 complete. System is up to date."
echo "---------------------------------------------------------------------"
