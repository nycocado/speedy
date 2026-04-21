#!/bin/bash
# ==============================================================================
# SPEEDY HOST TOOLS
# Script:  deploy_setup_to_pi.sh
# Purpose: Automates the deployment of setup scripts and dotfiles to the Target Pi
# ==============================================================================
set -e

# Default Configuration
PI_USER="speedy"
PI_HOST="speedy.local"

echo "====================================================================="
echo "[SPEEDY-HOST] DEPLOYMENT UTILITY"
echo "====================================================================="
echo "Note: If connected via direct Ethernet, you may use the assigned IP."
read -p "Enter Raspberry Pi Address [$PI_HOST]: " input_host
PI_HOST=${input_host:-$PI_HOST}

echo "[INFO] Establishing connection to $PI_USER@$PI_HOST..."

echo "[INFO] STEP 1: Deploying SSH Public Key for passwordless authentication..."
ssh-copy-id "$PI_USER@$PI_HOST" || true

echo "[INFO] STEP 2: Transferring Setup Scripts to Target root directory..."
scp ../target_pi/*.sh "$PI_USER@$PI_HOST:~/"

echo "[INFO] STEP 3: Transferring Dotfiles (Zsh/P10k) to Target root directory..."
scp ../target_pi/dotfiles/.zshrc "$PI_USER@$PI_HOST:~/.zshrc"
scp ../target_pi/dotfiles/.p10k.zsh "$PI_USER@$PI_HOST:~/.p10k.zsh"

echo "---------------------------------------------------------------------"
echo "[SUCCESS] Deployment completed successfully."
echo "---------------------------------------------------------------------"
echo "Next Steps:"
echo "1. Connect to the Target: ssh $PI_USER@$PI_HOST"
echo "2. Execute scripts sequentially (e.g., ./01_first_boot.sh)"
echo "====================================================================="
