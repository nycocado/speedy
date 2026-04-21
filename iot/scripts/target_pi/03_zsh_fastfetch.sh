#!/bin/bash
# ==============================================================================
# SPEEDY IOT SYSTEM
# Script:  03_zsh_fastfetch.sh
# Purpose: Shell Environment Setup (Zsh, Oh My Zsh, Fastfetch, Plugins)
# ==============================================================================
set -e

echo "====================================================================="
echo "[SPEEDY-PI] STAGE 3: Shell & Productivity Setup"
echo "====================================================================="

echo "[INFO] Installing base dependencies (Zsh, Git, Curl, Wget)..."
sudo apt install zsh git curl wget -y

echo "[INFO] Installing Fastfetch (ARM64 deb package)..."
wget https://github.com/fastfetch-cli/fastfetch/releases/latest/download/fastfetch-linux-aarch64.deb -O /tmp/fastfetch-linux-aarch64.deb
sudo dpkg -i /tmp/fastfetch-linux-aarch64.deb
rm /tmp/fastfetch-linux-aarch64.deb

echo "[INFO] Installing Oh My Zsh (unattended mode)..."
if [ ! -d "$HOME/.oh-my-zsh" ]; then
    sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
fi

echo "[INFO] Downloading Powerlevel10k theme and Zsh plugins..."
ZSH_CUSTOM=${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}
[ ! -d "$ZSH_CUSTOM/themes/powerlevel10k" ] && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "$ZSH_CUSTOM/themes/powerlevel10k"
[ ! -d "$ZSH_CUSTOM/plugins/zsh-autosuggestions" ] && git clone https://github.com/zsh-users/zsh-autosuggestions "$ZSH_CUSTOM/plugins/zsh-autosuggestions"
[ ! -d "$ZSH_CUSTOM/plugins/zsh-syntax-highlighting" ] && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git "$ZSH_CUSTOM/plugins/zsh-syntax-highlighting"
[ ! -d "$ZSH_CUSTOM/plugins/zsh-completions" ] && git clone https://github.com/zsh-users/zsh-completions "$ZSH_CUSTOM/plugins/zsh-completions"

echo "[INFO] Setting Zsh as the default shell for the current user..."
sudo chsh -s $(which zsh) $USER

echo "---------------------------------------------------------------------"
echo "[SUCCESS] Stage 3 complete. Shell environment configured."
echo "---------------------------------------------------------------------"
