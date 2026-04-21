# Speedy: Guia de Configuração Raspberry Pi 4

Preparação do Ubuntu Server 24.04 (Runtime OS). Siga a ordem rigorosamente.

Para facilitar, todos os blocos de comandos abaixo estão também disponíveis como *scripts* executáveis na pasta `iot/scripts/target_pi/`.

**[Na Máquina Host]**
Antes de começar, envie todos os scripts e dotfiles para a raiz (`~`) do Raspberry Pi. Criámos um script de "Deploy" para automatizar todo este processo inicial a partir do seu PC:

```bash
cd iot/scripts/host/
./deploy_setup_to_pi.sh
```

A partir daí, pode optar por entrar via SSH e executar os scripts (`./01_first_boot.sh`, etc.) ou copiar e colar os comandos manualmente deste guia.

## 1. Primeiros Passos (First Boot)

**[No Raspberry Pi]** (Script: `01_first_boot.sh`)

```bash
# Desativar unattended-upgrades
sudo systemctl stop unattended-upgrades
sudo systemctl disable unattended-upgrades

# Atualizar sistema base
sudo apt update && sudo apt full-upgrade -y
sudo apt install rpi-eeprom -y
sudo rpi-eeprom-update
sudo apt autoremove -y
```

## 2. Acesso Remoto e Redes

### 2.1. SSH e mDNS (speedy.local)

**[No Raspberry Pi]** (Script: `02_network.sh`)

```bash
sudo apt install avahi-daemon -y
sudo systemctl enable --now avahi-daemon
sudo systemctl enable --now ssh
```

### 2.2. Ativar a Porta Ethernet (Para ligações por cabo)

**[No Raspberry Pi]**
O Netplan do Ubuntu Server ignora a porta de rede (`eth0`) se ela não estiver explicitamente declarada. Para o Pi aceitar o IP do Fedora ao ligar por cabo (sem apagar o Wi-Fi), basta criar um ficheiro adicional:

```bash
cat <<EOF | sudo tee /etc/netplan/99-eth0.yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
      optional: true
EOF
sudo netplan apply
```

### 2.3. Chaves SSH e Acesso pelo Fedora

**[No Fedora: Descobrir o IP e Conectar]**
Firewalls podem bloquear o mDNS (`.local`) na rede cabeada, exigindo o acesso direto pelo IP gerado pelo Fedora.

**Comando Primário** (via mDNS):
```bash
ssh speedy@speedy.local
```

**Comando Secundário** (se falhar, procurar o IP e aceder diretamente):
```bash
arp -a | grep 10.42.0
ssh speedy@10.42.0.x
```

Por fim, copie a sua chave para login sem password:
```bash
ssh-copy-id speedy@speedy.local
```

## 3. Zsh e Produtividade

### 3.1. Instalação Oh My Zsh

**[No Raspberry Pi]** (Script: `03_zsh_fastfetch.sh`)

```bash
sudo apt install zsh git curl wget -y

# Instalar Fastfetch (via deb aarch64 para Ubuntu 24.04)
wget https://github.com/fastfetch-cli/fastfetch/releases/latest/download/fastfetch-linux-aarch64.deb -O /tmp/fastfetch-linux-aarch64.deb
sudo dpkg -i /tmp/fastfetch-linux-aarch64.deb
rm /tmp/fastfetch-linux-aarch64.deb

# Instalar Oh My Zsh (não-interativo)
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Instalar Tema Powerlevel10k e Plugins
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k"
git clone https://github.com/zsh-users/zsh-autosuggestions "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-autosuggestions"
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting"
git clone https://github.com/zsh-users/zsh-completions "${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/plugins/zsh-completions"

# Definir o Zsh como a shell padrão do utilizador
sudo chsh -s $(which zsh) $USER
```

### 3.2. Sincronizar Configurações

**[Na Máquina Host]**

Copie os ficheiros de configuração base (`.zshrc` e `.p10k.zsh`) incluídos neste repositório para o Raspberry Pi.

```bash
scp iot/scripts/target_pi/dotfiles/.zshrc speedy@speedy.local:~/.zshrc
scp iot/scripts/target_pi/dotfiles/.p10k.zsh speedy@speedy.local:~/.p10k.zsh
```

> **Nota:** O ficheiro `.zshrc` fornecido já inclui a variável `export TERM=xterm-256color` para resolver problemas de compatibilidade com emuladores de terminal modernos.

## 4. ROS 2 Jazzy Jalisco (Runtime)

### 4.1. Repositórios e Instalação Base

**[No Raspberry Pi]** (Script: `04_ros2_jazzy.sh`)

```bash
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-jazzy-ros-base ros-dev-tools python3-rosdep -y
sudo rosdep init && rosdep update
```

## 5. Hardware e GUI

### 5.1. Permissões e Telemetria

**[No Raspberry Pi]** (Script: `05_hardware_gui.sh`)

```bash
# Permissões de hardware
sudo usermod -aG dialout,video,i2c $USER

# Foxglove Bridge
sudo apt install ros-jazzy-foxglove-bridge -y
```

### 5.2. Interface Gráfica Opcional

**[No Raspberry Pi]**

```bash
sudo apt install xubuntu-desktop -y
echo "exec startxfce4" > ~/.xinitrc
sudo systemctl set-default multi-user.target
# Iniciar GUI: startx
```
