#!/bin/bash

# Speedy Hotspot Configuration
SSID="SpeedyBot_AP"
PASSWORD="speedyrobot"
INTERFACE="wlan0"
CON_NAME="SpeedyHotspot"

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (use sudo)"
  exit 1
fi

if ! command -v nmcli &> /dev/null; then
    echo "Error: NetworkManager (nmcli) is not installed."
    exit 1
fi

case "$1" in
    on|start)
        echo "Starting hotspot ($SSID) via NetworkManager..."
        
        # Ensure wifi is unblocked
        if command -v rfkill &> /dev/null; then
            rfkill unblock wifi
        fi
        
        # Check if connection already exists
        if nmcli con show "$CON_NAME" > /dev/null 2>&1; then
            nmcli con modify "$CON_NAME" connection.autoconnect yes
            nmcli con up "$CON_NAME"
        else
            # 5GHz (band a, ch36) to match the hotspot.nmconnection profile and give
            # the camera decent bandwidth. 2.4GHz/g (band bg) capped the link and stuttered.
            nmcli dev wifi hotspot ifname "$INTERFACE" ssid "$SSID" password "$PASSWORD" con-name "$CON_NAME" band a channel 36
            nmcli con modify "$CON_NAME" connection.autoconnect yes
        fi

        # Attempt to keep eth0 up if it exists
        if nmcli con show "eth0-connection" > /dev/null 2>&1; then
             nmcli con up "eth0-connection" > /dev/null 2>&1 || true
        fi

        if [ $? -eq 0 ]; then
            echo "Hotspot successfully activated."
            echo "Network: $SSID"
            echo "Password: $PASSWORD"
            
            # Wait a moment for IP to be assigned (max 5 seconds)
            echo -n "Waiting for IP assignment..."
            for i in {1..5}; do
                ROBOT_IP=$(nmcli -g IP4.ADDRESS dev show "$INTERFACE" 2>/dev/null | grep -v '^$' | head -n 1 | cut -d/ -f1)
                if [ ! -z "$ROBOT_IP" ]; then 
                    echo " Done."
                    break; 
                fi
                echo -n "."
                sleep 1
            done
            echo ""
            echo "Robot IP (Wi-Fi): ${ROBOT_IP:-10.42.0.1 (default)}"
        else
            echo "Failed to start hotspot."
        fi
        ;;

    off|stop)
        echo "Stopping hotspot..."
        if nmcli con show --active | grep -q "$CON_NAME"; then
            nmcli con down "$CON_NAME"
            echo "Hotspot successfully stopped."
        else
            echo "Hotspot is not active."
        fi
        # Disable auto-connect so it doesn't start on boot
        if nmcli con show "$CON_NAME" > /dev/null 2>&1; then
            nmcli con modify "$CON_NAME" connection.autoconnect no
        fi
        ;;

    status)
        if nmcli con show --active | grep -q "$CON_NAME"; then
            echo "Hotspot status: ACTIVE"
            ROBOT_IP=$(nmcli -g IP4.ADDRESS dev show "$INTERFACE" 2>/dev/null | grep -v '^$' | head -n 1 | cut -d/ -f1)
            echo "IP Address: ${ROBOT_IP}"
        else
            echo "Hotspot status: INACTIVE"
        fi
        ;;

    *)
        echo "Usage: speedy-hotspot {on|off|status}"
        echo "Example: sudo speedy-hotspot on"
        exit 1
        ;;
esac