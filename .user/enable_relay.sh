#!/bin/bash

# Check for root privileges
if [ "$EUID" -ne 0 ]; then 
  echo "Please run as root (sudo ./enable_relay.sh)"
  exit
fi

echo "--- Enabling ROS2 Wireless Relay ---"

# Enable IP Forwarding in the Kernel
echo "[1/3] Enabling IP Forwarding..."
sysctl -w net.ipv4.ip_forward=1

# Open Firewall Forwarding
echo "[2/3] Setting FORWARD policy to ACCEPT..."
iptables -P FORWARD ACCEPT

# Enable Masquerade (NAT)
echo "[3/3] Adding NAT Masquerade rule..."
iptables -t nat -C POSTROUTING -o wlan0 -j MASQUERADE 2>/dev/null
if [ $? -eq 0 ]; then
    echo "      (Rule already exists, skipping)"
else
    iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
    echo "      (Rule added)"
fi

echo "--- Relay is ACTIVE. ---"
