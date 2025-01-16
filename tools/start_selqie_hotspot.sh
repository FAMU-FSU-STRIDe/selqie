#!/bin/bash

# Script to start a mobile hotspot on Ubuntu 22.04

# Function to display an error message and exit
die() {
    echo "$1" >&2
    exit 1
}

# Default hotspot configuration
SSID="SELQIE Network"
PASSWORD=""
BAND="bg"  # Options: 'a' for 5GHz, 'bg' for 2.4GHz
CON_NAME="Hotspot"

# Ensure nmcli is available
command -v nmcli > /dev/null 2>&1 || die "nmcli not found. Please install NetworkManager."

# Check if Wi-Fi device is available
WIFI_DEVICE=$(nmcli device status | grep wifi | awk 'NR==1 {print $1}')
if [ -z "$WIFI_DEVICE" ]; then
    die "No Wi-Fi device found. Ensure your Wi-Fi adapter is enabled."
fi

# Delete any existing connection with the same name
nmcli connection delete "$CON_NAME" > /dev/null 2>&1

# Create the hotspot connection
nmcli connection add type wifi ifname "$WIFI_DEVICE" con-name "$CON_NAME" autoconnect yes ssid "$SSID" || die "Failed to create hotspot."

# Configure the hotspot settings
nmcli connection modify "$CON_NAME" \
    802-11-wireless.mode ap \
    802-11-wireless.band "$BAND" \
    ipv4.method shared \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD" || die "Failed to configure hotspot settings."

# Activate the hotspot
nmcli connection up "$CON_NAME" || die "Failed to start hotspot."

echo "Hotspot started successfully!"
echo "SSID: $SSID"
echo "Password: $PASSWORD"
