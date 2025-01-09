# Jetson AGX Orin Setup

## Hardware Info

* NVIDIA Jetson AGX Orin 64GB Developer Kit
* SAMSUNG 990 PRO with Heatsink - PCIe 4.0 NVMe M.2 SSD

## Flashing

### Resources
* [Download NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager)
* [Install Jetson Software with SDK Manager](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html)

### Procedure

1. Open SDK Manager on Host Machine (Ubuntu 22.04 x86_64)
2. Power Jetson in Recovery Mode
   - Press the Power button (left) and press the Reset button (right), all while holding down the Recovery button (middle)
3. Connect Jetson to Host Machine by a USB-C cable
4. Select Jetson AGX Orin 64GB Board in SDK Manager
5. Select all additional SDKS
6. Wait for Download and Install to finish
   - Some configuration prompts will appear during the installation
   ```
   Username: selqie
   Password: *Lab password*
   ```
7. When complete, connect the Jetson to the network via Ethernet or WiFi and note the IP address

## Install Software

### 1. SSH into the Jetson
```
ssh selqie@<ip address>
```

### 2. Update and Upgrade
```
sudo apt update && sudo apt upgrade
```

### 3. Generate SSH Key on the Jetson
1. [Generate new SSH Key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
2. [Add SSH Key to Github](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

### 4. Install SELQIE Repository
```
mkdir ~/selqie_ws
git clone git@github.com:FAMU-FSU-STRIDe/selqie src
```

### 5. Run Installation Bash Script
```
cd ~/selqie_ws/src/tools && ./install.sh
```

- When adding new software, make sure to add the installation commands to this script

### Troubleshooting
I ran into an issue with OpenCV, where versions 4.5 and 4.8 were installed causing conflicts. This gave errors when trying to run the camera disparity nodes.

Solution:
```
sudo apt purge *libopencv*
sudo apt remove opencv-licenses
sudo apt install libopencv-dev=4.5.*
sudo apt-mark hold libopencv-dev
```
Then re-run the install script