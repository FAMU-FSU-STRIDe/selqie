import Jetson.GPIO as GPIO
import os
import time

# To run this in the background as a service:
# 1. ensure the script has executable permissions. From the script location:
# chmod +x water_shutdown.py
# 2. add the following text to /etc/systemd/system/water_shutdown.service

############################################################
# [Unit]
# Description=Water Leakage Shutdown Service
# After=multi-user.target

# [Service]
# Type=simple
# ExecStart=/usr/bin/python3 /path/to/your/water_shutdown.py
# Restart=on-failure

# [Install]
# WantedBy=multi-user.target

################################################################
# 3. Enable and start the service
# sudo systemctl enable water_shutdown.service
# sudo systemctl start water_shutdown.service




# Pin Definitions
sensor_pin_front = 22  # Adjust this to the GPIO pin you're using for the front
sensor_pin_back = 13  # Adjust this to the GPIO pin you're using for the back

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor_pin_front, GPIO.IN)
GPIO.setup(sensor_pin_back, GPIO.IN)


def shutdown():
    print("Water detected! Shutting down...")
    os.system("sudo shutdown -h now")

try:
    while True:
        if GPIO.input(sensor_pin_front) == GPIO.HIGH:
            print("Leak in FRONT HIPS Detected!")
            shutdown()
        elif GPIO.input(sensor_pin_back) == GPIO.HIGH:
            print("Leak in BACK HIPS Detected!")
            shutdown()
        time.sleep(1)  # Check every second
except KeyboardInterrupt:
    print("Script interrupted by user")
finally:
    GPIO.cleanup()