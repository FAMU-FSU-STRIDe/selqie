import Jetson.GPIO as GPIO
import os
import time
import logging
from datetime import datetime

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

SENSOR_PIN = 35 # (physical pin)
SENSOR_PIN2 = 22 # (physical pin)
SENSOR_PIN3 = 38 # (physical pin)

# Logging Setup
logging.basicConfig(
	filename="/var/log/water_shutdown.log",
	level=logging.INFO,
	format="%(asctime)s - %(message)s"
)

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SENSOR_PIN, GPIO.IN) # Back
GPIO.setup(SENSOR_PIN2, GPIO.IN) # Front
GPIO.setup(SENSOR_PIN3, GPIO.IN) # Body

def shutdown():
    logging.info("Water detected! Shutting down...")
    os.system("sudo shutdown -h now")

try:
    logging.info("Leak Detection Started on GPIO (HDR 35, HDR 38, and HDR 22)")
    while True:
        if GPIO.input(SENSOR_PIN) == GPIO.HIGH:
            logging.info("Leak on Header 35 (Rear)")
            shutdown()
            time.sleep(10)
        if GPIO.input(SENSOR_PIN2) == GPIO.HIGH:
            logging.info("Leak on Header 22 (Front)")
            shutdown()
            time.sleep(10)
        if GPIO.input(SENSOR_PIN3) == GPIO.HIGH:
            logging.info("Leak on Header 38 (Body)")
            shutdown()
            time.sleep(10)
        time.sleep(1)  # Check every second
except KeyboardInterrupt:
    print("Script interrupted by user")
finally:
    GPIO.cleanup()
    logging.info("GPIO cleanup done.")
