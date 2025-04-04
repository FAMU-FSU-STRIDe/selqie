# ODrive Control Notes

### Links
- [ODrive Pro Shop](https://odriverobotics.com/shopfolder)
- [Documentation *(v0.6.8)*](https://docs.odriverobotics.com/v/0.6.8/guides/getting-started.html)
- [MJ5208 Motor Shop](https://mjbots.com/products/mj5208)
- [NTCLE300E3502SB Thermistor Shop](https://www.mouser.com/ProductDetail/Vishay-BC-Components/NTCLE300E3502SB?qs=%2FWiulJ9oly5IYkswf0Y9eA%3D%3D)
- [ODrive CAN Guide](https://docs.odriverobotics.com/v/0.6.8/guides/can-guide.html)
- [ODrive Pro Pinout](https://docs.odriverobotics.com/v/0.6.8/hardware/pro-datasheet.html#pro-pinout)

### ODrive + MJ5208 Auto-Configuration Script
***Note:** This script is for ODrive firmware v0.6.8. Other firmware versions may not work.*
1. Plug in USB from the Jetson to the ODrive controller 
2. Open the Terminal on the the Jetson
3. Go to the `tools` folder: $`cd ~/selqie_ws/src/tools`
4. Run the auto-configuration script: $`python3 configure_odrive_mj5208.py <CAN_ID>`
5. Let the script complete before unplugging

### ODrive CAN Node
- TODO