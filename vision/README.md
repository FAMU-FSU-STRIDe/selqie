## Camera Calibration Instructions

1. Turn on the robot and place it in the pool
2. Start the onboard computing
3. On the laptop (Not in SSH), run `ros2 launch vision_bringup camera_calibration.launch.py`
4. A GUI should appear on screen showing the two cameras
5. Insert the checkerboard in the water infront of the cameras
6. Move the checkerboard around and at different angles until all bars in the GUI are green
7. Once ready, hit the save button in the GUI
8. Close the GUI and go to the `tools` folder in the CI
9. Run `./update_stereo_calibration.sh`, which will move the new calibration to the `vision/vision_bringup/config` folder
10. Restart the onboard computing system
11. Test the calibration by visualizing the rectified images or disparity maps using `ros2 run image_view image_view image:=/stereo/left/image_rect` or `ros2 run image_view disparity_view image:=/stereo/disparity` from the remote laptop (Not in SSH)
12. You can tune the disparity parameters live using `ros2 run rqt_reconfigure rqt_reconfigure`
