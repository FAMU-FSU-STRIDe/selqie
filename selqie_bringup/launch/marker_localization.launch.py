import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

MARKER_LOCALIZATION_LAUNCH_FILE = os.path.join(
        get_package_share_directory('localization_bringup'), 'launch', 'marker_localization.launch.py')

def PointCloudRGBFilterLaunch(id, rgb, rgb_deviation):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MARKER_LOCALIZATION_LAUNCH_FILE),
        launch_arguments={
            'id': str(id),
            'rgb': str(rgb),
            'rgb_deviation': str(rgb_deviation),
        }.items()
    )

def MarkerLocalizationLaunch(id, x, y, z):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(MARKER_LOCALIZATION_LAUNCH_FILE),
        launch_arguments={
            'id': str(id),
            'marker_x': str(x),
            'marker_y': str(y),
            'marker_z': str(z),
        }.items()
    )

def generate_launch_description():
    return LaunchDescription([
        PointCloudRGBFilterLaunch('markerA', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        MarkerLocalizationLaunch('markerA', 0, 0, 0),
    ])