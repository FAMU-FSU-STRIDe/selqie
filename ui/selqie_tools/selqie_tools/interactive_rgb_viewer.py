import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import matplotlib.pyplot as plt

class RGBViewer(Node):
    def __init__(self):
        super().__init__('rgb_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            'stereo/left/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.rgb_values = []

        # Set up matplotlib figure
        self.fig, (self.ax_frame, self.ax_zoom) = plt.subplots(1, 2)
        self.im_display = self.ax_frame.imshow(np.zeros((480, 640, 3), dtype=np.uint8))
        self.ax_frame.set_title('Video Frame')
        self.zoom_display = self.ax_zoom.imshow(np.zeros((50, 50, 3), dtype=np.uint8))
        self.ax_zoom.set_title('Mean RGB Value')

        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        print("Instructions:\nClick on image to collect RGB.\nPress 'q' to quit.\n")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_frame = frame
        self.im_display.set_data(frame)
        self.fig.canvas.draw_idle()

    def onclick(self, event):
        if event.inaxes != self.ax_frame or self.latest_frame is None:
            return

        x, y = int(event.xdata), int(event.ydata)
        rgb = self.latest_frame[y, x, :]
        self.rgb_values.append(rgb)

        mean_rgb = np.mean(self.rgb_values, axis=0)
        zoom_color = np.ones((50, 50, 3), dtype=np.uint8) * mean_rgb.astype(np.uint8)
        self.zoom_display.set_data(zoom_color)
        self.ax_zoom.set_title(f'Mean RGB: {mean_rgb.astype(int)}')
        self.fig.canvas.draw_idle()

        print(f'Clicked at ({x}, {y}) -> RGB: {rgb}, Mean: {mean_rgb}')

    def on_key(self, event):
        if event.key == 'q':
            if self.rgb_values:
                mean_rgb = np.mean(self.rgb_values, axis=0)
                std_rgb = np.std(self.rgb_values, axis=0)
                print(f"\nFinal Stats:")
                print(f"Mean RGB: [{mean_rgb[0]:.2f}, {mean_rgb[1]:.2f}, {mean_rgb[2]:.2f}]")
                print(f"Std Dev:  [{std_rgb[0]:.2f}, {std_rgb[1]:.2f}, {std_rgb[2]:.2f}]")
            plt.close('all')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RGBViewer()

    plt.ion()  # interactive mode
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        plt.pause(0.01)

if __name__ == '__main__':
    main()
