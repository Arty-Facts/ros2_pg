import rclpy
from rclpy.node import Node
import numpy as np
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from cv_bridge import CvBridge
from PIL import Image

class ScreenDisplayerNode(Node):
    def __init__(self) -> None:
        super().__init__("screen_displayer_node")

        self.bridge = CvBridge()
        self.bg_cli = self.create_client(SetScreenBackground, 'screen/background')
        self.img_cli = self.create_client(SetScreenImage, 'screen/image')
        while not (self.bg_cli.wait_for_service(timeout_sec = 1.0) or self.img_cli.wait_for_service(timeout_sec = 1.0)):
            self.get_logger().info('Services not available, waiting...')
        image = Image.open("super_mario_world_desktop_by_tregnier2795_d27cao8-pre.jpg")
        self.set_image(image)

    def set_image(self, image):
        bg = np.array(image.resize((5760, 1200)))
        request = SetScreenBackground.Request()
        request.image = self.bridge.cv2_to_imgmsg(bg)
        self.bg_cli.call_async(request)

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = ScreenDisplayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()