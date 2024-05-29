import rclpy
from rclpy.node import Node
import numpy as np
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from cv_bridge import CvBridge
from PIL import Image
import time
import cv2

class ScreenDisplayerNode(Node):
    def __init__(self) -> None:
        super().__init__("screen_displayer_node")

        self.bridge = CvBridge()
        self.bg_cli = self.create_client(SetScreenBackground, 'screen/background')
        self.bg_size = (5760, 1200)
        self.img_cli = self.create_client(SetScreenImage, 'screen/image')
        while not (self.bg_cli.wait_for_service(timeout_sec = 1.0) or self.img_cli.wait_for_service(timeout_sec = 1.0)):
            self.get_logger().info('Services not available, waiting...')
        # image = Image.open("NEST-Background.png").convert('BGR')
        image = cv2.imread("NEST-Background.png") 
        cload = cv2.imread("cloud1_small.png")
        bee  = cv2.imread("yellowjacket-left-smallest.png")

        self.set_image(image)
        for i in range(0, 5760, 100):
            self.set_object(cload,i,10,'white')
            for j in range(0, 1200, 100):
                self.set_object(bee,100,j,'yellow')
                time.sleep(0.1)

    def set_image(self, image):
        request = SetScreenBackground.Request()
        request.image = self.bridge.cv2_to_imgmsg(image)
        self.bg_cli.call_async(request)
        
    def set_object(self, image, x , y, label):
        request = SetScreenImage.Request()
        request.id = label    
        if ( (x, y) >= (0, 0) ) and ( tuple([x,y]+image.shape[1:-1]) < self.bg_size ):      
            request.x = x
            request.y = y
        else:
            request.x = 0
            request.y = 0
            image = np.zeros((1, 1, 3), dtype=np.uint8)
        request.image = self.bridge.cv2_to_imgmsg(image)        
        self.img_cli.call_async(request)

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
