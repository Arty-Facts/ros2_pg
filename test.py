import rclpy
from rclpy.node import Node
import numpy as np
from visual_lab_interfaces.srv import SetScreenBackground, SetScreenImage
from cv_bridge import CvBridge
from PIL import Image
import time
import cv2
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import random

class ScreenDisplayerNode(Node):
    def __init__(self) -> None:
        super().__init__("screen_displayer_node")

        self.bridge = CvBridge()
        self.bg_cli = self.create_client(SetScreenBackground, 'screen/background')
        self.bg_size = (5760, 1200)
        self.img_cli = self.create_client(SetScreenImage, 'screen/image')
        self.boby_sub = self.create_subscription(String, "body_status",
            self.player_callback,
            qos_profile_sensor_data
        )
        while not (self.bg_cli.wait_for_service(timeout_sec = 1.0) or self.img_cli.wait_for_service(timeout_sec = 1.0)):
            self.get_logger().info('Services not available, waiting...')
        # image = Image.open("NEST-Background.png").convert('BGR')
        image = cv2.imread("NEST-Background.png") 
        self.cloud = cv2.imread("cloud1_small.png")
        self.cloud_startx = self.bg_size[0]-100
        self.cloud_x = [self.cloud_startx, self.cloud_startx, self.cloud_startx]
        self.cloud_y = [300, 400, 500]
        self.cloud_windspeed = [-20,-50,-75]
        
        self.bee  = cv2.imread("yellowjacket-right-smallest.png")
        self.bee_startx = 0
        self.bee_x = 0
        self.bee_y = 600
        
        self.cloud_timer = self.create_timer(0.2, self.move_cloud)  # Timer to call move_object every 0.1 seconds
        self.runner_timer = self.create_timer(0.1, self.bee_runner)

        self.set_image(image)

    def bee_runner(self):
        self.bee_x += 10
        if self.bee_x >= self.bg_size[0]:
            self.bee_x = self.bee_startx
        self.set_object(self.bee, self.bee_x, self.bee_y, 'bee')
                

    def set_image(self, image):
        request = SetScreenBackground.Request()
        request.image = self.bridge.cv2_to_imgmsg(image)
        self.bg_cli.call_async(request)
        
    def set_object(self, image, x , y, label):
        request = SetScreenImage.Request()
        request.id = label    
        if ( (x, y) >= (0, 0) ) and ( tuple(np.array([x,y])+image.shape[1::-1]) < self.bg_size ):      
            request.x = x
            request.y = y
        else:
            request.x = 0
            request.y = 0
            image = np.zeros((1, 1, 3), dtype=np.uint8)
        request.image = self.bridge.cv2_to_imgmsg(image)        
        self.img_cli.call_async(request)

    def player_callback(self, msg):
        print(msg.data)
        if msg.data == "jump":
            self.bee_y -= 20
            self.set_object(self.bee, self.bee_x, self.bee_y, 'bee')
        elif msg.data == "duck":
            self.bee_y += 20
            self.set_object(self.bee, self.bee_x, self.bee_y, 'bee')

    def move_cloud(self):
        # Move the object to the right by 10 pixels
        for cidx in range(0,3):
            self.cloud_x[cidx] += self.cloud_windspeed[cidx]
            if self.cloud_x[cidx] <= 0: #self.bg_size[0]:
                self.cloud_x[cidx] = self.cloud_startx  # Reset to the left edge if it goes off the screen
                self.cloud_y[cidx] = random.randint(300, 500)
            self.set_object(self.cloud, self.cloud_x[cidx], self.cloud_y[cidx], 'cloud'+str(cidx))


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
