import os
import threading
from time import sleep

import pyrealsense2 as rs
import numpy as np
import cv2

class ImageGrap:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)

        self.device = pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.is_running = False

    def start(self ):
        self.pipeline.start(self.config)
        self.is_running = True

    def grab_img(self):
        frame = self.pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        if not color_frame:
            return None,None
        color_img = np.asanyarray(color_frame.get_data())
        color_colormap_dim = color_img.shape
        return color_img,color_colormap_dim

    def save_img(self,img_path,img_name):
        if not self.is_running:
            self.start()
        img,_ = self.grab_img()
        if  img is not None:
            if not os.path.exists(img_path):
                os.makedirs(img_path)
                print(f"img_path dir created:{img_path}")
            cv2.imwrite(filename=f"{img_path}/{img_name}.png",img=img)
        else:
            print("Error! No image")

    def show(self):
        if not self.is_running:
            self.start()
        t = threading.Thread(target=self.window, args=())
        t.start()

    def window(self):
        while True:
            img,_ = self.grab_img()
            if img is not None:
                cv2.imshow("img",img)
                key =cv2.waitKey(1)
                if key ==ord('q'):
                    break


if __name__ == "__main__":
    img_path = "../img"
    IG = ImageGrap()
    IG.start()
    sleep(1)
    for i in range(10):
        IG.save_img(img_path =img_path,img_name=str(i))
    exit(0)

