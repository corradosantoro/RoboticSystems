#
# image_reader.py
#

import socket
import cv2
import numpy as np

class ImageReader:

    def __init__(self, _host, _port):
        self.sd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = _host
        self.port = _port

    def connect(self):
        self.sd.connect( (self.host, self.port) )

    def read_image(self, _width, _height):
        image_data = bytes()
        max_len = _width * _height * 3
        max_len += 4
        while len(image_data) < max_len:
            to_read = max_len - len(image_data)
            image_data += self.sd.recv(to_read)
        image_data = image_data[4:]
        image = np.frombuffer(image_data, np.uint8).reshape( _height, _width, 3 )
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image



if __name__ == "__main__":
    from dds import *
    dds = DDS()
    dds.start()
    imr = ImageReader('localhost', 4445)
    imr.connect()
    while True:
        dds.publish('read_image', 1, DDS.DDS_TYPE_INT)
        img = imr.read_image(512, 512)
        cv2.imshow('image', img)
        k = cv2.waitKey(20)
        if k == ord('q'):
            break

    dds.stop()
