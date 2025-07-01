#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np
import os
import cv2
os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"


from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from drink_selector_window import Ui_Form
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from PyQt5 import QtWidgets

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge
import logging

logging.basicConfig(level=logging.DEBUG)

bridge = CvBridge()

img = np.zeros([480, 640, 3])

class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.mouse_x = 0
        self.mouse_y = 0
        self.click_mouse = False

    def mouseMoveEvent(self,event):
        self.mouse_x = event.scenePos().x()
        self.mouse_y = event.scenePos().y()

    def mousePressEvent(self, event):
        self.click_mouse = True


class GUI(QDialog):

    def __init__(self,parent=None):
        super(GUI, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.center_window()

        self.scene = GraphicsScene(self.ui.graphicsView)
        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setMouseTracking(True)

        rclpy.init(args=None)
        self.camera_subscriber = Node('image_subscriber_node')
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.sub_rgb = self.camera_subscriber.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.pub_node = Node('pub_path')
        self.pub = self.pub_node.create_publisher(Int64MultiArray, '/pick_object_point', 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        # Initialize CvBridge
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # hoặc dùng custom model bạn đã train: "beer_coca.pt"
        
        self.brush = QBrush(QColor(255,255,255,255)) 
        self.latest_rgb_frame = None
        self.latest_depth_frame = None
        self.claw_sp_length = 0.48
        self.fx = self.fy = self.cx = self.cy = None
        self.target_point = np.zeros(3, dtype=float)

    def center_window(self):
        frameGm = self.frameGeometry()  # gọi trên self (QWidget), không phải self.ui
        screenCenter = QtWidgets.QDesktopWidget().availableGeometry().center()
        frameGm.moveCenter(screenCenter)
        self.move(frameGm.topLeft())

    def image_callback(self, msg):
        self.latest_rgb_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def display_image(self):
        if self.latest_rgb_frame is None:
            return
        
        self.scene.clear()
        rgb_image = cv2.cvtColor(self.latest_rgb_frame, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, 3 * width, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        pixmap_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(pixmap_item)

        if self.latest_rgb_frame is None:
            return
        
        logging.info("Start detecting objects ... ")
        results = self.model(self.latest_rgb_frame)
        detections = results[0].boxes
        for box in detections:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # Khoang cach giua tam obj vs chuot
            dist = math.sqrt((self.scene.mouse_x - cx)**2 + (self.scene.mouse_y - cy)**2)
            # Id obj
            cls_id = int(box.cls[0])
            label = f"{self.model.names[cls_id]}"

            points = np.array([
                [x1, y1],  # trên trái
                [x2, y1],  # trên phải
                [x2, y2],  # dưới phải
                [x1, y2],  # dưới trái
            ])
        
            qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])

            if dist < 15:

                self.scene.addPolygon(qpoly, QPen(QColor(255,0,0,255)), QBrush(QColor(255,0,0,100)))   

                if self.scene.click_mouse:
                    # self.calculate_point(cx, cy)
                    msg = Int64MultiArray()
                    msg.data = [cx, cy]
                    self.pub.publish(msg)
                    logging.info("Mouse click at " + label + " " + str(cx) + " " + str(cy))
                    self.scene.click_mouse = False
            else:
                self.scene.addPolygon(qpoly, QPen(QColor(0,0,255,255)), QBrush(QColor(0,0,255,100)))  

            self.scene.addEllipse(cx - 2, cy - 2, 4, 4, QPen(Qt.green), QBrush(Qt.green))     

    
    def update(self):
        rclpy.spin_once(self.camera_subscriber)
        self.display_image()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())