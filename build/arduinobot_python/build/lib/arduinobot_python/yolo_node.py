import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import sys
import logging

# class DummyFile(object):
#     def write(self, x): pass

# sys.stdout = DummyFile()
# sys.stderr = DummyFile()
# # Hàm callback xử lý sự kiện chuột
# def mouse_event(event, x, y, flags, param):
#     if event == cv2.EVENT_LBUTTONDOWN:  # Click chuột trái
#         logging.info(f"Clicked at position: ({x}, {y})")

# # Tạo cửa sổ hiển thị
# cv2.namedWindow('image')

# # Đăng ký callback chuột với cửa sổ 'image'
# cv2.setMouseCallback('image', mouse_event)

class Yolo8Node(Node):
    def __init__(self):
        super().__init__('yolo8_node')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt", verbose=False)  # hoặc dùng custom model bạn đã train: "beer_coca.pt"
        self.sub = self.create_subscription(Image, 'camera/image', self.callback, 10)
        self.pub = self.create_publisher(Detection2DArray, "/yolo/detections", 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)[0]

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        for det in results.boxes:
            x1, y1, x2, y2 = map(int, det.xyxy[0])
            cls_id = int(det.cls[0])
            conf = float(det.conf[0])
            label = f"{self.model.names[cls_id]} {conf:.2f}"

            # Vẽ box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Hiển thị (chỉ dùng để debug)
        cv2.imshow("YOLOv8 Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

        #     for det in results.boxes:
        #     cls_id = int(det.cls[0].item())
        #     score = float(det.conf[0].item())

        #     # Chỉ lấy nếu là beer/coca (giả sử id là 39 và 41)
        #     if cls_id in [39, 41]:  # class ID của "beer" hoặc "coca" tùy dataset
        #         detection = Detection2D()
        #         detection.bbox.center.x = float(det.xywh[0][0])
        #         detection.bbox.center.y = float(det.xywh[0][1])
        #         detection.bbox.size_x = float(det.xywh[0][2])
        #         detection.bbox.size_y = float(det.xywh[0][3])

        #         hypo = ObjectHypothesisWithPose()
        #         hypo.id = cls_id
        #         hypo.score = score
        #         detection.results.append(hypo)

        #         det_msg.detections.append(detection)

        # self.pub.publish(det_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Yolo8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
