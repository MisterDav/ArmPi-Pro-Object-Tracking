import ctypes
ctypes.CDLL("/home/ubuntu/.local/lib/python3.8/site-packages/ncnn.libs/libgomp-a49a47f9.so.1.0.0", mode=ctypes.RTLD_GLOBAL)

import threading
import cv2
from ultralytics import YOLO
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

class FastModel:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.latest_msg = None
        self.results = None
        self.lock = threading.Lock()
        self.running = True
        self.frame = None
        self.new_frame = threading.Event()
        self.processed_frame = threading.Event()

    def update_frame(self, msg):
        # Minimal work: just store latest frame
        with self.lock:
            self.latest_msg = msg
            self.new_frame.set()

    def get_frame(self):
        with self.lock:
            msg = self.latest_msg
            self.latest_msg = None  # drop old frames
        return msg

    def process_loop(self):
        while self.running and not rospy.is_shutdown():
            msg = self.get_frame()
            if msg is None:
                time.sleep(0.001)
                continue

            start = time.perf_counter()

            # Convert ROS -> OpenCV
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Resize for speed (critical)
            frame = cv2.resize(frame, (640, 640))

            # Inference
            self.results = self.model.track(frame, verbose=False, persist=True)

            self.draw(frame, self.results)
            self.frame = frame

            dt = (time.perf_counter() - start) * 1000
            self.processed_frame.set()
            
            #print(f"[FRAME] {dt:.1f} ms")

    def draw(self, frame, results):
        
        cv2.imshow("Detection", results[0].plot())
        cv2.waitKey(1)


model = None

def img_callback(msg):
    model.update_frame(msg)

def init():
    model = FastModel("./yolo_primitives_ncnn_model")
    print("[Camera] Initialized model.")
    rospy.Subscriber(
        "/usb_cam/image_raw",
        Image,
        img_callback,
        queue_size=1,        # always drop old frames
        buff_size=2**24,
        tcp_nodelay=True
    )
    # Start processing thread
    print("[Camera] Created camera subscriber.")
    threading.Thread(target=model.process_loop, daemon=True).start()
    print("[Camera] Started processing thread.")
    return model


if __name__ == "__main__":
    main()
    model.running = False
    cv2.destroyAllWindows()