#!/usr/bin/env python3
# image_streamer.py — subscribes to /camera/image_raw and exposes MJPEG on :8082

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response

app = Flask(__name__)
bridge = CvBridge()
latest_jpg = None
lock = threading.Lock()


class ImageListener(Node):
    def __init__(self):
        super().__init__("image_listener")
        self.get_logger().info("Subscribing to /camera/image_raw")
        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.cb,
            10,
        )

    def cb(self, msg: Image):
        global latest_jpg
        try:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, jpg = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return
            with lock:
                latest_jpg = jpg.tobytes()
        except Exception as e:
            self.get_logger().error(f"Image convert error: {e}")


def generate():
    global latest_jpg
    while True:
        frame = None
        with lock:
            frame = latest_jpg
        if frame is not None:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
        else:
            time.sleep(0.05)


@app.route("/stream")
def stream():
    return Response(
        generate(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/camera/stream")
def camera_stream():
    return Response(
        generate(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def spin_ros():
    rclpy.init()
    node = ImageListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def main():
    t = threading.Thread(target=spin_ros, daemon=True)
    t.start()
    print("Camera streamer running at: http://localhost:8082/camera/stream")
    app.run(host="0.0.0.0", port=8082, threaded=True)


if __name__ == "__main__":
    main()
