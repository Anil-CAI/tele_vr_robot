#!/usr/bin/env python3

import math
import json
import threading
import time
import ssl

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import websocket  # pip install websocket-client

WS_URL = "wss://localhost:8443"   # HTTPS/WSS Node server


def quat_to_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(theta: float):
    qz = math.sin(theta * 0.5)
    qw = math.cos(theta * 0.5)
    return 0.0, 0.0, qz, qw


class TeleopBridgeNode(Node):
    def __init__(self):
        super().__init__("teleop_bridge")

        # Manual teleop publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Pose feedback from Nav2 localization (map frame)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_cb,
            10,
        )

        # Nav2 NavigateToPose action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        self.get_logger().info("TeleopBridgeNode for TurtleBot3 started")

        self.ws = None
        self.ws_thread = threading.Thread(target=self.ws_loop, daemon=True)
        self.ws_thread.start()

    # -------- WebSocket loop --------
    def ws_loop(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to WS {WS_URL} ...")

                ws_app = websocket.WebSocketApp(
                    WS_URL,
                    on_message=lambda ws, msg: self.handle_ws_message(msg),
                    on_open=lambda ws: self.on_ws_open(ws),
                    on_error=lambda ws, err: self.on_ws_error(ws, err),
                    on_close=lambda ws, code, reason: self.on_ws_close(ws, code, reason),
                )

                # Ignore self-signed cert errors
                ws_app.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})

            except Exception as e:
                self.get_logger().error(f"WS loop error: {e}")

            time.sleep(1.0)

    def on_ws_open(self, ws):
        self.get_logger().info("WS connected")
        self.ws = ws

    def on_ws_error(self, ws, err):
        self.get_logger().error(f"WS error: {err}")

    def on_ws_close(self, ws, code, reason):
        self.get_logger().info(f"WS closed: code={code}, reason={reason}")
        self.ws = None

    def handle_ws_message(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON from WS: {raw}")
            return

        msg_type = data.get("type", "")

        if msg_type == "cmd_vel":
            self.handle_cmd_vel(data)
        elif msg_type == "nav_goal":
            self.handle_nav_goal(data)

    # ----- Manual cmd_vel from web -----
    def handle_cmd_vel(self, data: dict):
        lin = float(data.get("linear", 0.0))
        ang = float(data.get("angular", 0.0))

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_pub.publish(twist)

        self.send_ws({
            "type": "ack",
            "src": "bridge",
            "linear": lin,
            "angular": ang,
        })

    # ----- Nav2 goal from web -----
    def handle_nav_goal(self, data: dict):
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        theta = float(data.get("theta", 0.0))

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 NavigateToPose action server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(theta)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending Nav2 goal: x={x:.2f}, y={y:.2f}, theta={theta:.2f}"
        )

        _ = self.nav_client.send_goal_async(goal_msg)

        # Notify web UI
        self.send_ws({
            "type": "nav_goal_ack",
            "x": x,
            "y": y,
            "theta": theta,
        })

    def send_ws(self, obj: dict):
        if self.ws is None:
            return
        try:
            self.ws.send(json.dumps(obj))
        except Exception as e:
            self.get_logger().warn(f"WS send failed: {e}")

    # -------- ROS callbacks --------
    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quat_to_yaw(ori)

        self.get_logger().info(
            f"POSE OUT: x={pos.x:.2f}, y={pos.y:.2f}, th={yaw:.2f}"
        )

        payload = {
            "type": "pose",
            "x": pos.x,
            "y": pos.y,
            "theta": yaw,
            "frame_id": msg.header.frame_id,  # "map"
        }
        self.send_ws(payload)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
