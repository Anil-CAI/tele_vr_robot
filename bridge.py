#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json, _thread, time, traceback
from websocket import WebSocketApp

# WebSocket server from Node.js
WS_URL = "ws://localhost:8080"

# Real robot topic
CMD_TOPIC = "/cmd_vel"


class RosBridge(Node):
    def __init__(self):
        super().__init__('rosbridge_ws_dev')
        self.get_logger().info('Starting ROS <-> WebSocket bridge (dev mode)')

        # Publisher to real robot/cmd_vel
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)

        # Optional status back to WebSocket
        self.status_sub = self.create_subscription(String, '/bridge/status', self.status_cb, 10)

        self.ws = None
        self.ws_connected = False

        # Run WebSocket client thread
        _thread.start_new_thread(self.ws_thread, ())

    # send status back to browser
    def status_cb(self, msg: String):
        if self.ws_connected and self.ws is not None:
            try:
                self.ws.send(msg.data)
            except Exception as e:
                self.get_logger().warn(f"Failed to send status to WS: {e}")

    # WebSocket thread
    def ws_thread(self):
        def on_message(ws, message):
            try:
                data = json.loads(message)
            except Exception:
                self.get_logger().warn("Received non-json message over WS")
                return

            # Browser sent cmd_vel
            if data.get("type") == "cmd_vel":
                try:
                    t = Twist()
                    t.linear.x = float(data.get("linear", 0.0))
                    t.angular.z = float(data.get("angular", 0.0))

                    self.cmd_pub.publish(t)
                    self.get_logger().info(
                        f"Published cmd_vel -> {CMD_TOPIC}: linear={t.linear.x}, angular={t.angular.z}"
                    )
                except Exception as e:
                    self.get_logger().error("Error publishing cmd_vel: " + str(e) + "\n" + traceback.format_exc())

        def on_open(ws):
            self.get_logger().info("WS connected to server")
            self.ws_connected = True
            self.ws = ws

        def on_close(ws, code, msg):
            self.get_logger().info(f"WS closed: {code} {msg}")
            self.ws_connected = False
            self.ws = None

        def on_error(ws, err):
            self.get_logger().error(f"WS error: {err}")

        # Persistent reconnect loop
        while rclpy.ok():
            try:
                wsapp = WebSocketApp(
                    WS_URL,
                    on_message=on_message,
                    on_open=on_open,
                    on_close=on_close,
                    on_error=on_error,
                )
                wsapp.run_forever()
            except Exception as e:
                self.get_logger().error(f"WebSocket run_forever exception: {e}")

            time.sleep(2)  # retry delay

    # standard ROS main
    def shutdown(self):
        self.get_logger().info("Shutting down rosbridge_ws_dev")


def main(args=None):
    rclpy.init(args=args)
    node = RosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
