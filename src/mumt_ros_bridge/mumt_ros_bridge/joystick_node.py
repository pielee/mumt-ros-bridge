"""
joystick_node.py
----------------
Maps a sensor_msgs/Joy (e.g. an Xbox-compatible gamepad read by the standard
`joy` package) into a manned-aircraft control command and publishes it as a JSON
std_msgs/String on /mumt/aircraft_commands. bridge_node then forwards it to
Unreal over UDP 5005, where AUDPControlReceiver applies it to the manned pawn
(UDP_Roll/Pitch/Yaw/Throttle on M_F16).

Hardware note: an Xbox-compatible gamepad on a desktop PC (stock Linux driver;
no Jetson/xpad setup needed). A gamepad has NO detented throttle axis, so
throttle defaults to "incremental" mode (RT spools up, LT spools down, value
latches). An "axis" mode is also provided for throttle quadrants/sliders that do
have an absolute throttle axis.

Output JSON (one named command for the manned pawn):
  {"commands": [{"aircraft_name": "M_F16",
                 "roll": r, "pitch": p, "yaw": y, "throttle": t}]}
  roll/pitch/yaw in [-1, 1], throttle in [0, 1].

Find your axis indices with:  jstest /dev/input/js0   or   ros2 topic echo /joy
"""

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoystickCommandNode(Node):
    def __init__(self):
        super().__init__("mumt_joystick")

        # --- target / IO ---
        self.declare_parameter("aircraft_name", "M_F16")
        self.declare_parameter("command_topic", "/mumt/aircraft_commands")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("publish_rate_hz", 50.0)

        # --- stick axis indices into sensor_msgs/Joy.axes (Xbox layout defaults) ---
        self.declare_parameter("roll_axis", 3)    # right stick X
        self.declare_parameter("pitch_axis", 4)   # right stick Y
        self.declare_parameter("yaw_axis", 0)     # left stick X

        # --- per-axis sign (+1 / -1) and gain (tune live; flip sign if reversed) ---
        self.declare_parameter("roll_sign", -1.0)
        self.declare_parameter("pitch_sign", -1.0)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("roll_scale", 1.0)
        self.declare_parameter("pitch_scale", 1.0)
        self.declare_parameter("yaw_scale", 1.0)

        # --- deadzone for the stick axes (roll/pitch/yaw), in raw axis units ---
        self.declare_parameter("deadzone", 0.08)

        # --- throttle ---
        # "incremental": RT spools up / LT spools down, value latches (good for a gamepad)
        # "axis":        a single absolute throttle axis mapped to [0,1] (e.g. a throttle slider)
        self.declare_parameter("throttle_mode", "incremental")

        # incremental-mode params (triggers)
        self.declare_parameter("throttle_up_axis", 5)     # RT
        self.declare_parameter("throttle_down_axis", 2)   # LT
        self.declare_parameter("trigger_rest", 1.0)       # raw value when released
        self.declare_parameter("trigger_full", -1.0)      # raw value when fully pressed
        self.declare_parameter("throttle_rate", 0.4)      # per second (0->1 in ~2.5 s)
        self.declare_parameter("throttle_init", 0.0)

        # axis-mode params (absolute throttle axis)
        self.declare_parameter("throttle_axis", 5)
        self.declare_parameter("throttle_raw_at_zero", 1.0)
        self.declare_parameter("throttle_raw_at_full", -1.0)

        g = self.get_parameter
        self._name = g("aircraft_name").value
        command_topic = g("command_topic").value
        joy_topic = g("joy_topic").value
        rate = float(g("publish_rate_hz").value)
        if rate <= 0.0:
            rate = 50.0
        self._dt = 1.0 / rate

        self._roll_axis = int(g("roll_axis").value)
        self._pitch_axis = int(g("pitch_axis").value)
        self._yaw_axis = int(g("yaw_axis").value)

        self._roll_sign = float(g("roll_sign").value)
        self._pitch_sign = float(g("pitch_sign").value)
        self._yaw_sign = float(g("yaw_sign").value)
        self._roll_scale = float(g("roll_scale").value)
        self._pitch_scale = float(g("pitch_scale").value)
        self._yaw_scale = float(g("yaw_scale").value)
        self._deadzone = float(g("deadzone").value)

        self._throttle_mode = str(g("throttle_mode").value)
        self._thr_up_axis = int(g("throttle_up_axis").value)
        self._thr_down_axis = int(g("throttle_down_axis").value)
        self._trig_rest = float(g("trigger_rest").value)
        self._trig_full = float(g("trigger_full").value)
        self._thr_rate = float(g("throttle_rate").value)
        self._throttle = max(0.0, min(1.0, float(g("throttle_init").value)))

        self._thr_axis = int(g("throttle_axis").value)
        self._thr_zero = float(g("throttle_raw_at_zero").value)
        self._thr_full = float(g("throttle_raw_at_full").value)

        self._last_joy = None

        self._pub = self.create_publisher(String, command_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.create_timer(self._dt, self._publish_cmd)

        self.get_logger().info(
            f"mumt_joystick started | {joy_topic} -> {command_topic} as '{self._name}' "
            f"@ {rate:.0f} Hz | throttle_mode={self._throttle_mode} | "
            f"axes roll={self._roll_axis} pitch={self._pitch_axis} yaw={self._yaw_axis}"
        )

    def _on_joy(self, msg: Joy):
        self._last_joy = msg

    @staticmethod
    def _axis(msg: Joy, idx: int) -> float:
        return float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else 0.0

    def _apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self._deadzone else v

    def _stick(self, raw: float, sign: float, scale: float) -> float:
        v = self._apply_deadzone(raw) * sign * scale
        return max(-1.0, min(1.0, v))

    def _trigger_amount(self, raw: float) -> float:
        """Map a trigger's raw reading to a 0..1 'pressed' amount."""
        span = self._trig_full - self._trig_rest
        if span == 0.0:
            return 0.0
        return max(0.0, min(1.0, (raw - self._trig_rest) / span))

    def _throttle_axis_value(self, raw: float) -> float:
        span = self._thr_full - self._thr_zero
        if span == 0.0:
            return 0.0
        return max(0.0, min(1.0, (raw - self._thr_zero) / span))

    def _update_throttle(self, j: Joy) -> float:
        if self._throttle_mode == "axis":
            return self._throttle_axis_value(self._axis(j, self._thr_axis))
        # incremental
        up = self._trigger_amount(self._axis(j, self._thr_up_axis))
        down = self._trigger_amount(self._axis(j, self._thr_down_axis))
        self._throttle += (up - down) * self._thr_rate * self._dt
        self._throttle = max(0.0, min(1.0, self._throttle))
        return self._throttle

    def _publish_cmd(self):
        if self._last_joy is None:
            return
        j = self._last_joy
        roll = self._stick(self._axis(j, self._roll_axis), self._roll_sign, self._roll_scale)
        pitch = self._stick(self._axis(j, self._pitch_axis), self._pitch_sign, self._pitch_scale)
        yaw = self._stick(self._axis(j, self._yaw_axis), self._yaw_sign, self._yaw_scale)
        throttle = self._update_throttle(j)

        payload = {
            "commands": [
                {
                    "aircraft_name": self._name,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                    "throttle": throttle,
                }
            ]
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
