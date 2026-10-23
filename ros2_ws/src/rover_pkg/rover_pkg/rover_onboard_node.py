#!/usr/bin/env python3
"""
Orange Pi onboard ROS 2 node
- Subscribes to ArmCommand on /rover/arm_command and forwards it to the Pico
- Reads JSON lines from the Pico and publishes Telemetry on /rover/telemetry
- Reconnects automatically if the Pico is unplugged or resets

Serial protocol (newline-terminated JSON, 115200 baud, see pico/rover_pico):
    Orange Pi -> Pico:  {"command": "HEARTBEAT" | "ARM_ENABLE" | "STOP_ALL" | "LINK_LOST"}
                        {"axis": "Axis1", "angle": 95.0}
                        {"motor": "DC", "speed": 15.0}
    Pico -> Orange Pi:  {"type": "status", "enabled": true, "note": "...", ...}
                        {"type": "sensors", "data": {"temperature": 23.1, ...}}
"""

import json
import math
import time

import rclpy
import serial
from rclpy.node import Node
from rover_comms.msg import ArmCommand, Telemetry

COMMAND_TIMEOUT = 0.5   # s without ArmCommand before the rover stops
PICO_TIMEOUT = 1.0      # s without a Pico line before it counts as disconnected
AXES = ('Axis1', 'Axis2', 'Axis3A', 'Axis3B')


class OrangePiRoverNode(Node):
    def __init__(self):
        super().__init__('onboard')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('reconnect_s', 2.0)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.reconnect_s = self.get_parameter('reconnect_s').value

        self.ser = None
        self.rx_buffer = b''
        self.last_connect_attempt = 0.0
        self.last_pico_line = 0.0

        self.cmd = None             # newest ArmCommand
        self.cmd_time = 0.0
        self.enable_sent = False    # ARM_ENABLE goes out once per enable press
        self.sent_angles = {}

        self.pico_enabled = False
        self.note = 'Waiting for Pico'
        self.sensors = {}

        self.publisher_ = self.create_publisher(Telemetry, 'rover/telemetry', 10)
        self.subscription = self.create_subscription(
            ArmCommand, 'rover/arm_command', self.command_callback, 10)

        self.send_timer = self.create_timer(0.05, self.send_tick)       # 20 Hz to the Pico
        self.read_timer = self.create_timer(0.01, self.read_tick)       # poll serial at 100 Hz
        self.telemetry_timer = self.create_timer(0.2, self.publish_telemetry)

        self.connect()

    # ---------- serial connection ----------

    def connect(self):
        self.last_connect_attempt = time.monotonic()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.1)
            self.ser.reset_input_buffer()
            self.rx_buffer = b''
            self.enable_sent = False
            self.write({'command': 'LINK_LOST'})  # start from a known stopped state
            self.get_logger().info(f'Connected to Pico on {self.port}')
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(
                f'Could not open {self.port}: {e}. Retrying every {self.reconnect_s:.0f}s',
                throttle_duration_sec=10.0)

    def disconnect(self, reason):
        self.get_logger().error(f'Serial connection lost: {reason}')
        try:
            if self.ser:
                self.ser.close()
        except serial.SerialException:
            pass
        self.ser = None
        self.pico_enabled = False

    def ensure_connected(self):
        if self.ser is None and time.monotonic() - self.last_connect_attempt > self.reconnect_s:
            self.connect()
        return self.ser is not None

    def write(self, packet):
        if self.ser is None:
            return
        try:
            self.ser.write((json.dumps(packet) + '\n').encode('ascii'))
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as e:
            self.disconnect(e)

    # ---------- ROS -> Pico ----------

    def command_callback(self, msg):
        self.cmd = msg
        self.cmd_time = time.monotonic()

    def send_tick(self):
        if not self.ensure_connected():
            return

        if self.cmd is None or time.monotonic() - self.cmd_time > COMMAND_TIMEOUT:
            # Ground station gone: stop once, then stay quiet so the Pico's
            # own heartbeat timeout also trips.
            if self.enable_sent or self.pico_enabled:
                self.get_logger().warn('Ground station lost; stopping outputs')
                self.write({'command': 'LINK_LOST'})
                self.enable_sent = False
            return

        self.write({'command': 'HEARTBEAT'})

        if not self.cmd.enable:
            if self.enable_sent or self.pico_enabled:
                self.write({'command': 'STOP_ALL'})
            self.enable_sent = False
            return

        if not self.enable_sent:
            self.write({'command': 'ARM_ENABLE'})
            self.enable_sent = True
            self.sent_angles = {}  # send every target once the Pico confirms
            return

        # Joint and motor commands are only accepted once the Pico is enabled.
        # If the Pico stops itself, the operator has to enable again.
        if not self.pico_enabled:
            return

        targets = (self.cmd.axis1, self.cmd.axis2, self.cmd.axis3a, self.cmd.axis3b)
        for axis, angle in zip(AXES, targets):
            if self.sent_angles.get(axis) != angle:
                self.write({'axis': axis, 'angle': round(angle, 2)})
                self.sent_angles[axis] = angle
        # The Pico zeroes the motor unless it is refreshed every 300 ms.
        self.write({'motor': 'DC', 'speed': round(self.cmd.motor, 2)})

    # ---------- Pico -> ROS ----------

    def read_tick(self):
        if not self.ensure_connected():
            return
        try:
            waiting = self.ser.in_waiting
            if waiting:
                self.rx_buffer += self.ser.read(waiting)
        except (serial.SerialException, OSError) as e:
            self.disconnect(e)
            return

        while b'\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
            self.handle_line(line.decode('utf-8', errors='replace').strip())

        if len(self.rx_buffer) > 8192:  # garbage with no newlines; drop it
            self.rx_buffer = b''

    def handle_line(self, line):
        if not line:
            return
        try:
            packet = json.loads(line)
        except ValueError:
            self.get_logger().info(f'Pico: {line}')
            return
        if not isinstance(packet, dict):
            return
        self.last_pico_line = time.monotonic()

        if packet.get('type') == 'sensors' and isinstance(packet.get('data'), dict):
            self.sensors = packet['data']
        elif packet.get('type') == 'status':
            self.pico_enabled = packet.get('enabled') is True
            note = packet.get('error') or packet.get('note') or ''
            if note != self.note:
                self.get_logger().info(f'Pico: {note}')
            self.note = str(note)

    def publish_telemetry(self):
        def reading(name):
            value = self.sensors.get(name)
            return float(value) if isinstance(value, (int, float)) else math.nan

        connected = time.monotonic() - self.last_pico_line < PICO_TIMEOUT
        msg = Telemetry()
        msg.altitude = reading('altitude')
        msg.temperature = reading('temperature')
        msg.pressure = reading('pressure')
        msg.acceleration.x = reading('accel_x')
        msg.acceleration.y = reading('accel_y')
        msg.acceleration.z = reading('accel_z')
        msg.pico_connected = connected
        msg.enabled = connected and self.pico_enabled
        msg.note = self.note if connected else 'Pico not connected'
        self.publisher_.publish(msg)

    def close(self):
        self.write({'command': 'STOP_ALL'})
        if self.ser and self.ser.is_open:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = OrangePiRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
