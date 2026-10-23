#!/usr/bin/env python3
"""
Ground station ROS 2 node
- Keyboard arm control through a pygame window (window must have focus)
- Publishes ArmCommand to /rover/arm_command at 20 Hz (this is also the heartbeat)
- Subscribes to Telemetry on /rover/telemetry and shows it in the window

Controls (hold keys to move, rate-based):
    ENTER          Enable / disable outputs
    SPACE          STOP (disable outputs)
    A / D          Base DC motor (axis 0), runs while held
    W / S          Axis1
    UP / DOWN      Axis2
    LEFT / RIGHT   Axis3A
    Q / E          Axis3B
    SHIFT (hold)   Fine mode (slow movement)
    R              Return to home position
    ESC            Quit (stops the rover)
"""

import math
import time

import pygame
import rclpy
from rclpy.node import Node
from rover_comms.msg import ArmCommand, Telemetry

PUBLISH_HZ = 20.0
SPEED = 30.0             # degrees per second while a key is held
FINE_FACTOR = 0.25       # speed multiplier while SHIFT is held
MOTOR_PERCENT = 30.0     # base motor duty while A/D is held (the Pico caps it too)
TELEMETRY_TIMEOUT = 1.0  # seconds before telemetry is shown as stale

# joint: (decrease key, increase key, min, max, home). Match pico/rover_pico/config.h.
JOINTS = {
    'axis1':  (pygame.K_s,    pygame.K_w,       0.0, 180.0, 90.0),
    'axis2':  (pygame.K_DOWN, pygame.K_UP,      0.0, 180.0, 90.0),
    'axis3a': (pygame.K_LEFT, pygame.K_RIGHT, -70.0,  70.0,  0.0),
    'axis3b': (pygame.K_q,    pygame.K_e,       0.0, 180.0, 90.0),
}

LABELS = {
    'axis1':  'Axis1   [W/S]',
    'axis2':  'Axis2   [Up/Down]',
    'axis3a': 'Axis3A  [Left/Right]',
    'axis3b': 'Axis3B  [Q/E]',
}


class GSNode(Node):
    def __init__(self):
        super().__init__('ground_station')
        self.publisher_ = self.create_publisher(ArmCommand, 'rover/arm_command', 10)
        self.subscription = self.create_subscription(
            Telemetry, 'rover/telemetry', self.listener_callback, 10)

        pygame.init()
        self.screen = pygame.display.set_mode((560, 460))
        pygame.display.set_caption('Rover Ground Station')
        self.font = pygame.font.Font(None, 26)
        self.small = pygame.font.Font(None, 20)

        self.angles = {j: cfg[4] for j, cfg in JOINTS.items()}
        self.motor = 0.0
        self.enable = False
        self.enable_time = 0.0
        self.running = True
        self.telemetry = None
        self.last_telemetry_time = 0.0
        self.last_tick = time.monotonic()

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.tick)
        self.get_logger().info('Ground station ready. Focus the pygame window to drive.')

    def listener_callback(self, msg):
        self.telemetry = msg
        self.last_telemetry_time = time.monotonic()

    def telemetry_fresh(self):
        return (self.telemetry is not None and
                time.monotonic() - self.last_telemetry_time < TELEMETRY_TIMEOUT)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    self.enable = not self.enable
                    self.enable_time = time.monotonic()
                    self.get_logger().info('Enable requested' if self.enable else 'Disabled')
                elif event.key == pygame.K_SPACE:
                    self.enable = False
                    self.get_logger().warn('STOP')
                elif event.key == pygame.K_r:
                    self.angles = {j: cfg[4] for j, cfg in JOINTS.items()}
                    self.get_logger().info('Returning to home position')

    def update_controls(self, dt):
        keys = pygame.key.get_pressed()
        fine = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        step = SPEED * (FINE_FACTOR if fine else 1.0) * dt
        for joint, (dec, inc, lo, hi, _) in JOINTS.items():
            delta = (step if keys[inc] else 0.0) - (step if keys[dec] else 0.0)
            self.angles[joint] = max(lo, min(hi, self.angles[joint] + delta))
        self.motor = MOTOR_PERCENT * (keys[pygame.K_d] - keys[pygame.K_a])

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now

        self.handle_events()
        self.update_controls(dt)

        # If the rover stopped itself (timeout, fault, bad config), drop our
        # enable too so the next ENTER press re-enables it.
        if (self.enable and now - self.enable_time > 1.5 and
                not (self.telemetry_fresh() and self.telemetry.enabled)):
            self.enable = False
            self.get_logger().warn('Rover is not enabled; press ENTER to try again')

        msg = ArmCommand()
        msg.enable = self.enable
        for joint, value in self.angles.items():
            setattr(msg, joint, float(value))
        msg.motor = float(self.motor) if self.enable else 0.0
        self.publisher_.publish(msg)
        self.draw()

    def stop(self):
        msg = ArmCommand()
        msg.enable = False
        self.publisher_.publish(msg)

    def draw(self):
        bg, fg, dim = (25, 28, 34), (230, 230, 230), (140, 140, 150)
        good, bad, accent = (90, 200, 120), (220, 90, 90), (90, 160, 230)
        self.screen.fill(bg)
        y = 15

        fresh = self.telemetry_fresh()
        rover_enabled = fresh and self.telemetry.enabled
        if rover_enabled:
            self.text('ENABLED  (Space = STOP)', 15, y, good)
        elif self.enable:
            self.text('ENABLING...', 15, y, accent)
        else:
            self.text('STOPPED  (Enter = enable)', 15, y, bad)
        y += 35

        for joint, (_, _, lo, hi, _) in JOINTS.items():
            value = self.angles[joint]
            self.text(LABELS[joint], 15, y, fg)
            self.text(f'{value:6.1f}°', 210, y, fg)
            bar_x, bar_w = 300, 240
            pygame.draw.rect(self.screen, (60, 64, 72), (bar_x, y + 3, bar_w, 12))
            fill = int(bar_w * (value - lo) / (hi - lo))
            pygame.draw.rect(self.screen, accent, (bar_x, y + 3, fill, 12))
            y += 30
        self.text('Base motor  [A/D]', 15, y, fg)
        self.text(f'{self.motor:+5.0f}%', 210, y, fg)
        y += 40

        if self.telemetry is None:
            self.text('Telemetry: waiting for rover...', 15, y, bad)
        else:
            t = self.telemetry
            if not fresh:
                self.text('Telemetry: STALE (rover node not running?)', 15, y, bad)
            elif not t.pico_connected:
                self.text('Telemetry: Pico not connected', 15, y, bad)
            else:
                self.text('Telemetry', 15, y, good)
            y += 28
            a = t.acceleration
            for line in (
                f'Altitude:     {fmt(t.altitude)} m',
                f'Temperature:  {fmt(t.temperature)} °C',
                f'Pressure:     {fmt(t.pressure)} kPa',
                f'Accel:        <{fmt(a.x)}, {fmt(a.y)}, {fmt(a.z)}> m/s²',
                f'Pico:         {t.note}',
            ):
                self.text(line, 15, y, fg)
                y += 24

        self.text('Shift: fine   R: home   Enter: enable   Space: STOP   Esc: quit',
                  15, 435, dim, self.small)
        pygame.display.flip()

    def text(self, s, x, y, color, font=None):
        self.screen.blit((font or self.font).render(s, True, color), (x, y))


def fmt(value):
    return '--' if math.isnan(value) else f'{value:.2f}'


def main(args=None):
    rclpy.init(args=args)
    node = GSNode()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
