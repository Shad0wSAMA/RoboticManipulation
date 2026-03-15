import argparse
import json
import math
import time
from typing import List, Optional, Tuple

import paho.mqtt.client as mqtt
import pygame


Point = Tuple[float, float]
DISPLAY_SWAP_XY = True
OUTPUT_FLIP_Y = True


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


class TimeStrokeSampler:
    def __init__(self, sample_hz: float) -> None:
        self.sample_hz = max(1e-6, sample_hz)
        self.sample_period = 1.0 / self.sample_hz
        self.stroke_points_px: List[Point] = []

        self._last_cursor_px: Optional[Point] = None
        self._last_cursor_time: Optional[float] = None
        self._next_sample_time: Optional[float] = None

    def start(self, pos: Point, t: float) -> None:
        self.stroke_points_px = [pos]
        self._last_cursor_px = pos
        self._last_cursor_time = t
        self._next_sample_time = t + self.sample_period

    def add_cursor(self, pos: Point, t: float) -> None:
        if self._last_cursor_px is None or self._last_cursor_time is None:
            self.start(pos, t)
            return

        p0 = self._last_cursor_px
        t0 = self._last_cursor_time
        p1 = pos
        t1 = t

        if t1 <= t0:
            self._last_cursor_px = pos
            self._last_cursor_time = t
            return

        while self._next_sample_time is not None and self._next_sample_time <= t1:
            alpha = (self._next_sample_time - t0) / (t1 - t0)
            alpha = max(0.0, min(1.0, alpha))
            sx = lerp(p0[0], p1[0], alpha)
            sy = lerp(p0[1], p1[1], alpha)
            self.stroke_points_px.append((sx, sy))
            self._next_sample_time += self.sample_period

        self._last_cursor_px = pos
        self._last_cursor_time = t

    def tick(self, t: float) -> None:
        if self._last_cursor_px is None or self._next_sample_time is None:
            return

        while self._next_sample_time <= t:
            self.stroke_points_px.append(self._last_cursor_px)
            self._next_sample_time += self.sample_period

    def end(self, t: float) -> List[Point]:
        self.tick(t)

        if self._last_cursor_px is not None:
            last = self._last_cursor_px
            if not self.stroke_points_px:
                self.stroke_points_px.append(last)
            elif (
                abs(last[0] - self.stroke_points_px[-1][0]) > 1e-6
                or abs(last[1] - self.stroke_points_px[-1][1]) > 1e-6
            ):
                self.stroke_points_px.append(last)

        pts = self.stroke_points_px
        self.stroke_points_px = []
        self._last_cursor_px = None
        self._last_cursor_time = None
        self._next_sample_time = None
        return pts


def map_screen_to_robot(
    p: Point,
    workspace_rect: pygame.Rect,
    robot_x_min: float,
    robot_x_max: float,
    robot_y_min: float,
    robot_y_max: float,
    display_swap_xy: bool = False,
) -> Point:
    sx, sy = p
    nx = (sx - workspace_rect.left) / max(1, workspace_rect.width)
    ny = (sy - workspace_rect.top) / max(1, workspace_rect.height)
    nx = max(0.0, min(1.0, nx))
    ny = max(0.0, min(1.0, ny))

    if display_swap_xy:
        rx = robot_x_max - ny * (robot_x_max - robot_x_min)
        ry = robot_y_min + nx * (robot_y_max - robot_y_min)
    else:
        rx = robot_x_min + nx * (robot_x_max - robot_x_min)
        ry = robot_y_max - ny * (robot_y_max - robot_y_min)

    if OUTPUT_FLIP_Y:
        ry = robot_y_min + robot_y_max - ry

    return rx, ry


def map_robot_to_screen(
    p: Point,
    workspace_rect: pygame.Rect,
    robot_x_min: float,
    robot_x_max: float,
    robot_y_min: float,
    robot_y_max: float,
    display_swap_xy: bool = False,
) -> Tuple[int, int]:
    rx, ry = p

    if OUTPUT_FLIP_Y:
        ry = robot_y_min + robot_y_max - ry

    if display_swap_xy:
        nx = (ry - robot_y_min) / max(1e-9, (robot_y_max - robot_y_min))
        ny = (robot_x_max - rx) / max(1e-9, (robot_x_max - robot_x_min))
    else:
        nx = (rx - robot_x_min) / max(1e-9, (robot_x_max - robot_x_min))
        ny = (robot_y_max - ry) / max(1e-9, (robot_y_max - robot_y_min))

    sx = workspace_rect.left + int(round(nx * workspace_rect.width))
    sy = workspace_rect.top + int(round(ny * workspace_rect.height))
    return sx, sy


def compute_workspace_rect(
    screen_width: int,
    screen_height: int,
    robot_x_min: float,
    robot_x_max: float,
    robot_y_min: float,
    robot_y_max: float,
    display_swap_xy: bool = False,
    margin: int = 30,
    top_reserved: int = 40,
) -> pygame.Rect:
    avail_w = max(80, screen_width - 2 * margin)
    avail_h = max(80, screen_height - margin - top_reserved)

    if display_swap_xy:
        world_w = max(1e-9, robot_y_max - robot_y_min)
        world_h = max(1e-9, robot_x_max - robot_x_min)
    else:
        world_w = max(1e-9, robot_x_max - robot_x_min)
        world_h = max(1e-9, robot_y_max - robot_y_min)

    scale = min(avail_w / world_w, avail_h / world_h)

    rect_w = int(round(world_w * scale))
    rect_h = int(round(world_h * scale))
    left = (screen_width - rect_w) // 2
    top = top_reserved + (avail_h - rect_h) // 2
    return pygame.Rect(left, top, rect_w, rect_h)


def clamp_point_to_rect(p: Point, rect: pygame.Rect) -> Point:
    x, y = p
    cx = min(max(x, rect.left), rect.right - 1)
    cy = min(max(y, rect.top), rect.bottom - 1)
    return float(cx), float(cy)


def build_payload(stroke_id: int, robot_points: List[Point], sample_hz: float) -> str:
    payload = {
        "stroke_id": stroke_id,
        "timestamp": time.time(),
        "point_count": len(robot_points),
        "sample_hz": sample_hz,
        "dt": 1.0 / max(1e-6, sample_hz),
        "points": [[round(x, 4), round(y, 4)] for x, y in robot_points],
    }
    return json.dumps(payload, ensure_ascii=False)


def build_workspace_grid(
    screen_width: int,
    screen_height: int,
    workspace_rect: pygame.Rect,
    robot_x_min: float,
    robot_x_max: float,
    robot_y_min: float,
    robot_y_max: float,
    display_swap_xy: bool = False,
) -> pygame.Surface:
    surf = pygame.Surface((screen_width, screen_height))
    surf.fill((235, 238, 242))
    pygame.draw.rect(surf, (248, 250, 252), workspace_rect)

    grid_color = (220, 225, 230)

    x = math.ceil(robot_x_min)
    while x <= math.floor(robot_x_max):
        p1 = map_robot_to_screen(
            (x, robot_y_min),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        p2 = map_robot_to_screen(
            (x, robot_y_max),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        pygame.draw.line(surf, grid_color, p1, p2, 1)
        x += 1

    y = math.ceil(robot_y_min)
    while y <= math.floor(robot_y_max):
        p1 = map_robot_to_screen(
            (robot_x_min, y),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        p2 = map_robot_to_screen(
            (robot_x_max, y),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        pygame.draw.line(surf, grid_color, p1, p2, 1)
        y += 1

    pygame.draw.rect(surf, (120, 130, 140), workspace_rect, 1)

    if robot_x_min <= 0.0 <= robot_x_max:
        p1 = map_robot_to_screen(
            (0.0, robot_y_min),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        p2 = map_robot_to_screen(
            (0.0, robot_y_max),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        pygame.draw.line(surf, (80, 110, 220), p1, p2, 1)

    if robot_y_min <= 0.0 <= robot_y_max:
        p1 = map_robot_to_screen(
            (robot_x_min, 0.0),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        p2 = map_robot_to_screen(
            (robot_x_max, 0.0),
            workspace_rect,
            robot_x_min,
            robot_x_max,
            robot_y_min,
            robot_y_max,
            display_swap_xy,
        )
        pygame.draw.line(surf, (80, 110, 220), p1, p2, 1)

    return surf


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Pygame drawing -> robot points -> MQTT")
    parser.add_argument("--host", default="127.0.0.1", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--topic", default="robot/trajectory/add", help="MQTT topic")
    parser.add_argument("--client-id", default="pygame_draw_sender", help="MQTT client id")
    parser.add_argument("--username", default=None, help="MQTT username")
    parser.add_argument("--password", default=None, help="MQTT password")
    parser.add_argument("--width", type=int, default=1000, help="Window width")
    parser.add_argument("--height", type=int, default=700, help="Window height")
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=20,
        help="Fixed point sampling frequency in Hz",
    )
    parser.add_argument("--robot-x-min", type=float, default=4.0)
    parser.add_argument("--robot-x-max", type=float, default=9.0)
    parser.add_argument("--robot-y-min", type=float, default=-8.0)
    parser.add_argument("--robot-y-max", type=float, default=8.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    mqtt_client = mqtt.Client(client_id=args.client_id, protocol=mqtt.MQTTv311)
    if args.username:
        mqtt_client.username_pw_set(args.username, args.password)

    try:
        mqtt_client.connect(args.host, args.port, keepalive=30)
        mqtt_client.loop_start()
        print(f"[MQTT] Connected to {args.host}:{args.port}, topic={args.topic}")
    except Exception as exc:
        print(f"[MQTT] Connect failed: {exc}")
        return

    pygame.init()
    screen = pygame.display.set_mode((args.width, args.height))
    pygame.display.set_caption("Robot Draw Sender")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)

    workspace_rect = compute_workspace_rect(
        args.width,
        args.height,
        args.robot_x_min,
        args.robot_x_max,
        args.robot_y_min,
        args.robot_y_max,
        DISPLAY_SWAP_XY,
    )

    canvas = pygame.Surface((args.width, args.height))
    canvas.blit(
        build_workspace_grid(
            args.width,
            args.height,
            workspace_rect,
            args.robot_x_min,
            args.robot_x_max,
            args.robot_y_min,
            args.robot_y_max,
            DISPLAY_SWAP_XY,
        ),
        (0, 0),
    )

    sampled_overlay = pygame.Surface((args.width, args.height), pygame.SRCALPHA)

    sampler = TimeStrokeSampler(sample_hz=args.sample_hz)
    drawing = False
    stroke_id = 0
    all_strokes = []
    save_counter = 0
    running = True

    while running:
        now = time.perf_counter()

        if drawing:
            sampler.tick(now)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    canvas.blit(
                        build_workspace_grid(
                            args.width,
                            args.height,
                            workspace_rect,
                            args.robot_x_min,
                            args.robot_x_max,
                            args.robot_y_min,
                            args.robot_y_max,
                            DISPLAY_SWAP_XY,
                        ),
                        (0, 0),
                    )
                    sampled_overlay.fill((0, 0, 0, 0))
                    all_strokes = []
                    print("[CLEAR] Canvas and stored strokes cleared")

                elif event.key == pygame.K_s:
                    save_counter += 1
                    filename = f"strokes{save_counter}.json"
                    with open(filename, "w", encoding="utf-8") as f:
                        json.dump({"strokes": all_strokes}, f, indent=2, ensure_ascii=False)
                    print(f"[SAVE] Saved {len(all_strokes)} strokes to {filename}")

                elif event.key == pygame.K_ESCAPE:
                    running = False

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if not workspace_rect.collidepoint(event.pos):
                    continue

                drawing = True
                start_pos = clamp_point_to_rect(event.pos, workspace_rect)
                sampler.start(start_pos, time.perf_counter())

            elif event.type == pygame.MOUSEMOTION and drawing:
                prev = sampler._last_cursor_px
                curr = clamp_point_to_rect(event.pos, workspace_rect)
                sampler.add_cursor(curr, time.perf_counter())

                if prev is not None:
                    pygame.draw.line(canvas, (20, 20, 20), prev, curr, 3)

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1 and drawing:
                curr = clamp_point_to_rect(event.pos, workspace_rect)
                sampler.add_cursor(curr, time.perf_counter())
                drawing = False

                px_points = sampler.end(time.perf_counter())

                if px_points:
                    stroke_id += 1

                    for sx, sy in px_points:
                        pygame.draw.circle(
                            sampled_overlay,
                            (220, 50, 50, 210),
                            (int(round(sx)), int(round(sy))),
                            4,
                        )

                    robot_points = [
                        map_screen_to_robot(
                            p,
                            workspace_rect,
                            args.robot_x_min,
                            args.robot_x_max,
                            args.robot_y_min,
                            args.robot_y_max,
                            DISPLAY_SWAP_XY,
                        )
                        for p in px_points
                    ]

                    msg = build_payload(stroke_id, robot_points, args.sample_hz)

                    print(f"[STROKE] id={stroke_id} points={len(robot_points)}")
                    print(robot_points)
                    print(f"[JSON] {msg}")

                    result = mqtt_client.publish(args.topic, payload=msg, qos=0, retain=False)
                    if result.rc == mqtt.MQTT_ERR_SUCCESS:
                        print(
                            f"[PUB] stroke={stroke_id} points={len(robot_points)} "
                            f"topic={args.topic}"
                        )
                    else:
                        print(f"[PUB] failed stroke={stroke_id}, rc={result.rc}")

                    all_strokes.append(
                        {
                            "stroke_id": stroke_id,
                            "sample_hz": args.sample_hz,
                            "dt": 1.0 / max(1e-6, args.sample_hz),
                            "points": [
                                [round(x, 4), round(y, 4), 0.07] for x, y in robot_points
                            ],
                        }
                    )

        screen.fill((240, 240, 240))
        screen.blit(canvas, (0, 0))
        screen.blit(sampled_overlay, (0, 0))

        hint = "LMB draw | C clear | S save to JSON | ESC quit"
        info = (
            f"{hint} | sample={args.sample_hz:.1f}Hz | "
            f"map x:[{args.robot_x_min},{args.robot_x_max}] "
            f"y:[{args.robot_y_min},{args.robot_y_max}]"
        )
        screen.blit(font.render(info, True, (50, 50, 50)), (12, 10))

        pygame.display.flip()
        clock.tick(120)

    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    pygame.quit()


if __name__ == "__main__":
    main()