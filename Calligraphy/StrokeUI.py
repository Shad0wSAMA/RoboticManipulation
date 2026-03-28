import json
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

import pygame


# Window/layout constants
WINDOW_W = 1400
WINDOW_H = 860
TOP_BAR_H = 50
PANEL_W = 310
CANVAS_MARGIN = 30
MIN_WINDOW_W = 1000
MIN_WINDOW_H = 640

# Visual and interaction constants
BG_COLOR = (245, 247, 250)
CANVAS_BG = (255, 255, 255)
GRID_COLOR = (232, 236, 242)
POINT_COLOR = (40, 93, 170)
POINT_SELECTED_COLOR = (220, 76, 70)
LINE_COLOR = (112, 132, 160)
TEXT_COLOR = (32, 40, 52)
MUTED_TEXT = (95, 106, 122)
BORDER_COLOR = (180, 188, 202)
WIDGET_BG = (255, 255, 255)
BUTTON_BG = (237, 242, 250)
BUTTON_BG_HOVER = (223, 232, 247)

BASE_POINT_RADIUS = 11
MID_Z = 0.07
SELECT_HIT_RADIUS = 12
Z_STEP = 0.0025
RADIUS_STEP_Z = 0.0025
RADIUS_SCALE_PER_STEP = 1.2  # each 0.0025 lower z => +20% radius
ZOOM_STEP = 1.12
ZOOM_MIN = 0.2
ZOOM_MAX = 8.0
DEFAULT_VIEW_BOUNDS = (5.0, 9.0, -2.0, 2.0)  # xmin, xmax, ymin, ymax


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class ScreenPoint:
    stroke_idx: int
    point_idx: int
    sx: float
    sy: float


class InputBox:
    def __init__(self, x: int, y: int, w: int, h: int, label: str):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.text = ""
        self.active = False

    def set_text(self, value: float) -> None:
        self.text = f"{value:.4f}".rstrip("0").rstrip(".")

    def handle_event(self, event: pygame.event.Event) -> Optional[float]:
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)
        elif event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                return self._parse_value()
            if event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                if event.unicode and event.unicode in "0123456789.-":
                    self.text += event.unicode
        return None

    def _parse_value(self) -> Optional[float]:
        try:
            return float(self.text)
        except ValueError:
            return None

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, small_font: pygame.font.Font) -> None:
        pygame.draw.rect(surface, WIDGET_BG, self.rect, border_radius=6)
        pygame.draw.rect(surface, (80, 120, 200) if self.active else BORDER_COLOR, self.rect, 2, border_radius=6)
        label_surf = small_font.render(self.label, True, MUTED_TEXT)
        surface.blit(label_surf, (self.rect.x, self.rect.y - 16))
        txt_surf = font.render(self.text, True, TEXT_COLOR)
        surface.blit(txt_surf, (self.rect.x + 8, self.rect.y + 7))


class Button:
    def __init__(self, x: int, y: int, w: int, h: int, label: str):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label

    def is_hover(self) -> bool:
        return self.rect.collidepoint(pygame.mouse.get_pos())

    def handle_event(self, event: pygame.event.Event) -> bool:
        return event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        bg = BUTTON_BG_HOVER if self.is_hover() else BUTTON_BG
        pygame.draw.rect(surface, bg, self.rect, border_radius=8)
        pygame.draw.rect(surface, BORDER_COLOR, self.rect, 1, border_radius=8)
        txt = font.render(self.label, True, TEXT_COLOR)
        surface.blit(txt, txt.get_rect(center=self.rect.center))


class Dropdown:
    def __init__(self, x: int, y: int, w: int, h: int, title: str, multi: bool = False):
        self.rect = pygame.Rect(x, y, w, h)
        self.title = title
        self.multi = multi
        self.expanded = False
        self.options: List[Tuple[str, str]] = []
        self.selected_single: Optional[str] = None
        self.selected_multi = set()

    def set_options(self, options: List[Tuple[str, str]]) -> None:
        self.options = options
        keys = {k for k, _ in options}
        if self.multi:
            self.selected_multi = {k for k in self.selected_multi if k in keys}
        else:
            if self.selected_single not in keys:
                self.selected_single = options[0][0] if options else None

    def set_selected_all(self) -> None:
        if self.multi:
            self.selected_multi = {k for k, _ in self.options}

    def get_selected_label(self) -> str:
        if self.multi:
            n = len(self.selected_multi)
            total = len(self.options)
            return f"{self.title}: {n}/{total}"
        selected = next((label for k, label in self.options if k == self.selected_single), "None")
        return f"{self.title}: {selected}"

    def handle_event(self, event: pygame.event.Event) -> Optional[Tuple[str, str]]:
        # return: ("single_changed", key) or ("multi_changed", key)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.expanded = not self.expanded
                return None
            if self.expanded:
                for i, (k, _) in enumerate(self.options):
                    item_rect = pygame.Rect(self.rect.x, self.rect.bottom + i * self.rect.height, self.rect.width, self.rect.height)
                    if item_rect.collidepoint(event.pos):
                        if self.multi:
                            if k in self.selected_multi:
                                self.selected_multi.remove(k)
                            else:
                                self.selected_multi.add(k)
                            return ("multi_changed", k)
                        self.selected_single = k
                        self.expanded = False
                        return ("single_changed", k)
                self.expanded = False
        return None

    def option_rect(self, index: int) -> pygame.Rect:
        return pygame.Rect(self.rect.x, self.rect.bottom + index * self.rect.height, self.rect.width, self.rect.height)

    def contains(self, pos: Tuple[int, int]) -> bool:
        if self.rect.collidepoint(pos):
            return True
        if self.expanded:
            for i in range(len(self.options)):
                if self.option_rect(i).collidepoint(pos):
                    return True
        return False

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        pygame.draw.rect(surface, WIDGET_BG, self.rect, border_radius=6)
        pygame.draw.rect(surface, BORDER_COLOR, self.rect, 1, border_radius=6)
        txt = font.render(self.get_selected_label(), True, TEXT_COLOR)
        surface.blit(txt, (self.rect.x + 8, self.rect.y + 8))
        arrow = "^" if self.expanded else "v"
        arr = font.render(arrow, True, MUTED_TEXT)
        surface.blit(arr, (self.rect.right - 18, self.rect.y + 8))

        if not self.expanded:
            return

        for i, (k, label) in enumerate(self.options):
            r = self.option_rect(i)
            pygame.draw.rect(surface, WIDGET_BG, r)
            pygame.draw.rect(surface, BORDER_COLOR, r, 1)

            if self.multi:
                checked = k in self.selected_multi
                box = pygame.Rect(r.x + 8, r.y + 9, 14, 14)
                pygame.draw.rect(surface, (255, 255, 255), box)
                pygame.draw.rect(surface, BORDER_COLOR, box, 1)
                if checked:
                    pygame.draw.line(surface, (54, 100, 170), (box.x + 3, box.y + 8), (box.x + 6, box.y + 11), 2)
                    pygame.draw.line(surface, (54, 100, 170), (box.x + 6, box.y + 11), (box.x + 11, box.y + 3), 2)
                text_x = r.x + 28
            else:
                text_x = r.x + 8

            t = font.render(label, True, TEXT_COLOR)
            surface.blit(t, (text_x, r.y + 8))


class StrokeEditor:
    def __init__(self, root_dir: str):
        pygame.init()
        pygame.display.set_caption("Stroke JSON Editor")
        self.window_w = WINDOW_W
        self.window_h = WINDOW_H
        self.screen = pygame.display.set_mode((self.window_w, self.window_h), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)
        self.small_font = pygame.font.SysFont("consolas", 14)

        self.root_dir = root_dir
        self.file_dropdown = Dropdown(12, 10, 330, 30, "File", multi=False)
        self.stroke_dropdown = Dropdown(356, 10, 280, 30, "Strokes", multi=True)

        panel_x = self.window_w - PANEL_W + 16
        self.input_x = InputBox(panel_x, 120, PANEL_W - 32, 34, "x")
        self.input_y = InputBox(panel_x, 180, PANEL_W - 32, 34, "y")
        self.input_z = InputBox(panel_x, 240, PANEL_W - 32, 34, "z")
        self.add_btn = Button(panel_x, 300, PANEL_W - 32, 38, "New Point After")
        self.del_btn = Button(panel_x, 350, PANEL_W - 32, 38, "Delete Point")
        self.center_btn = Button(panel_x, 400, PANEL_W - 32, 38, "Center To (7,0)")
        self.scale_input = InputBox(panel_x, 450, PANEL_W - 32, 34, "scale factor")
        self.scale_input.text = "1.1"
        self.scale_btn = Button(panel_x, 500, PANEL_W - 32, 38, "Scale Around Center")

        self.current_path: Optional[str] = None
        self.data = {"strokes": []}
        self.selected: Optional[Tuple[int, int]] = None
        self.dragging = False
        self.drag_moved = False
        self.rotation_steps = 0  # 0/1/2/3 -> 0/90/180/270 deg clockwise
        self.zoom = 1.0
        self.view_offset = (0.0, 0.0)  # screen-space offset after rotate/zoom
        self.view_bounds = DEFAULT_VIEW_BOUNDS
        self.panning = False
        self.invert_y_axis = False

        self.canvas_rect = pygame.Rect(0, 0, 0, 0)
        self.panel_rect = pygame.Rect(0, 0, 0, 0)
        self.update_layout(self.window_w, self.window_h)

        self.refresh_file_list()

    def update_layout(self, width: int, height: int) -> None:
        self.window_w = max(width, MIN_WINDOW_W)
        self.window_h = max(height, MIN_WINDOW_H)

        self.canvas_rect = pygame.Rect(
            CANVAS_MARGIN,
            TOP_BAR_H + CANVAS_MARGIN,
            self.window_w - PANEL_W - CANVAS_MARGIN * 2,
            self.window_h - TOP_BAR_H - CANVAS_MARGIN * 2,
        )
        self.panel_rect = pygame.Rect(self.window_w - PANEL_W, TOP_BAR_H, PANEL_W, self.window_h - TOP_BAR_H)

        panel_x = self.panel_rect.x + 16
        self.input_x.rect.topleft = (panel_x, 120)
        self.input_y.rect.topleft = (panel_x, 180)
        self.input_z.rect.topleft = (panel_x, 240)
        self.add_btn.rect.topleft = (panel_x, 300)
        self.del_btn.rect.topleft = (panel_x, 350)
        self.center_btn.rect.topleft = (panel_x, 400)
        self.scale_input.rect.topleft = (panel_x, 450)
        self.scale_btn.rect.topleft = (panel_x, 500)

    def refresh_file_list(self) -> None:
        files = sorted([f for f in os.listdir(self.root_dir) if f.lower().endswith(".json")])
        opts = [(f, f) for f in files]
        self.file_dropdown.set_options(opts)
        if opts:
            if self.current_path is None:
                self.load_file(opts[0][0])

    def load_file(self, filename: str) -> None:
        path = os.path.join(self.root_dir, filename)
        with open(path, "r", encoding="utf-8") as f:
            self.data = json.load(f)

        self.current_path = path
        self.selected = None
        self.dragging = False
        self.drag_moved = False

        stroke_options = []
        for i, s in enumerate(self.data.get("strokes", [])):
            sid = s.get("stroke_id", i + 1)
            stroke_options.append((str(i), f"stroke_id={sid}"))

        self.stroke_dropdown.set_options(stroke_options)
        self.stroke_dropdown.set_selected_all()

    def save(self) -> None:
        if not self.current_path:
            return
        with open(self.current_path, "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=2, ensure_ascii=False)

    def selected_point_ref(self) -> Optional[List[float]]:
        if not self.selected:
            return None
        si, pi = self.selected
        strokes = self.data.get("strokes", [])
        if not (0 <= si < len(strokes)):
            return None
        pts = strokes[si].get("points", [])
        if not (0 <= pi < len(pts)):
            return None
        return pts[pi]

    def visible_strokes(self) -> List[int]:
        if not self.stroke_dropdown.options:
            return []
        return sorted([int(k) for k in self.stroke_dropdown.selected_multi])

    def world_bounds(self) -> Tuple[float, float, float, float]:
        return self.view_bounds

    def world_to_screen(self, x: float, y: float, bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
        xmin, xmax, ymin, ymax = bounds
        w = self.canvas_rect.width
        h = self.canvas_rect.height

        base_x = self.canvas_rect.x + (x - xmin) / (xmax - xmin) * w
        y_ratio = (y - ymin) / (ymax - ymin)
        if self.invert_y_axis:
            base_y = self.canvas_rect.y + y_ratio * h
        else:
            base_y = self.canvas_rect.bottom - y_ratio * h
        return self._apply_view_transform(base_x, base_y)

    def screen_to_world(self, sx: float, sy: float, bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
        xmin, xmax, ymin, ymax = bounds
        base_x, base_y = self._inverse_view_transform(sx, sy)
        x = xmin + (base_x - self.canvas_rect.x) / self.canvas_rect.width * (xmax - xmin)
        if self.invert_y_axis:
            y = ymin + (base_y - self.canvas_rect.y) / self.canvas_rect.height * (ymax - ymin)
        else:
            y = ymin + (self.canvas_rect.bottom - base_y) / self.canvas_rect.height * (ymax - ymin)
        return x, y

    def _apply_view_transform(self, px: float, py: float) -> Tuple[float, float]:
        cx, cy = self.canvas_rect.center
        dx = px - cx
        dy = py - cy

        if self.rotation_steps == 1:
            dx, dy = -dy, dx
        elif self.rotation_steps == 2:
            dx, dy = -dx, -dy
        elif self.rotation_steps == 3:
            dx, dy = dy, -dx

        return cx + self.view_offset[0] + dx * self.zoom, cy + self.view_offset[1] + dy * self.zoom

    def _inverse_view_transform(self, sx: float, sy: float) -> Tuple[float, float]:
        cx, cy = self.canvas_rect.center
        dx = (sx - cx - self.view_offset[0]) / self.zoom
        dy = (sy - cy - self.view_offset[1]) / self.zoom

        if self.rotation_steps == 1:
            dx, dy = dy, -dx
        elif self.rotation_steps == 2:
            dx, dy = -dx, -dy
        elif self.rotation_steps == 3:
            dx, dy = -dy, dx

        return cx + dx, cy + dy

    def zoom_at(self, mouse_pos: Tuple[int, int], zoom_in: bool) -> None:
        old_zoom = self.zoom
        if zoom_in:
            new_zoom = clamp(old_zoom * ZOOM_STEP, ZOOM_MIN, ZOOM_MAX)
        else:
            new_zoom = clamp(old_zoom / ZOOM_STEP, ZOOM_MIN, ZOOM_MAX)
        if abs(new_zoom - old_zoom) < 1e-9:
            return

        cx, cy = self.canvas_rect.center
        mx, my = mouse_pos
        qx = mx - cx - self.view_offset[0]
        qy = my - cy - self.view_offset[1]
        scale = new_zoom / old_zoom
        self.view_offset = (mx - cx - qx * scale, my - cy - qy * scale)
        self.zoom = new_zoom

    def build_screen_points(self, bounds: Tuple[float, float, float, float]) -> List[ScreenPoint]:
        out: List[ScreenPoint] = []
        for si in self.visible_strokes():
            pts = self.data["strokes"][si].get("points", [])
            for pi, p in enumerate(pts):
                sx, sy = self.world_to_screen(float(p[0]), float(p[1]), bounds)
                out.append(ScreenPoint(si, pi, sx, sy))
        return out

    def point_radius(self, z: float) -> int:
        # z larger => smaller radius; every 0.0025 z change => 20% radius change
        steps = (MID_Z - z) / RADIUS_STEP_Z
        r = BASE_POINT_RADIUS * (RADIUS_SCALE_PER_STEP ** steps)
        return int(clamp(r, 4, 40))

    def set_selected(self, sel: Optional[Tuple[int, int]]) -> None:
        self.selected = sel
        p = self.selected_point_ref()
        if p is None:
            return
        self.input_x.set_text(float(p[0]))
        self.input_y.set_text(float(p[1]))
        self.input_z.set_text(float(p[2]))

    def add_point_after_selected(self) -> None:
        if not self.selected:
            return
        si, pi = self.selected
        pts = self.data["strokes"][si].get("points", [])
        if not (0 <= pi < len(pts)):
            return

        cur = pts[pi]
        if pi + 1 < len(pts):
            nxt = pts[pi + 1]
            newp = [round((cur[0] + nxt[0]) / 2, 4), round((cur[1] + nxt[1]) / 2, 4), round((cur[2] + nxt[2]) / 2, 4)]
        else:
            newp = [float(cur[0]), float(cur[1]), float(cur[2])]

        pts.insert(pi + 1, newp)
        self.set_selected((si, pi + 1))
        self.save()

    def delete_selected(self) -> None:
        if not self.selected:
            return
        si, pi = self.selected
        pts = self.data["strokes"][si].get("points", [])
        if not (0 <= pi < len(pts)):
            return

        pts.pop(pi)
        if not pts:
            self.set_selected(None)
        else:
            self.set_selected((si, min(pi, len(pts) - 1)))
        self.save()

    def apply_inputs_to_selected(self) -> None:
        p = self.selected_point_ref()
        if p is None:
            return

        changed = False
        nx = self.input_x._parse_value()
        ny = self.input_y._parse_value()
        nz = self.input_z._parse_value()

        if nx is not None:
            p[0] = round(nx, 4)
            changed = True
        if ny is not None:
            p[1] = round(ny, 4)
            changed = True
        if nz is not None:
            p[2] = round(nz, 4)
            changed = True

        if changed:
            self.input_x.set_text(p[0])
            self.input_y.set_text(p[1])
            self.input_z.set_text(p[2])
            self.save()

    def center_all_points_to_target(self, target_x: float = 7.0, target_y: float = 0.0) -> None:
        xs: List[float] = []
        ys: List[float] = []
        for stroke in self.data.get("strokes", []):
            for p in stroke.get("points", []):
                xs.append(float(p[0]))
                ys.append(float(p[1]))

        if not xs:
            return

        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        dx = target_x - avg_x
        dy = target_y - avg_y

        for stroke in self.data.get("strokes", []):
            for p in stroke.get("points", []):
                p[0] = round(float(p[0]) + dx, 4)
                p[1] = round(float(p[1]) + dy, 4)

        selected = self.selected_point_ref()
        if selected is not None:
            self.input_x.set_text(float(selected[0]))
            self.input_y.set_text(float(selected[1]))

        self.save()

    def scale_all_points_about_center(self, scale: float) -> None:
        if scale <= 0:
            return

        xs: List[float] = []
        ys: List[float] = []
        for stroke in self.data.get("strokes", []):
            for p in stroke.get("points", []):
                xs.append(float(p[0]))
                ys.append(float(p[1]))

        if not xs:
            return

        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        for stroke in self.data.get("strokes", []):
            for p in stroke.get("points", []):
                px = float(p[0])
                py = float(p[1])
                p[0] = round(cx + (px - cx) * scale, 4)
                p[1] = round(cy + (py - cy) * scale, 4)

        selected = self.selected_point_ref()
        if selected is not None:
            self.input_x.set_text(float(selected[0]))
            self.input_y.set_text(float(selected[1]))

        self.save()

    def draw(self) -> None:
        self.screen.fill(BG_COLOR)

        # Top bar
        pygame.draw.rect(self.screen, (248, 250, 253), (0, 0, self.window_w, TOP_BAR_H))
        pygame.draw.line(self.screen, BORDER_COLOR, (0, TOP_BAR_H), (self.window_w, TOP_BAR_H), 1)

        # Canvas
        pygame.draw.rect(self.screen, CANVAS_BG, self.canvas_rect, border_radius=8)
        pygame.draw.rect(self.screen, BORDER_COLOR, self.canvas_rect, 1, border_radius=8)

        # Side panel
        panel_rect = self.panel_rect
        pygame.draw.rect(self.screen, (250, 251, 253), panel_rect)
        pygame.draw.line(self.screen, BORDER_COLOR, (panel_rect.x, TOP_BAR_H), (panel_rect.x, self.window_h), 1)

        bounds = self.world_bounds()
        prev_clip = self.screen.get_clip()
        self.screen.set_clip(self.canvas_rect)
        self.draw_grid()
        self.draw_strokes(bounds)
        self.screen.set_clip(prev_clip)

        title = self.font.render("Selected Point", True, TEXT_COLOR)
        self.screen.blit(title, (panel_rect.x + 16, 84))

        if self.selected_point_ref() is None:
            msg = self.small_font.render("Click a point to edit x/y/z", True, MUTED_TEXT)
            self.screen.blit(msg, (panel_rect.x + 16, 118))
        else:
            self.input_x.draw(self.screen, self.font, self.small_font)
            self.input_y.draw(self.screen, self.font, self.small_font)
            self.input_z.draw(self.screen, self.font, self.small_font)
            self.add_btn.draw(self.screen, self.font)
            self.del_btn.draw(self.screen, self.font)

            si, pi = self.selected
            sid = self.data["strokes"][si].get("stroke_id", si + 1)
            info = self.small_font.render(f"stroke_id={sid}, index={pi + 1}", True, MUTED_TEXT)
            self.screen.blit(info, (panel_rect.x + 16, 404))

        self.center_btn.draw(self.screen, self.font)
        self.scale_input.draw(self.screen, self.font, self.small_font)
        self.scale_btn.draw(self.screen, self.font)

        help1 = self.small_font.render("Drag: move x/y | Wheel: +/- z (when selected)", True, MUTED_TEXT)
        help2 = self.small_font.render("No selection wheel: zoom | R: rotate 90deg | Y: flip Y", True, MUTED_TEXT)
        help3 = self.small_font.render("Enter in box: press Enter to apply", True, MUTED_TEXT)
        self.screen.blit(help1, (panel_rect.x + 16, self.window_h - 56))
        self.screen.blit(help2, (panel_rect.x + 16, self.window_h - 36))
        self.screen.blit(help3, (panel_rect.x + 16, self.window_h - 16))

        # Draw dropdowns last so expanded lists stay above canvas/panel.
        self.file_dropdown.draw(self.screen, self.font)
        self.stroke_dropdown.draw(self.screen, self.font)

        pygame.display.flip()

    def draw_grid(self) -> None:
        for i in range(1, 10):
            x = self.canvas_rect.x + i * self.canvas_rect.width / 10
            y = self.canvas_rect.y + i * self.canvas_rect.height / 10
            pygame.draw.line(self.screen, GRID_COLOR, (x, self.canvas_rect.y), (x, self.canvas_rect.bottom), 1)
            pygame.draw.line(self.screen, GRID_COLOR, (self.canvas_rect.x, y), (self.canvas_rect.right, y), 1)

    def draw_strokes(self, bounds: Tuple[float, float, float, float]) -> None:
        for si in self.visible_strokes():
            pts = self.data["strokes"][si].get("points", [])
            screen_pts = [self.world_to_screen(float(p[0]), float(p[1]), bounds) for p in pts]

            for i in range(len(screen_pts) - 1):
                pygame.draw.line(self.screen, LINE_COLOR, screen_pts[i], screen_pts[i + 1], 2)

            for pi, p in enumerate(pts):
                sx, sy = screen_pts[pi]
                is_sel = self.selected == (si, pi)
                radius = self.point_radius(float(p[2]))
                color = POINT_SELECTED_COLOR if is_sel else POINT_COLOR
                pygame.draw.circle(self.screen, color, (int(sx), int(sy)), radius)
                pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), radius, 2)

                # sequence number
                idx_text = self.small_font.render(str(pi + 1), True, (255, 255, 255))
                self.screen.blit(idx_text, idx_text.get_rect(center=(sx, sy)))

                # xy label next to point
                xy = self.small_font.render(f"({p[0]:.3f},{p[1]:.3f})", True, TEXT_COLOR)
                self.screen.blit(xy, (sx + radius + 4, sy - 8))

    def find_hit_point(self, mouse_pos: Tuple[int, int], bounds: Tuple[float, float, float, float]) -> Optional[Tuple[int, int]]:
        closest = None
        closest_dist = 1e9
        for sp in self.build_screen_points(bounds):
            d = math.hypot(mouse_pos[0] - sp.sx, mouse_pos[1] - sp.sy)
            if d <= SELECT_HIT_RADIUS and d < closest_dist:
                closest = (sp.stroke_idx, sp.point_idx)
                closest_dist = d
        return closest

    def handle_event(self, event: pygame.event.Event) -> bool:
        change = self.file_dropdown.handle_event(event)
        if change and change[0] == "single_changed":
            self.load_file(change[1])
            return True

        self.stroke_dropdown.handle_event(event)

        in_x = self.input_x.handle_event(event)
        in_y = self.input_y.handle_event(event)
        in_z = self.input_z.handle_event(event)
        in_scale = self.scale_input.handle_event(event)
        if in_x is not None or in_y is not None or in_z is not None:
            self.apply_inputs_to_selected()
        if in_scale is not None:
            self.scale_all_points_about_center(in_scale)
            self.scale_input.set_text(in_scale)

        if self.add_btn.handle_event(event):
            self.add_point_after_selected()
        if self.del_btn.handle_event(event):
            self.delete_selected()
        if self.center_btn.handle_event(event):
            self.center_all_points_to_target(7.0, 0.0)
        if self.scale_btn.handle_event(event):
            scale = self.scale_input._parse_value()
            if scale is not None:
                self.scale_all_points_about_center(scale)
                self.scale_input.set_text(scale)

        bounds = self.world_bounds()

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.canvas_rect.collidepoint(event.pos):
                hit = self.find_hit_point(event.pos, bounds)
                if hit is None:
                    self.set_selected(None)
                else:
                    self.set_selected(hit)
                    self.dragging = True
                    self.drag_moved = False
            else:
                over_ui = (
                    self.file_dropdown.contains(event.pos)
                    or self.stroke_dropdown.contains(event.pos)
                    or self.input_x.rect.collidepoint(event.pos)
                    or self.input_y.rect.collidepoint(event.pos)
                    or self.input_z.rect.collidepoint(event.pos)
                    or self.add_btn.rect.collidepoint(event.pos)
                    or self.del_btn.rect.collidepoint(event.pos)
                    or self.center_btn.rect.collidepoint(event.pos)
                    or self.scale_input.rect.collidepoint(event.pos)
                    or self.scale_btn.rect.collidepoint(event.pos)
                )
                if not over_ui:
                    self.set_selected(None)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
            if self.canvas_rect.collidepoint(event.pos):
                self.panning = True

        elif event.type == pygame.MOUSEMOTION:
            if self.panning:
                ox, oy = self.view_offset
                self.view_offset = (ox + event.rel[0], oy + event.rel[1])
            elif self.dragging and self.selected and self.canvas_rect.collidepoint(event.pos):
                p = self.selected_point_ref()
                if p is not None:
                    wx, wy = self.screen_to_world(event.pos[0], event.pos[1], bounds)
                    p[0] = round(wx, 4)
                    p[1] = round(wy, 4)
                    self.input_x.set_text(p[0])
                    self.input_y.set_text(p[1])
                    self.drag_moved = True

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self.dragging and self.drag_moved:
                self.save()
            self.dragging = False
            self.drag_moved = False
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 2:
            self.panning = False

        elif event.type == pygame.MOUSEWHEEL:
            p = self.selected_point_ref()
            if p is not None:
                p[2] = round(float(p[2]) + (Z_STEP if event.y > 0 else -Z_STEP), 4)
                self.input_z.set_text(p[2])
                self.save()
            else:
                mouse_pos = pygame.mouse.get_pos()
                if event.y > 0:
                    self.zoom_at(mouse_pos, zoom_in=True)
                elif event.y < 0:
                    self.zoom_at(mouse_pos, zoom_in=False)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                self.rotation_steps = (self.rotation_steps + 1) % 4
            elif event.key == pygame.K_y:
                self.invert_y_axis = not self.invert_y_axis
        elif event.type in (pygame.VIDEORESIZE, pygame.WINDOWRESIZED):
            if hasattr(event, "w") and hasattr(event, "h"):
                new_w, new_h = event.w, event.h
            else:
                new_w, new_h = event.x, event.y
            new_w = max(new_w, MIN_WINDOW_W)
            new_h = max(new_h, MIN_WINDOW_H)
            self.screen = pygame.display.set_mode((new_w, new_h), pygame.RESIZABLE)
            self.update_layout(new_w, new_h)

        return True

    def run(self) -> None:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                else:
                    self.handle_event(event)

            self.draw()
            self.clock.tick(60)

        pygame.quit()


def main() -> None:
    root = os.path.dirname(os.path.abspath(__file__))
    app = StrokeEditor(root)
    app.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}")
        pygame.quit()
        sys.exit(1)
