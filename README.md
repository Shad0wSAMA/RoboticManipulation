# RobotController README

## Overview
`RobotController` is a control stack for a 4-DOF Dynamixel robotic arm with gripper, focused on:
- reliable point-to-point motion (`GotoPoint` family)
- task-level automation (Task 2/Task 3)
- trajectory execution for manipulation and calligraphy

## Most Important Functions

### 1) `GotoPoint` (core motion primitive)
`GotoPoint` is the key interface used across tasks. Given a Cartesian target (and tool pose constraints), it computes a feasible joint solution and executes a smooth move.

Why it matters:
- Every high-level behavior (pick/place, gates, drawing) is built from repeated `GotoPoint` calls.
- It encapsulates IK + trajectory timing + joint-limit safety in one command.

Typical behavior in the controller:
- Validate target reachability and joint limits.
- Solve inverse kinematics (analytical / Jacobian variant depending on mode).
- Generate time-parameterized joint motion (accel, cruise, decel).
- Send synchronized motor commands so joints arrive together.

Related variants used in the project:
- `GotoPoint_Angle()`: Cartesian target + end-effector angle with analytic IK.
- `GotoPoint_Jacobian()`: iterative Jacobian/DLS solver for continuous correction.
- `GotoPoint_k_points()`: multi-waypoint Cartesian interpolation.

### 2) Task Orchestration Functions
Task functions are where motion primitives become complete robot behaviors.

- `task2a()`:
  deterministic pick-and-place to predefined targets.
  Emphasis: repeatability and baseline block transport.

- `task2b()`:
  extended manipulation logic (orientation/stacking/conditional placement).
  Emphasis: composing multiple primitives into robust sequences.

- Gate/route tasks (`pass_gate()`, `pass_all_gates()`):
  navigation through constrained waypoints while maintaining safe approach/clearance.

- Combined routines (for example `imaginaryDragon()`):
  end-to-end scripted demos that chain pickup, transport, drawing, and placement.

## Task 3: Calligraphy and `StrokeUI.py`
Task 3 is the calligraphy pipeline. `StrokeUI.py` is the authoring/editing tool for stroke trajectories, and the controller executes those trajectories on the robot.

### A) What `StrokeUI.py` does
`Calligraphy/StrokeUI.py` is a Pygame-based stroke editor for JSON trajectory files.

Core capabilities:
- Load/select stroke JSON files from the `Calligraphy` folder.
- Visualize each stroke as ordered points and connected segments.
- Select, drag, and numerically edit per-point `x`, `y`, `z`.
- Add or delete points to refine stroke geometry.
- Adjust global layout:
  - center all points to `(7, 0)`
  - scale all points about dataset center
- View transforms for editing convenience:
  - zoom/pan
  - rotate view (`R`)
  - flip Y-axis (`Y`)
- Save edits back to JSON.

### B) Stroke JSON model
The editor uses a stroke list, where each stroke contains ordered 3D points:

```json
{
  "strokes": [
    {
      "stroke_id": 1,
      "points": [[x, y, z], [x, y, z], ...]
    }
  ]
}
```

Execution meaning:
- `x, y`: planar path of each calligraphy segment.
- `z`: pen height/pressure proxy.
  higher `z` -> lift/travel,
  lower `z` -> contact/thicker stroke intent.

### C) How Task 3 drawing works on robot
High-level execution flow:
1. Prepare stroke JSON in `StrokeUI.py` (shape, spacing, stroke order, lift heights).
2. Load trajectory in controller (for example through `draw_from_json()`).
3. For each stroke:
   - move to pre-stroke safe height,
   - descend to writing `z`,
   - trace points in order using `GotoPoint`/path traversal,
   - lift before moving to next stroke.
4. Optional full routine (`draw_full()`): pick pen, draw all strokes, return pen.

Why this is reliable:
- Stroke order is explicit in JSON.
- Pen-up/pen-down is encoded directly in point `z` values.
- The same `GotoPoint` safety/kinematics stack used in manipulation tasks also drives drawing.

### D) Practical tuning notes for Task 3
- Keep adjacent points dense enough for smooth curves.
- Use clear lift `z` between disconnected strokes to avoid unwanted marks.
- Re-center and scale in `StrokeUI.py` before execution to match reachable workspace.
- Validate first stroke slowly, then increase speed once contact quality is stable.

## Minimal Usage
1. Connect Dynamixel motors.
2. Verify a simple `GotoPoint` move.
3. Run Task 2 (`task2a`/`task2b`) or Task 3 calligraphy (`draw_from_json` / `draw_full`).
4. Monitor trajectory and adjust stroke JSON in `Calligraphy/StrokeUI.py` as needed.
