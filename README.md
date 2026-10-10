# AXIS � Robot Arm Studio

Interactive browser simulation of a planar robot arm with 2-DOF / 3-DOF modes, analytic IK, a gripper, collision detection and a simple teach-program workflow.

## Features

- Switchable **2-DOF / 3-DOF** planar arm
- Forward kinematics with manual joint sliders
- Analytic inverse kinematics
- Explicit **elbow-up / elbow-down** IK branch selection
- 3-DOF IK with requested TCP orientation
- Joint limits: J1 ±160°, J2 ±145°, J3 ±135°
- TCP trajectory visualization
- Open/close gripper and automatic pick-and-place
- Rectangular workspace obstacles
- Link-vs-obstacle collision detection with link thickness
- Motion preflight: unsafe joint-space trajectories are rejected before execution
- Manual motion collision guard
- Randomize / reset / disable obstacle layout
- **Teach Mode** with up to 8 stored poses
- Taught poses store joint configuration, arm DOF and gripper state
- **RUN PROGRAM** sequence replay
- Program stop control and automatic collision abort
- Live telemetry for TCP pose, branch, payload, collision and program state
- Responsive engineering-console UI for desktop, laptop, tablet and mobile

## Collision model

Each arm link is modeled as a line segment with a small safety radius. Rectangular obstacles are expanded by that radius before segment intersection checks. Motion commands are sampled along their joint-space interpolation path; if any sample intersects an obstacle, the motion is rejected.

This is intentionally a lightweight educational collision model, not a rigid-body physics engine.

## Teach Mode

1. Move the arm into a desired pose.
2. Set the gripper state if needed.
3. Press **SAVE POSE**.
4. Repeat for additional positions.
5. Press **RUN PROGRAM** to replay the sequence.

The collision guard remains active during program execution. Unsafe motions abort the sequence.

## Kinematics

For the 2R planar manipulator:

```text
x = L1 cos(theta1) + L2 cos(theta1 + theta2)
y = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

For the 3R arm, desired end-effector orientation `phi` defines a wrist target:

```text
x_w = x - L3 cos(phi)
y_w = y - L3 sin(phi)
theta3 = phi - theta1 - theta2
```

Solutions are rejected when joint limits or collision constraints are violated.

## Tech

Plain HTML, CSS and JavaScript with vendored Three.js r180. No build step, runtime CDN, framework or backend.


## AXIS visual studio

The Blender-built articulated arm follows the original planar solver in a Three.js workcell. Orbit with a pointer drag, zoom with the wheel or pinch gesture, and use Top for precise targeting. FK sliders and IK targets share the same model in the 3D studio and the original 2D plan. WebGL initialization failure or context loss falls back to 2D.

- `studio.js`: on-demand renderer, table, scene picking, camera and GLB pose synchronization.
- `assets/axis-arm.blend` / `assets/axis-arm.glb`: editable source and runtime model.
- `tools/build_axis.py`: reproducible Blender generator; stable J1 > J2 > J3 > TCP hierarchy.
- `docs/axis/index.html`: before/after gallery and a real browser video.
- `docs/axis-review.md`: verification results and modeling limits.

### Preview

From this repository run `python -m http.server 8780 --bind 127.0.0.1`, then open `http://127.0.0.1:8780/`. Serve over HTTP; ES modules do not support opening this app as a local file.

### Rebuild the model

`blender --background --python tools/build_axis.py --python-exit-code 1`

Requires Blender. The original export used Blender 5.2.2 LTS. The script writes only the model's source and GLB into `assets/`.

### Browser checks

Use an existing Playwright installation and browser; no browser download is required. Set `PLAYWRIGHT_MODULE` to its module path and `BROWSER_PATH` to the browser executable, then run `node tests/studio-browser.cjs` and `node tests/studio-focused.cjs` from this repository. `BASE_URL` overrides the default workspace preview URL. Optional `BASELINE_URL` enables comparison with the untouched original lab at commit `ad5fa36`.

The 3D elevations, enclosure and gripper geometry are presentation. Kinematics and collision remain planar; the finite motion samples are not continuous collision detection or industrial safety certification.
