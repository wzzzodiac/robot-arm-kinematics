# Robot Arm Kinematics Lab

Interactive browser simulation of a planar robot arm with 2-DOF / 3-DOF modes, analytic IK, a gripper, collision detection and a simple teach-program workflow.

## V3 features

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

Plain HTML, CSS and JavaScript. No framework and no backend.
