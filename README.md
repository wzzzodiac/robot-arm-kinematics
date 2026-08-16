# Robot Arm Kinematics Lab

Interactive browser simulation of a planar robot arm with 2-DOF / 3-DOF modes, a gripper and a small pick-and-place task.

## V2 features

- Switchable **2-DOF / 3-DOF** planar arm
- Forward kinematics with manual joint sliders
- Analytic inverse kinematics
- Explicit **elbow-up / elbow-down** IK branch selection
- 3-DOF IK with requested TCP orientation
- Joint limits:
  - J1: ±160°
  - J2: ±145°
  - J3: ±135°
- Click-to-target IK control
- Adjustable link lengths
- Reachable-workspace visualization
- TCP trajectory / path trace
- Open/close gripper logic
- Orientation tolerance for 3-DOF grasping
- Selectable movable parts
- Automatic pick-and-place cycle
- Reachability and joint-limit checking
- Live telemetry for joint angles, TCP pose, target error, elbow branch, gripper state and payload
- Responsive engineering-console UI for desktop, laptop, tablet and mobile

## Controls

- **FK / MANUAL**: directly move the available joint sliders.
- **IK / TARGET**: click inside the simulation plane to command a Cartesian target.
- **2-DOF / 3-DOF**: change the arm model.
- **ELBOW UP / ELBOW DOWN**: choose the analytic IK branch.
- **TCP φ**: in 3-DOF IK mode, request the end-effector orientation.
- Click a movable part to select it.
- **CLOSE GRIPPER**: grabs a selected/nearby part if position and orientation tolerances are satisfied.
- **AUTO PICK + PLACE**: moves the selected part into the marked drop zone.
- **CLEAR TCP TRAIL**: clears the end-effector trajectory from the workspace.

## Model

For the 2R planar manipulator:

```text
x = L1 cos(theta1) + L2 cos(theta1 + theta2)
y = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

For the 3R arm, the desired end-effector orientation `phi` defines a wrist target:

```text
x_w = x - L3 cos(phi)
y_w = y - L3 sin(phi)
```

The first two joints solve the wrist position analytically. The third joint is then:

```text
theta3 = phi - theta1 - theta2
```

Each solution is rejected if it violates the configured joint limits.

## Tech

Plain HTML, CSS and JavaScript. No framework and no backend.
