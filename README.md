# Robot Arm Kinematics Lab

Interactive browser simulation of a 2-DOF planar robot arm with a gripper and a small pick-and-place task.

## Features

- Forward kinematics with manual joint sliders
- Analytic inverse kinematics for a 2-link arm
- Click-to-target IK control
- Adjustable link lengths
- Open/close gripper logic
- Selectable movable parts
- Automatic pick-and-place cycle
- Reachability checking
- Live telemetry for joint angles, TCP position, target error, gripper state and payload
- Responsive engineering-console UI for desktop, laptop, tablet and mobile

## Controls

- **FK / MANUAL**: move Joint 1 and Joint 2 directly.
- **IK / TARGET**: click inside the simulation plane to command a Cartesian target.
- Click a movable part to select it.
- **CLOSE GRIPPER**: grabs a selected/nearby part if the TCP is close enough.
- **AUTO PICK + PLACE**: moves the selected part into the marked drop zone.

## Model

For the planar two-link manipulator:

```text
x = L1 cos(theta1) + L2 cos(theta1 + theta2)
y = L1 sin(theta1) + L2 sin(theta1 + theta2)
```

Inverse kinematics is solved analytically from the requested XY target. Unreachable targets are rejected.

## Tech

Plain HTML, CSS and JavaScript. No framework and no backend.
