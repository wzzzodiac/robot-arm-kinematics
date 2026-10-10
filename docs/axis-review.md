# AXIS visual review — 2026-10-10

Review branch: `codex/robot-arm-axis-studio`. Production and the Hub are unchanged until approval.

## Experience

A warm, industrial robotics workbench replaces the original dark console. The simulation occupies the main surface; motion controls sit beside it, with telemetry and teaching below. The scene contains a perforated calibration table, physically shaded enamel/metal arm, layered joints, articulated gripper, selectable workpieces, obstacle fixtures, drop zone and TCP trail. Orbit, top view, zoom and 2D plan share one solver state.

Blender 5.2.2 was executed to create and export the asset. The GLB loaded in Three.js has `AxisRoot > J1 > J2 > J3 > TCP`, scalable link bodies and separate finger pivots. Blender could not write its optional user thumbnail in the sandbox; both the saved `.blend` and GLB export completed and the actual GLB was validated in the browser.

## Functional evidence

`axis/verification.json` records 33 browser checks; `axis/final-checks.json` records eight focused checks and performance samples. All passed in Edge 155.0.4283.45 on Windows, with no unexpected page/console errors or failed resource requests in the main suite.

- 24 imported-rig poses across both DOFs, angle limits and three link-length sets matched analytic FK. Maximum TCP discrepancy: 8.29e-14 solver units.
- 96 original-vs-AXIS comparisons of FK, IK and obstacle collision results matched exactly. Baseline: `ad5fa36`.
- Verified 3D part selection, IK reach/rejection, gripper capture/release, full automatic pick/place with obstacles enabled, taught sequence playback, stop, eight-pose cap, delete/clear, 2R/3R conversion, obstacle reset/randomization and 2D/3D state continuity.
- Verified mouse orbit/zoom, mobile-emulated touch targeting at 390×844, no horizontal overflow at 320/390/768/1024/1440/1920 px, and letterboxed 2D target coordinates.
- WebGL unavailability and context loss activate the working 2D view. Idle and hidden-plan 3D rendering stops; repeated pose updates held GPU allocations at 109 geometries and six textures.
- Fixed an existing manual-slider collision gap: a 30° → 15° jump has free endpoints but crosses an obstacle near 20°. Manual moves now sample the intervening joint path and reject that jump.
- Fixed held-payload synchronization when link lengths or DOF change.

## Performance and limits

A 2.2-second commanded movement measured approximately 120 fps and 8.5 ms P95 animation-frame intervals in both desktop and mobile viewport emulation. Hardware: NVIDIA RTX 4070 SUPER, ANGLE/D3D11. The renderer caps pixel ratio at 1.75. A representative frame reported 105 draw calls and 43,552 triangles. These are short local samples, not sustained-load measurements or physical-phone performance.

The mathematics remain a planar 2R/3R model with rectangular obstacles and finite collision samples. Joint heights are visual offsets; no vertical IK, self-collision, rigid-body dynamics or real robot control was added. The existing elbow preference can fall back to the other valid branch. Mobile evidence is browser emulation; Safari, Firefox and real touch devices were not tested.

## Review artifacts

Open `axis/index.html` through the local server for the before/after images, mobile comparison and video. `axis/motion.mp4` captures the real WebGL application at 24 fps, including pick/place, camera presets and a taught sequence. It is a screen recording, not a Blender render.

The preview remains local for user evaluation. A draft PR preserves the implementation without merging or changing GitHub Pages.
