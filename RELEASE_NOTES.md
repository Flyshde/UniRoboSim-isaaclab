# UniRoboSim Isaac Lab 0.10.30

Requires UniRoboSim >=0.10.13,<0.11. Adds seat-gated finite compliant connections
using force-limited PhysX D6 springs. Connections activate only inside the declared
position/angle admission envelope, break outside the distance/angle envelope, and
remain disconnected until episode reset. Connector checkpoints are explicitly
unsupported. Authored collision behavior is preserved; no hand-object fixed
attachment or global collision disable is introduced.

Preserves cable capsule planning geometry and supported exact sparse collision
filters. Corrects embedded rigid actor-origin/center-of-mass velocity conversion
and publishes paused Replay body transforms to Fabric. Native testing exposed a
partial-environment write defect: PhysX tensor setters require full view-sized
buffers even with selected indices. The common writer now scatters selected rows
into complete buffers, preserving untouched environments for Replay, reset and
scene pose updates. Released 0.10.29.post1 Replay visibility behavior is retained.

Validation: all 907 software tests passed in two processes (94 import-boundary
checks and 813 remaining checks). The original main branch reproduces the same
16 import-cleanliness failures when all tests share a process with native USD
imports; these failures and the partition rationale are retained in the review
record. No assertions were disabled.

Native PhysX minimal-fixture validation passed 14 connector checks and 9 embedded
rigid Replay checks. Measured saturated resultant spring force was 3.000001 N for
a 3 N cap; measured angular torque was 0.0057711 N m for a 0.01/sqrt(3) N m
per-axis cap. Checks cover seat gating, gravity retention, angular response,
break/no-reconnection, partial reset, checkpoint rejection, actor/COM twist,
paused Fabric publication, and partial-environment Replay/reset/pose updates.

Two expanded native launches stalled before Kit startup and were terminated;
the bounded diagnostic rerun passed all 9 embedded checks without an SDK or
source change. The startup stalls remain recorded and unexplained; a successful
rerun does not establish startup reliability. Full phone insertion-task physics,
complete cable/robot task assets, and production deployment remain unverified.
These minimal fixtures do not constitute complete phone-task acceptance.

For the current production scope, preserve camera `render_exclude_meshes` and run
services with `UNIROBOSIM_ISAACLAB_CAMERA_PRIMARY_RAY_EXCLUSIONS=true`; verify the
actual service/native process environment before admission. Older release notes
below describe historical settings and do not override this current requirement.
No simulator SDK, Mission, cuRobo, Genesis or vendor source asset was modified.

# UniRoboSim Isaac Lab 0.10.29.post1

Retains the cleared replay visibility layer through native renderer teardown. Reset restores source visibility and reuses the same world-owned layer; runtime keeps a strong reference until SimulationApp.close completes. Avoids releasing the replay USD layer while RTX may retain weak references. Source assets and replay physics remain unchanged. Installed-wheel focused checks: 124 passed. A real Kit two-generation RGB render/clear/reapply/close check exited zero. The minimal old-version comparison also exited zero; the original production cleanup abort was not reproduced in the tiny scene, so full production-scene cleanup acceptance remains pending.

# UniRoboSim Isaac Lab 0.10.29

Adds explicit replay visibility on a single static/composite environment, requiring UniRoboSim >=0.10.12,<0.11. All bounded requests are validated before a world-owned anonymous USD session layer is installed. Source/effective visibility, native readback and ordered receipts are checked; failed reads roll back, and reset/close remove the scope. One batch is permitted until reset. Source assets, geometry and physics are not edited. Existing camera partition exclusions are retained.

Focused CPU/backend/worker/package tests: 133 passed. Actual scene011/scene015 calibrated camera replay remains a required GPU canary before batch acceptance; this release note does not claim it passed. No simulator SDK, Mission or cuRobo changes.

## 0.10.27 production compatibility

Restores the previously production-validated static primary-ray camera partition and stationary kinematic render-state support from 0.10.25.post2, while retaining 0.10.26 soft-mount and effective-joint-limit behavior. No extra render pass or per-frame visibility switching is introduced. Production headless-physics keeps `UNIROBOSIM_ISAACLAB_CAMERA_PRIMARY_RAY_EXCLUSIONS=false`; enable it explicitly for the validated camera replay profile. Nonzero kinematic replay velocity remains rejected before any frame mutation.

# UniRoboSim Isaac Lab 0.10.26

Requires UniRoboSim >=0.10.11,<0.11. Adds capability-gated elastic materials,
permanent solver mounts, prescribed kinematic boundaries, and explicit deformable
collision offsets on the actual collision mesh. Preserves the previously unpublished
native effective joint-position limit fix and strengthens malformed DOF/numeric
admission. Existing SDK SI precision is retained in catalog and binding descriptors.

Two-mode, two-environment native create/step/partial-reset/close smokes pass with
rotated parents and nonzero free-node initial velocity. This is not complete G2
contact/load/reaction certification. See scripts/soft_mount_conformance.py.
No SDK, Mission/cuRobo, vendor asset, or global solver timestep was modified.

## Previous release

# UniRoboSim Isaac Lab 0.10.25

Adds immutable appearance capture/restore contracts for FSR materials, directional lighting, environment, stable visual bindings and portable opaque base-color PNG textures with explicit UV/topology validation. Includes prerequisite deformable render-state/topology and exact float32 per-particle RGBA contracts. Recorded render-state application does not advance physics.

Validation: a Genesis recording was restored into Isaac Lab for all 121 mixed rigid/articulated/deformable/fluid frames; positions/joints/colors matched exactly except root-position float error below 2.4e-8 m. An embedded four-color PNG restored after original texture files were isolated. A real FastSimApplication Replay completed all 76 frames, with 2 materials, 2 bindings and 1 light restored.

Known scope: native output transfer is renderer dependent (Genesis gamma 2.2 versus RTX sRGB), reported through conversion notes. Unsupported material graphs, non-opaque PNG textures, independent emissive materials and unsupported light types are rejected rather than silently approximated. Dynamic appearance mutation is not supported by this release's recording path. The extra FastSimApplication camera subscription currently reports a render-only tick-consistency error; normal Replay completed successfully and SDK RGB output was verified separately. Occasional native Kit startup stalls are bounded by worker startup deadlines.

No Mission, cuRobo, simulator engine or vendor source was changed.

Requires UniRoboSim >=0.10.10,<0.11. Release checks: 93 import-boundary/profile tests and 615 remaining CPU tests passed in separate processes (the native USD tests intentionally import pxr and therefore cannot share the import-cleanliness test process). Wheel and source distribution built successfully.
