# Passive finite connectors (local review)

Optional `scenario.scene.compliant_connections` declares an intrinsic connector,
independent of Mission or the model controller. No force acts before the measured
child-in-parent pose meets both admission thresholds. It then enables a finite D6
spring; crossing the break envelope removes it for the rest of the episode.
World reset re-arms it. Neither operand can be a robot/hand articulation.

```yaml
scenario:
  scene:
    compliant_connections:
      phone_usb:
        parent: objects.phone
        child: objects.plug
        rest_pose:
          xyz_m: [0, -0.082, 0]
          quat_xyzw: [-0.5, -0.5, 0.5, 0.5]
        properties:
          stiffness_n_m: 300
          damping_ns_m: 1
          angular_stiffness_nm_rad: 1
          angular_damping_nms_rad: 0.002
          max_force_n: 3
          max_torque_nm: 0.010
          break_distance_m: 0.008
          break_angle_rad: 0.35
          admission_position_m: 0.0015
          admission_angle_rad: 0.020
```

Coordinates are child asset-root in parent asset-root coordinates, SI metres and
XYZW quaternions. Both objects must exist as rigid bodies. All ten parameters are
required, positive and finite; admission must be inside the break envelope.
Unknown fields, duplicate child ownership, absent or articulation endpoints fail.
The optional declarations are part of the immutable Plan/WorldSpec digest.
Without declarations, canonical WorldSpec output and capability requirements are
unchanged. Required `physics.compliant_connections@1` prevents unsupported
providers from silently ignoring the mechanism. There is no Mission-only action,
implicit hand attach, pose overwrite, collision filter or fixed infinite joint.

The Isaac provider checks admission/break at each native physics substep. All six
D6 degrees of freedom remain compliant. Per-axis force/torque caps are divided by
sqrt(3), bounding the resultant vector. Angular SI stiffness/damping are converted
to USD degree units. Body-body collision remains enabled. Prescribed kinematic
Replay endpoints receive no connection forces. Checkpoint capture explicitly
rejects worlds with connector state until that state has a portable checkpoint
representation; ordinary worlds retain existing behavior.

This is a local, uncommitted review feature. Unit tests establish contracts and
joint attributes, not native force behavior or phone task success. Full physical
seating, release, loaded retention, table return and camera review are required.
