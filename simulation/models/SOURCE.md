# UR5e model provenance

`ur5e/` is copied without changes from Google DeepMind's MuJoCo Menagerie:
https://github.com/google-deepmind/mujoco_menagerie/tree/8161bba264d7fa7c99ca301e91e7fb44737676ad/universal_robots_ur5e

Its BSD-3-Clause license and original README are included in that directory.
The model and meshes total about 31 MB, so setup needs no additional model download.

Our `simulation/model.py` applies the following changes in memory:

- Align world coordinates with ROS `base_link` by removing the base's Z rotation.
- Match the ROS driver's nominal UR5e dimensions: shoulder height 0.1625 m,
  forearm length 0.3922 m, wrist offsets 0.1333/0.0997/0.0996 m. Menagerie spreads
  the first wrist offset over three bodies: 0.138 - 0.131 + 0.1263 = 0.1333 m.
- Name the existing attachment site `tool0`; its orientation already matches
  ROS after the base correction.

These are nominal dimensions, not physical robot calibration. Every exported
plan includes MoveIt FK reference poses, which must match the adapted model
before simulation starts. Collision shapes and inertias remain approximations.
