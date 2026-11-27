# Implementation Plan: Gazebo ArUco PX4 SITL

1. Add a PC-only simulation package with workspace-owned Gazebo models,
   textures, world, ROS bridge configuration and launch composition.
2. Include the custom X500 in the world and attach PX4 SITL to the existing
   Gazebo entity using `PX4_GZ_MODEL_NAME`, avoiding edits to PX4 sources.
3. Bridge the camera image and CameraInfo into the existing ROS contract.
4. Launch the current detector in `ros_topic` mode while reusing the owning
   ArUco YAML; do not launch PID or PX4 bridge in the phase-2 detector test.
5. Add root operator commands and documentation for repeatable terminals.
6. Verify SDF/assets statically, build/tests locally, then run Gazebo, PX4,
   XRCE and detector smoke tests when the host supports them.
