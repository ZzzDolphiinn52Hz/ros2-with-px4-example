# Implementation Plan: Gazebo ArUco Landing End-to-End

1. Add a free-flight Gazebo world with a visible takeoff zone, an 80 mm marker
   target and the camera-equipped X500 starting away from the target.
2. Add a simulation landing launch that reuses component-owned ArUco, landing
   and bridge configuration while overriding only the simulated camera input.
3. Separate controller enable from `start_landing` so the operator has two
   explicit gates and enabling alone cannot create motion commands.
4. Add root commands for the free-flight world, QGroundControl, landing stack,
   read-only readiness/status checks, one-action service gates and rosbag.
5. Document the exact multi-terminal workflow, go/no-go checks, abort sequence
   and expected QGC/PX4/ROS state transitions.
6. Add regression tests, build the workspace, run the static safety checks and
   execute a controlled SITL landing when the environment permits.
7. Measure simulation capture stamps at Gazebo, the bridge callback, and ROS.
   Keep one pending frame and add Gazebo-clock age checks to readiness/PID.
