# Approach Document — TidyBot Home-Tidying Robot

## Robot Model Design

### Base & Mobility
The robot uses a **4-wheeled skid-steer base** (0.50m x 0.40m x 0.12m, 8kg). All four wheels are actuated via Gazebo's `DiffDrive` plugin — both left wheels receive the same velocity command, both right wheels receive the same command. The control model is differential drive; the physical behavior is skid-steer (wheels scrub laterally during turns). The 0.40m width keeps the robot narrow enough for the 1.5m doorway while providing stable ground contact. Wheels are 0.075m radius with realistic rubber friction (mu=1.0/0.8) and proper cylindrical inertia tensors computed via xacro macros.

### Torso & Arms
A vertical torso (0.20m x 0.15m x 0.40m) is fixed-mounted on the base center. The **right arm** has 3 revolute joints (shoulder, elbow, wrist) on the Y-axis, with PID-controlled position controllers in Gazebo. The shoulder joint origin is rotated -1.0 rad so the arm rests pointing forward-down instead of hanging straight through the base. The arm can reach the ground plane when the shoulder rotates back toward vertical. A flat-plate end-effector (0.08m x 0.06m box) is fixed at the wrist.

The **left arm** is cosmetic (fixed joints) posed in a natural resting position with a slight forward lean.

### Head / Face
A flat cyan panel (0.18m x 0.10m) mounted atop the torso represents an LED display face. Visual-only element.

### Sensors
- **Camera**: 640x480 RGB at 15fps, mounted on the torso front, facing forward. Includes an optical frame with the standard ROS rotation.
- **LiDAR**: 360-sample GPU lidar, 0.12-8.0m range at 10Hz, mounted on the base top. Primary sensor for navigation and costmap obstacle detection.
- **IMU**: 50Hz inertial measurement unit on the base. Provides orientation and angular velocity data.

### Physical Properties
Every link has computed mass and inertia values using box/cylinder inertia formulas in xacro macros; for example, the base uses `I_xx = m(h²+d²)/12` with actual dimensions and an 8kg mass. Joint damping ranges from 0.1 (wheels) to 1.0 (shoulder), with friction values preventing drift at rest.

## Home World Design

### Layout
Two rooms connected by a 1.5m-wide doorway:
- **Room 1**: 5m x 5m (x: -4.5 to 0.5, y: -2.5 to 2.5)
- **Room 2**: 6m x 5m (x: 0.5 to 6.5, y: -2.5 to 2.5)

The robot spawns at (-2.0, 0.0) in Room 1, facing the doorway. The collection box sits at (-3.5, -1.8) in the southwest corner of Room 1.

### Furniture
All furniture is grounded (solid cuboids extending to the floor) so the LiDAR at 0.215m height can detect it:

**Room 1**: couch with base and back against the west wall, solid coffee table near the north wall, side table along the south wall, cylindrical plant pot near the doorway.

**Room 2**: shelf against the north wall, solid desk in the northeast, armchair in the southwest, cabinet along the east wall, cylindrical lamp stand in the center-north.

**Doorway obstacle**: I deliberately placed a shoe rack just past the doorway (x=1.2) to test the robot's ability to navigate around unexpected obstacles in tight spaces. This forces Nav2 to plan a curved path through the doorway instead of a straight line, which is a more realistic test of the navigation stack.

Each room has a mix of rectangular and cylindrical furniture to create varied laser scan profiles. This helps verify that the costmap correctly represents the environment and that the planner can find paths between different obstacle shapes.

### Pickup Objects
5 small objects are scattered strategically across both rooms so that collecting them requires the robot to traverse the entire home:

1. Red block (-1.5, -0.3) — Room 1, ahead of spawn
2. Green can (-1.0, 0.5) — Room 1, near the doorway side
3. Blue toy (2.0, 0.0) — Room 2, just past the doorway
4. Yellow ball (3.5, -0.8) — Room 2, center-south
5. Purple cylinder (5.0, 0.5) — Room 2, far east corner

The placement ensures the robot must navigate through the doorway, maneuver around the shoe rack obstacle, and reach all corners of Room 2. Each object has realistic mass (40-80g), proper inertia tensors, and surface friction.

### Physics
ODE physics engine with 1ms step size, ground friction 0.8/0.6. The `ignition-gazebo-imu-system` plugin is explicitly loaded in the world SDF — a Gazebo Fortress requirement since the general `Sensors` system only handles rendering-based sensors (camera, LiDAR), not the IMU.

### Pre-built Map
The occupancy grid (`home_map.pgm`, 260x140 pixels at 0.05m/cell) is generated programmatically from the exact SDF wall and furniture coordinates using `generate_map.py`. This guarantees alignment between the map and the simulated world. All furniture visible at LiDAR height is included as occupied cells.

## Navigation Strategy

### From Waypoints to Nav2
My initial approach was hardcoded waypoint navigation with proportional control — turn toward the next waypoint, drive forward, avoid obstacles using LiDAR sectors. This got the robot moving quickly but had significant problems: the robot moved in a jerky manner, oscillated near waypoints, and collided with furniture during turns. Tuning the proportional gains and obstacle avoidance thresholds was becoming a time sink with diminishing returns.

Rather than spending more time on PID tuning and path smoothing for a custom waypoint follower, I switched to **Nav2** — the standard ROS 2 navigation stack. This gave me a properly tested global planner (A*), local controller (DWB), recovery behaviors (spin, backup, wait), and costmap management out of the box. The tradeoff is more setup complexity (lifecycle management, parameter tuning, TF chain requirements) but much more robust navigation.

### Nav2 Configuration
- **NavfnPlanner** (A* global planner) with 0.75m tolerance — plans paths on the static map
- **DWB local controller** with tuned velocity limits (0.8 m/s linear, 1.0 rad/s angular) — follows the global path while avoiding local obstacles
- **Behavior server** for spin/backup/wait recovery actions — handles situations where the robot gets stuck
- **Global costmap** from the static map with inflation layer — used for path planning
- **Local costmap** with live LiDAR obstacle layer — used for real-time collision avoidance

### Costmap Tuning
The inflation radius (0.30m) and robot radius (0.22m) were tuned specifically for the doorway passage. The initial inflation of 0.45m left only 0.60m of effective doorway width — the robot frequently got stuck. The final values give 0.46m clearance on each side, enough for smooth passage while still keeping the robot away from walls.

### Localization

Getting reliable localization for a skid-steer robot was the biggest challenge in this project. The Gazebo `DiffDrive` plugin computes odometry assuming ideal differential drive kinematics, but the 4-wheel skid-steer causes significant wheel slip during turns — the computed rotation can be off by 30-50% during a 90-degree turn.

**Approaches I tried (in order):**

1. **Static map→odom transform**: Works perfectly at spawn but drifts as soon as the robot turns. Nav2 drove into walls within seconds.
2. **AMCL (laser + map matching)**: Couldn't converge because the odom rotation was too wrong for the particle filter to match the scan to the map. The robot's believed orientation diverged from reality faster than AMCL could correct.
3. **EKF with IMU fusion** (`robot_localization`): Fused wheel odom (translation) + IMU gyro (rotation). Fixed the rotation problem — IMU is perfect in simulation — but translation still drifted because the wheel velocity includes lateral slip during turns.
4. **EKF with laser scan matching** (`rf2o_laser_odometry`): Added scan-to-scan matching as a third source. Best results in debug mode, but rf2o had initialization issues and Eigensolver failures when the robot was stationary near walls.
5. **Simulation-native pose** (final): Leveraged Gazebo's SceneBroadcaster to get the robot's world pose directly from the physics engine. Zero drift, zero tuning.

**Final TF chain:**
```
map ──[identity]──> odom ──[world pose]──> base_footprint ──[URDF]──> lidar_link
```

## Pick-and-Place

### Approach
For each object:
1. Nav2 drives toward the object's known position
2. The task node monitors the robot's distance to the target via TF lookup
3. When within 0.5m, the Nav2 goal is canceled — this avoids the oscillation that occurs when DWB tries to reach an exact pose
4. The arm plays a reach-grab animation (pre_reach → reach → grab)
5. The object is teleported into the collection box via `ign service set_pose`
6. The arm returns to tucked position

After all 5 objects are collected, the robot navigates back to the collection box area for a release animation.



## Tradeoffs

1. **Nav2 over custom waypoint follower**: More setup complexity but dramatically better path planning, obstacle avoidance, and recovery behaviors. The assignment says simple logic is fine — but Nav2 is actually simpler to get working correctly than a well-tuned custom controller.

2. **Proximity cancel over precise goal reaching**: Canceling navigation at 0.5m from the target and using the arm's reach eliminates the goal-oscillation problem without requiring controller parameter tuning.


3. **Pre-built map over SLAM**: Generating the map from the SDF geometry guarantees alignment. SLAM would be more realistic but adds a mapping phase and can fail to close loops in rooms with similar geometry.

4. **Grounded furniture over legged furniture**: Solid cuboids from floor to table height are visible to the LiDAR and appear correctly in the costmap. Floating tabletops or thin table legs would be invisible at the LiDAR scan height, creating phantom free space in the costmap.

## What I Would Improve With More Time

### Navigation
- **Custom behavior trees**: the default NavigateToPose BT works but doesn't handle multi-room exploration well. A custom BT could add room-clearing patterns, doorway alignment behaviors, and smarter recovery sequences instead of the generic spin-backup-wait cycle.
- **Costmap filters**: keepout zones around the collection box and preferred lanes through the doorway. Nav2 supports costmap filter layers that mark regions as preferred or prohibited.

### Manipulation
- **Actuated gripper** using Gazebo's detachable joint plugin. The robot would make contact, create a temporary fixed joint, carry the object physically, then release. This removes the teleportation and makes pick-and-place physically realistic.
- **Joint trajectory control** via `ros2_control` with `gz_ros2_control`. Currently each arm joint has an independent PID controller. With ros2_control, I could use MoveIt2 for coordinated motion planning with self-collision checking.

### Perception
- **Camera-based object detection**: a color filter on the camera feed could detect the colored objects and compute their position using the LiDAR range at the same bearing. This would replace hardcoded positions and work with randomly placed objects.


### Simulation Quality
- **Custom mesh models**: replace primitive geometries with proper .dae or .obj meshes for realistic furniture and robot appearance. Rounded edges on the torso, proper joint housings on the arm, actual chair/table models.
- **Sensor noise models**: Gaussian noise on LiDAR range data, IMU bias drift, camera distortion. Perfect sensors don't test robustness — noise would force the navigation stack to handle real-world conditions.
- **Domain randomization**: vary lighting, furniture placement, object positions, and floor friction between runs. Standard practice for sim-to-real transfer and testing robustness.

### Infrastructure
- **Automated testing**: a CI pipeline that launches the sim headless, runs the task, and checks — did the robot visit both rooms? Collect all objects? Collide with anything? This catches regressions when tuning parameters.


## My Build vs. Drift's Build

My initial plan was to use Drift to generate the robot and world first, evaluate its output, and then build an improved version on top of it. I wanted to see where the tool excels and where external judgment is still needed.

### Setup Experience
I started without ROS or Gazebo installed and asked Drift to set up both in a conda environment. I was aware that Gazebo can be problematic in conda, but I hoped it would leverage the latest Robostack packages to handle this. After 20+ debugging steps, the tool was unable to resolve the conda-based installation and its responses became repetitive — restating the initial instructions rather than adapting to the specific errors encountered.

I moved to a bare metal ROS 2 + Gazebo setup. Drift installed ROS 2 Humble but paired it with Gazebo Harmonic (`gz sim 8`). This created a fundamental compatibility issue: `ros-humble-ros-gz-bridge` is compiled against Ignition Fortress (`ign-transport11`), while Gazebo Harmonic uses `gz-transport13`. The bridge couldn't forward any data between Gazebo and ROS — a version mismatch that should have been caught before installation since the ROS-Gazebo pairing is well-documented. After that I did ask drift to make the robot URDF but given the time I had already spent, I decided to proceed with the assignment on my own after that.

### Robot URDF Comparison
Drift did generate a robot URDF before I moved on. Comparing it with my final build:

**What Drift did well:**
- Computed inertia values using proper box/cylinder formulas (not placeholders)
- Set joint damping and friction on all actuated joints
- Included both a functional right arm (3-DOF with prismatic gripper fingers) and a cosmetic left arm
- Proper `base_footprint → base_link` hierarchy
- Camera and LiDAR sensors with correct `gz_frame_id` tags

**What was wrong:**

1. **Arm rest position**: Drift's URDF has no shoulder joint origin rotation — at joint position 0, the arm hangs straight down through the robot body and the gripper drags on the ground. This is a common URDF authoring mistake that only becomes visible when you spawn the robot. My URDF rotates the shoulder origin by -1.0 rad so the arm rests pointing forward-down with ground clearance. This required me to adjust all arm pose values and joint limits accordingly — the kind of iterative tuning that requires running the simulation.

2. **Gazebo version mismatch in bridge config**: Drift's launch file uses `gz.msgs.*` message types (Harmonic convention) in the `ros_gz_bridge` arguments. On a ROS 2 Humble system, the bridge expects `ignition.msgs.*` (Fortress convention). The bridge would create the topics but no data would flow — a silent failure that only manifests at runtime. My bridge uses the correct `ignition.msgs.*` types verified by checking actual data flow with `ros2 topic echo`.

3. **Sensor frame integration**: Drift's sensor configuration is syntactically correct, but there is no `ignition-gazebo-imu-system` plugin in the world SDF. In Gazebo Fortress, the IMU requires its own dedicated system plugin separate from the general `Sensors` system. Without it, the IMU sensor silently doesn't exist — no topic, no data, no error message. This is a Fortress-specific requirement that isn't obvious from documentation.


I want to be clear that these observations reflect my specific experience with the tool at a particular point in time and with my particular prompting approach. I'm sure with better prompts I could have finished the complete piepline using drift.

## External Resources & Tooling

To move faster on the repetitive parts, I used Ai to generate boilerplate URDF/xacro structure and SDF world scaffolding — the xacro property blocks, inertia macros, link/joint templates, wall definitions, etc. I then filled in the actual dimensions, tuned the physics parameters, fixed the sensor integration, and debugged everything by running the simulation and iterating. The code was cleaned up and commented properly using Ai as well. In addition to this I used:

- Gazebo Fortress plugin API documentation
- ROS 2 Humble + Gazebo Fortress compatibility matrix (ros_gz README)
- Nav2 documentation for DWB controller tuning and costmap configuration
- Standard box/cylinder inertia formulas from robotics textbooks

