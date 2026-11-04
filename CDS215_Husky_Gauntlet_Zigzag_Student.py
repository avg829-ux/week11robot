# CDS215 — SENSOR ENGINEERING CHALLENGE (STUDENT STARTER)
# Built from the working Stage 2 faster Husky simulation.
# Two built-in courses: narrow gap and a more demanding Sensor Gauntlet.
# All sensor positioning and the timed-forward safety check are PROVIDED.
# Student coding task: change ONE elif condition in Section 6.
#
STAGE = 2
STAGE_DESCRIPTION = "Two-course front-corner sensor challenge — zigzag gauntlet"

'''CDS215 Front-Corner Sensor Engineering Challenge — student starter.
Requires: pip install pybullet. Run locally with a desktop GUI.
The obstacle-avoidance controller is ACTIVE. Change ONLY the marked elif.
'''
import math
import time
import pybullet as p
import pybullet_data

# ============================================================
# PART 1 — SIMULATION SPEED EXPERIMENT
# The fast Stage 2 sensor graphics optimization is retained:
# Ray detection runs at EVERY physics step, but debug lines are
# updated less often and their three IDs are reused.
# Try RAY_DRAW_HZ = 90 then 10 to see how speed changes.
# MAX_PLAYBACK_SPEED = 0 allows unlimited speed; 1.0 = real-time cap.
# TEST_REAL_SECONDS is clock time; TEST_SIM_SECONDS is robot time.
# A test ends when it reaches EITHER positive limit.
# Results are appended to Husky_Two_Course_Results.csv.
# ============================================================
import csv
from pathlib import Path

TEST_REAL_SECONDS = 180.0    # Real seconds; increase to 90 for longer Course 2
TEST_SIM_SECONDS = 0.0     # Optional simulated-time limit; 0 = no limit
MAX_PLAYBACK_SPEED = 3  # 0 = unlimited; 1 = real-time; 3 = up to 3x
# For fair performance comparisons, set MAX_PLAYBACK_SPEED = 0.
# Keep at least one of the two time limits above positive.
if TEST_REAL_SECONDS <= 0 and TEST_SIM_SECONDS <= 0:
    raise ValueError("Set TEST_REAL_SECONDS or TEST_SIM_SECONDS > 0")


# ============================================================
# TEACHING SECTION 1 — SETTINGS
# ============================================================
PHYSICS_HZ = 240       # Physics updates per simulated second. Keep 240 initially.
RAY_DRAW_HZ = 10       # Visual sensor updates per simulated second; try 90.
# RAY_DRAW_HZ = 0 turns off ray visualization (sensing still operates).
# Changing PHYSICS_HZ changes physics behavior and requires retesting.
SHOW_SENSOR_RAYS = True
_sensor_ray_ids = {"center": -1, "left": -1, "right": -1}
GOAL_RADIUS = 0.65
SPEED = 7.0
TURN_SPEED = 5.0
HEADING_TOLERANCE = 0.14
SENSOR_RANGE = 2.0
SENSOR_HEIGHT = 0.35
SENSOR_OFFSET = 0.65         # Front-to-back location of all sensors, meters
CORNER_OFFSET = 0.35         # Left/right placement of corner sensors, meters
STOP_DISTANCE = 1.0          # Same trigger distance for ALL three sensors
# ============================================================
# PART 4 — COURSE SELECTION: CHANGE JUST THIS NUMBER!
# 1 = Narrow Gap: investigate why the center ray misses an opening.
# 2 = Sensor Gauntlet: 20 obstacles scattered in a wide zigzag course.
# The same sensor settings apply to BOTH courses.
# You are NOT required to edit the course-building code below.
# ============================================================
COURSE = 2
if COURSE not in (1, 2):
    raise ValueError("COURSE must be 1 (Narrow Gap) or 2 (Sensor Gauntlet)")

# PART 3 — EXPERIMENT WITH SENSOR DIRECTION AND PLACEMENT
# Try CORNER_OFFSET values 0.20, 0.35, 0.45 (meters).
# Try SENSOR_ANGLE values 0, 15, 25 (degrees, converted to radians).
# The CENTER sensor always looks straight forward.
TURN_STEPS = 200         # Equivalent to 1.5 simulated seconds at 240 Hz
FORWARD_STEPS = 480      # Equivalent to 2.0 simulated seconds at 240 Hz
SENSOR_ANGLE = math.radians(15)
# Preserve the same maneuver durations if PHYSICS_HZ changes.
EFFECTIVE_TURN_STEPS = round(TURN_STEPS * PHYSICS_HZ / 240)
EFFECTIVE_FORWARD_STEPS = round(FORWARD_STEPS * PHYSICS_HZ / 240)

# Choose the destinations BEFORE creating the gold target markers.
# Course 1 preserves the original three destinations.
# Course 2 uses four destinations in a larger arena with changing turns.
if COURSE == 1:
    COURSE_NAME = "Narrow Gap"
    waypoints = [(4, 2), (6, 7), (2, 8)]
else:
    COURSE_NAME = "Sensor Gauntlet"
    waypoints = [(5, 9), (13, -7), (22, 10), (31, -7)]
LEFT_WHEELS = [2, 4]
RIGHT_WHEELS = [3, 5]

# ============================================================
# EXISTING WHEEL COMMANDS — DO NOT EDIT
# ============================================================
def set_wheel_speeds(robot_id, left, right):
    for joint in LEFT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=left, force=100)
    for joint in RIGHT_WHEELS:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL,
                                targetVelocity=right, force=100)

def stop_robot(robot_id):
    set_wheel_speeds(robot_id, 0, 0)

def move_forward(robot_id):
    set_wheel_speeds(robot_id, SPEED, SPEED)

def turn_left(robot_id):
    set_wheel_speeds(robot_id, -TURN_SPEED, TURN_SPEED)

def turn_right(robot_id):
    set_wheel_speeds(robot_id, TURN_SPEED, -TURN_SPEED)

def angle_difference(target_angle, current_angle):
    return math.atan2(math.sin(target_angle - current_angle),
                      math.cos(target_angle - current_angle))

# ============================================================
# TEACHING SECTION 2 — THREE FRONT-MOUNTED SENSORS (PROVIDED)
# ============================================================
# side = 0: center of front bumper; +1: left corner; -1: right.
# The sensor POSITION is offset from the robot's actual heading.
# The sensor DIRECTION may point outward by SENSOR_ANGLE.
# These are separate quantities: CORNER_OFFSET is in METERS,
# while SENSOR_ANGLE is in RADIANS (converted from degrees above).
# Students do NOT have to write or reindent these calculations.
def detect_obstacle(robot_id, x, y, heading, sensor_angle=0, side=0):
    look_heading = heading + sensor_angle
    front_x = x + SENSOR_OFFSET * math.cos(heading)
    front_y = y + SENSOR_OFFSET * math.sin(heading)
    # Shift left or right PERPENDICULAR to the heading.
    ray_start = [front_x - side * CORNER_OFFSET * math.sin(heading),
                 front_y + side * CORNER_OFFSET * math.cos(heading),
                 SENSOR_HEIGHT]
    ray_end = [ray_start[0] + SENSOR_RANGE * math.cos(look_heading),
               ray_start[1] + SENSOR_RANGE * math.sin(look_heading),
               SENSOR_HEIGHT]
    hit = p.rayTest(ray_start, ray_end)[0]
    obstacle_found = (hit[0] != -1 and hit[0] != robot_id)
    # KEEP the Stage 2 improvement: check the sensor every physics step,
    # but update its debug line only RAY_DRAW_HZ times per simulated second.
    # Use side instead of angle to identify the rays: all three can be
    # parallel when SENSOR_ANGLE is 0.
    if (SHOW_SENSOR_RAYS and RAY_DRAW_HZ > 0 and
        (_experiment_step_count == 0 or
         (_experiment_step_count * RAY_DRAW_HZ) // PHYSICS_HZ !=
         ((_experiment_step_count - 1) * RAY_DRAW_HZ) // PHYSICS_HZ)):
        sensor_key = {0: "center", 1: "left", -1: "right"}[side]
        line_color = [1, 0, 0] if obstacle_found else [0, 1, 0]
        _sensor_ray_ids[sensor_key] = p.addUserDebugLine(
            ray_start, ray_end, line_color, 3,
            replaceItemUniqueId=_sensor_ray_ids[sensor_key])
    if obstacle_found:
        return hit[2] * SENSOR_RANGE
    return None

# ============================================================
# TEACHING SECTION 3 — WHICH WAY IS CLEARER? (PROVIDED)
# ============================================================
# No hit is None, which counts as maximum clearance. Ties go left.
def choose_direction(left,right):
    if left is None:
        left = SENSOR_RANGE
    if right is None:
        right = SENSOR_RANGE
    if left >= right:
        return "left"
    else:
        return "right"

# ============================================================
# EXISTING SCENE SETUP — DO NOT EDIT
# ============================================================
def add_target_markers():
    for i, (x, y) in enumerate(waypoints):
        marker = p.createVisualShape(p.GEOM_CYLINDER, radius=GOAL_RADIUS,
                                     length=0.02, rgbaColor=[1, 0.75, 0, 1])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=marker,
                          basePosition=[x, y, 0.015])
        p.addUserDebugText(f"TARGET {i + 1}", [x, y, 0.45],
                           textColorRGB=[0.5, 0.3, 0], textSize=1.5)

def add_obstacle(x, y, color=(1, 0, 0, 1), half_extents=(0.6, 0.6, 0.5)):
    size = list(half_extents)
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size,
                                 rgbaColor=color)
    return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=[x, y, 0.5])

# ============================================================
# COURSE SETUP (PROVIDED — DO NOT EDIT FOR THIS LAB)
# ============================================================
# Course 1: TWO boxes that create a gap too narrow for the Husky.
# Course 2: TWENTY colorful obstacles scattered across a wide, zigzagging
# route (up/down/up/down) instead of a single straight line.
# Some boxes block the direct paths; others create misleading side gaps.
# An alternate physical path exists; the timed controller may still struggle.
# This is a CHALLENGE, not a verified autonomous completion run.
# Actual PyBullet gameplay must be tested by the instructor.
# Note: the target markers are only visual, never physical barriers.

GAUNTLET_OBSTACLES = [
    # 20 obstacles arranged ACROSS a wide 2-D zigzag, not in one line.
    # A few block the direct line; others flank the route or create false gaps.
    # Clear paths exist around the obstacles, but the timed controller is not a path planner.
    # LEG 1: from (-3, -1.5) to (5, 9) — scattered across both sides of the route.
    ( -0.90,   1.40, (0.94, 0.21, 0.19, 1), (0.68, 0.65, 0.50), 'Red'),
    ( -0.80,   4.65, (0.17, 0.42, 0.96, 1), (0.64, 0.64, 0.50), 'Blue'),
    (  1.70,   4.80, (1.0, 0.47, 0.36, 1), (0.72, 0.68, 0.50), 'Coral'),
    (  3.40,   3.00, (0.13, 0.18, 0.55, 1), (0.66, 0.66, 0.50), 'Navy'),
    (  3.20,   7.00, (0.99, 0.72, 0.16, 1), (0.70, 0.70, 0.50), 'Gold'),
    # LEG 2: from (5, 9) to (13, -7) — scattered across both sides of the route.
    (  6.40,   6.40, (0.19, 0.75, 0.33, 1), (0.68, 0.67, 0.50), 'Green'),
    (  8.80,   0.30, (0.65, 0.28, 0.86, 1), (0.66, 0.62, 0.50), 'Purple'),
    (  8.80,   2.12, (0.99, 0.49, 0.14, 1), (0.66, 0.62, 0.50), 'Orange'),
    ( 10.20,   4.80, (0.7, 0.6, 0.98, 1), (0.70, 0.67, 0.50), 'Lavender'),
    ( 11.20,  -3.40, (0.1, 0.76, 0.89, 1), (0.68, 0.70, 0.50), 'Cyan'),
    # LEG 3: from (13, -7) to (22, 10) — scattered across both sides of the route.
    ( 14.60,  -3.95, (0.05, 0.56, 0.48, 1), (0.68, 0.69, 0.50), 'Teal'),
    ( 16.40,  -5.40, (0.64, 0.9, 0.21, 1), (0.70, 0.70, 0.50), 'Lime'),
    ( 17.90,   2.20, (0.92, 0.28, 0.63, 1), (0.70, 0.70, 0.50), 'Pink'),
    ( 18.40,   8.40, (0.51, 0.51, 0.56, 1), (0.68, 0.70, 0.50), 'Gray'),
    ( 20.30,   6.80, (0.56, 0.34, 0.18, 1), (0.68, 0.70, 0.50), 'Brown'),
    # LEG 4: from (22, 10) to (31, -7) — scattered across both sides of the route.
    ( 23.50,   7.20, (0.08, 0.83, 0.73, 1), (0.72, 0.66, 0.50), 'Turquoise'),
    ( 25.60,   5.00, (0.96, 0.95, 0.12, 1), (0.70, 0.68, 0.50), 'Yellow'),
    ( 26.50,   1.40, (0.91, 0.13, 0.8, 1), (0.70, 0.70, 0.50), 'Magenta'),
    ( 27.70,  -6.20, (0.53, 0.61, 0.15, 1), (0.68, 0.70, 0.50), 'Olive'),
    ( 29.40,  -3.95, (0.64, 0.14, 0.13, 1), (0.68, 0.69, 0.50), 'Dark Red'),
]

client = p.connect(p.GUI)
if client < 0:
    raise RuntimeError("Cannot open PyBullet GUI")
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / PHYSICS_HZ)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("husky/husky.urdf", [-3, -1.5, 0.2])
add_target_markers()
if COURSE == 1:
    # Narrow gap: 0.50 m empty space across y, centered on TARGET 1's path.
    narrow_box = (0.30, 0.60, 0.50)
    add_obstacle(0.9, -0.40, [0.30, 0.50, 1.0, 1], narrow_box)  # Blue
    add_obstacle(0.9,  1.30, [0.80, 0.30, 0.90, 1], narrow_box)  # Purple
    add_obstacle(5, 5)  # Existing obstacles from Week 10
    add_obstacle(4, 7)
    camera_distance = 14
    camera_target = [3, 4, 0]
else:
    for ox, oy, color, box_size, nickname in GAUNTLET_OBSTACLES:
        add_obstacle(ox, oy, color, box_size)
    camera_distance = 47
    camera_target = [14, 1.5, 0]
# Overhead view makes the zigzag and lateral obstacle spacing easier to see.
p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                             cameraYaw=20 if COURSE == 2 else 45,
                             cameraPitch=-78 if COURSE == 2 else -65,
                             cameraTargetPosition=camera_target)

current_target = 0
TARGET_X, TARGET_Y = waypoints[current_target]
mission_complete = False
last_report = time.monotonic()  # Console reporting only; NOT avoidance timing.

# Stage 2 performance measurement: excludes scene-loading time.
_experiment_start = time.monotonic()
_experiment_step_count = 0
_experiment_last_report = _experiment_start
_experiment_last_steps = 0
print(f"\n=== STAGE {STAGE}: {STAGE_DESCRIPTION} ===")
print(f"COURSE {COURSE}: {COURSE_NAME} | {len(waypoints)} waypoints")
print(f"Physics {PHYSICS_HZ} Hz | sensor display {RAY_DRAW_HZ} Hz | "
      f"playback cap {MAX_PLAYBACK_SPEED or 'unlimited'}x")
print(f"Max time: {TEST_REAL_SECONDS or 'off'} real seconds; "
      f"{TEST_SIM_SECONDS or 'off'} simulated seconds")
print("More simulated seconds per real second means faster playback.")


# ============================================================
# TEACHING SECTION 4 — AVOIDANCE MEMORY (PROVIDED)
# ============================================================
avoiding_obstacle = False
avoid_direction = None
avoid_stage = None
avoid_counter = 0

# ============================================================
# MAIN SIMULATION LOOP — ONLY EDIT MARKED SECTIONS
# ============================================================
print("Husky Sensor Engineering Challenge: press Q in PyBullet to quit.")
while p.isConnected():
    keys = p.getKeyboardEvents()
    if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
        break
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    x, y = position[0], position[1]
    heading = p.getEulerFromQuaternion(orientation)[2]
    distance = math.hypot(TARGET_X - x, TARGET_Y - y)
    target_angle = math.atan2(TARGET_Y - y, TARGET_X - x)
    heading_error = angle_difference(target_angle, heading)

    # ========================================================
    # TEACHING SECTION 5 — READ THREE SENSORS
    # ========================================================
    center = detect_obstacle(robot_id, x, y, heading, 0, side=0)
    left = detect_obstacle(robot_id, x, y, heading, SENSOR_ANGLE, side=1)
    right = detect_obstacle(robot_id, x, y, heading, -SENSOR_ANGLE, side=-1)

    # ========================================================
    # TEACHING SECTION 6 — MOVEMENT DECISION
    # ========================================================
    # DO NOT change the waypoint-arrival section.
    if distance < GOAL_RADIUS and not mission_complete:
        stop_robot(robot_id)
        print(f"TARGET {current_target + 1} REACHED")
        current_target += 1
        if current_target < len(waypoints):
            TARGET_X, TARGET_Y = waypoints[current_target]
        else:
            mission_complete = True
            print("MISSION COMPLETE")
    elif not mission_complete:
        # This is the already-completed Week 10 obstacle avoidance controller.
        # The forward-stage safety check below is PROVIDED.
        # Only the clearly marked ONE elif condition is student-written.
        
        if avoiding_obstacle:
            avoid_counter += 1
            if avoid_stage == "turning":
                if avoid_counter <= EFFECTIVE_TURN_STEPS:
                    if avoid_direction == "left":
                        turn_left(robot_id)
                    else:
                        turn_right(robot_id)
                else:
                    avoid_stage = "forward"
                    avoid_counter = 0
                    move_forward(robot_id)
            elif avoid_stage == "forward":
                # PROVIDED SAFETY CHECK: even during a timed forward move,
                # do not blindly drive into a newly detected obstacle.
                # This short prewritten any(...) expression checks all three.
                # Students need NOT modify or understand this helper in detail.
                if any(reading is not None and reading < STOP_DISTANCE
                       for reading in (center, left, right)):
                    stop_robot(robot_id)
                    avoid_direction = choose_direction(left, right)
                    avoid_stage = "turning"
                    avoid_counter = 0
                    print("Obstacle during forward stage: turn again")
                elif avoid_counter <= EFFECTIVE_FORWARD_STEPS:
                    move_forward(robot_id)
                else:
                    stop_robot(robot_id)
                    avoiding_obstacle = False
                    avoid_direction = None
                    avoid_stage = None
                    avoid_counter = 0
                    print("Avoidance complete!")

        # ====================================================
        # STUDENT CODING CHALLENGE — MODIFY JUST THE elif BELOW
        # ====================================================
        # Currently only the center sensor can START obstacle avoidance.
        # Change it so CENTER OR LEFT OR RIGHT can trigger avoidance
        # when its reading is not None and is less than STOP_DISTANCE.
        # Do not change the code beneath the elif.
        elif (center is not None and center < STOP_DISTANCE)or (left is not None and left < STOP_DISTANCE) or (right is not None and right < STOP_DISTANCE):
            stop_robot(robot_id)
            avoiding_obstacle = True
            avoid_direction = choose_direction(left, right)
            avoid_stage = "turning"
            avoid_counter = 0
            print(f"Obstacle detected! Turning {avoid_direction}")
        elif heading_error > HEADING_TOLERANCE:
            turn_left(robot_id)
        elif heading_error < -HEADING_TOLERANCE:
            turn_right(robot_id)
        else:
            move_forward(robot_id)
        

    if not mission_complete and time.monotonic() - last_report >= 2:
        def reading(value):
            return "CLEAR" if value is None else f"{value:.2f} m"
        print(f"Target distance={distance:.2f} m | "
              f"L={reading(left)} C={reading(center)} R={reading(right)}")
        last_report = time.monotonic()

    # Exactly one physics step each loop; time step = 1 / PHYSICS_HZ.
    p.stepSimulation()

    # PERFORMANCE MEASUREMENT (same in every stage).
    _experiment_step_count += 1
    # No tiny sleep after each step; cap once per block if requested.
    if MAX_PLAYBACK_SPEED > 0 and _experiment_step_count % max(1, PHYSICS_HZ // 10) == 0:
        _desired_wall = _experiment_step_count / (PHYSICS_HZ * MAX_PLAYBACK_SPEED)
        _remaining = _desired_wall - (time.monotonic() - _experiment_start)
        if _remaining > 0:
            time.sleep(_remaining)

    _now = time.monotonic()
    if _now - _experiment_last_report >= 4.0:
        _interval_wall = _now - _experiment_last_report
        _interval_sim = (_experiment_step_count - _experiment_last_steps) / PHYSICS_HZ
        print(f"Stage {STAGE}: {_interval_sim / _interval_wall:.2f}x real time "
              f"| targets reached: {current_target}/{len(waypoints)}")
        _experiment_last_report = _now
        _experiment_last_steps = _experiment_step_count

    # Stop when either active time limit is reached.
    if TEST_REAL_SECONDS > 0 and _now - _experiment_start >= TEST_REAL_SECONDS:
        break
    if TEST_SIM_SECONDS > 0 and _experiment_step_count / PHYSICS_HZ >= TEST_SIM_SECONDS:
        break



# Save the result even if the robot has not reached its destination.
# If you want to run a fresh experiment, you can rename/delete the CSV.
_experiment_elapsed = time.monotonic() - _experiment_start
_experiment_simulated = _experiment_step_count / PHYSICS_HZ
_experiment_speed = _experiment_simulated / _experiment_elapsed
print(f"\nSTAGE {STAGE} RESULT: {_experiment_speed:.2f}x real time "
      f"| {_experiment_step_count} steps | "
      f"{current_target}/{len(waypoints)} targets reached")
_csv_file = Path(__file__).with_name("Husky_Two_Course_Results.csv")
_fields = ["stage", "change", "physics_hz", "ray_draw_hz", "playback_cap",
           "corner_offset", "sensor_angle_deg", "course", "course_name",
           "real_seconds", "simulated_seconds", "speed_x", "steps",
           "targets_reached", "total_targets", "mission_complete"]
_new = not _csv_file.exists()
with _csv_file.open("a", newline="", encoding="utf-8") as _csv_out:
    _writer = csv.DictWriter(_csv_out, fieldnames=_fields)
    if _new:
        _writer.writeheader()
    _writer.writerow({
        "stage": STAGE, "change": STAGE_DESCRIPTION, "physics_hz": PHYSICS_HZ,
        "ray_draw_hz": RAY_DRAW_HZ, "playback_cap": MAX_PLAYBACK_SPEED,
        "corner_offset": CORNER_OFFSET,
        "sensor_angle_deg": round(math.degrees(SENSOR_ANGLE), 1),
        "course": COURSE, "course_name": COURSE_NAME,
        "real_seconds": round(_experiment_elapsed, 3),
        "simulated_seconds": round(_experiment_simulated, 3),
        "speed_x": round(_experiment_speed, 3),
        "steps": _experiment_step_count, "targets_reached": current_target,
        "total_targets": len(waypoints), "mission_complete": mission_complete
    })
print(f"Saved result to {_csv_file.name}")

if p.isConnected():
    stop_robot(robot_id)
    p.disconnect()
