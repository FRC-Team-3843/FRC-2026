# Working Notes - FRC-2026

================================================================================
FRC Team 3843 - 2026 Robot Setup Guide
================================================================================

Welcome! This guide will walk you through setting up the 2026 swerve robot from
hardware check to first autonomous run. Follow the sections in order for best
results. When in doubt, measure twice and deploy once.

Quick reference links:
- YAGSL Docs: https://broncbotz3481.github.io/YAGSL-Example/
- PhotonVision Docs: https://docs.photonvision.org/
- WPILib Docs: https://docs.wpilib.org/en/stable/

================================================================================
SECTION 1: PRE-FLIGHT CHECKLIST
================================================================================

Before you write a single line of code, verify the hardware is ready:

**Power & CAN Bus:**
1. All motor controllers have correct CAN IDs (see CLAUDE.md for assignments)
2. CANCoders are plugged into data, not just power (yes, this has happened)
3. NavX is connected via SPI port on RoboRIO
4. No loose connections - tug test every cable

**Electrical Safety:**
1. Main breaker off when making connections
2. Battery secured with velcro/straps
3. Emergency stop accessible

**Physical Inspection:**
1. All four swerve modules spin freely (no binding)
2. Wheels are the same model and firmware
3. Check bevel gear direction (we'll come back to this in Section 2)

**Software Tools Ready:**
- Latest WPILib installed (2026.1.1 or newer)
- Phoenix Tuner X for CTRE devices
- REV Hardware Client for SparkMax
- AdvantageScope or Glass for telemetry

================================================================================
SECTION 2: YAGSL CONFIGURATION
================================================================================

YAGSL (Yet Another Generic Swerve Library) handles the math. You handle the
measurements. Be precise here - "close enough" leads to drift and wonky auto.

### 2.1 Module Wheel Positions

Open each module JSON in `src/main/deploy/swerve/modules/`

The `location` field tells YAGSL where each wheel is relative to robot center
(the point on the floor equidistant from all four wheels). Units are INCHES.

- `front`: + is toward front, - is toward back
- `left`: + is toward left, - is toward right

**How to measure:**
1. Measure center-to-center between FL and FR wheels (call this W)
2. Measure center-to-center between FL and BL wheels (call this L)
3. FL module location: `"front": L/2, "left": W/2`
4. FR module location: `"front": L/2, "left": -W/2`
5. BL module location: `"front": -L/2, "left": W/2`
6. BR module location: `"front": -L/2, "left": -W/2`

**Common mistake:** Using frame size instead of wheel center distance. A 27"
frame with MK4i modules (2.625" inset on each side) gives 21.75" wheelbase, not 27".

Current config has `13.5` inches (27" wheelbase = ~32" frame). Verify this
matches YOUR robot or odometry will be off.

Reference: https://broncbotz3481.github.io/YAGSL-Example/configuring/swerve-modules.html

### 2.2 Module Drive Motor Direction (The Bevel Gear Check)

Here's a common gotcha: swerve wheels are symmetric, but the gearbox is not.

When you point a wheel "forward", the bevel gear (the angled metal gear visible
near the wheel) can be on the left OR right side. If your modules aren't
consistent, some will drive backward when commanded forward.

**Quick check:**
1. Point all four wheels straight forward (parallel to frame rails)
2. Look at each bevel gear - are they all on the same side?
   - All on LEFT: Set `"inverted": {"drive": false}` in all module JSONs
   - All on RIGHT: Set `"inverted": {"drive": true}` in all module JSONs
   - Mixed (some left, some right): Set each module individually

The current config assumes all bevels are on the left (`drive: false`). If your
modules are different, update the JSONs now before proceeding.

### 2.3 Module Zeroing (Absolute Encoder Offsets)

The CANCoders remember their position even when powered off, but they don't know
what "forward" means. You have to tell them.

**Procedure:**
1. Put robot on blocks so wheels can spin freely
2. Use a straightedge (ruler, level, piece of aluminum) along the frame rail
3. Rotate FL module until wheel is perfectly aligned with straightedge
   - Double-check the bevel gear is on the correct side (see 2.2 above)
4. Open Phoenix Tuner X and connect to the robot
5. Find the FL CANCoder (ID 9) and read its "Absolute Position"
6. Write this number in `frontleft.json` as `"absoluteEncoderOffset"`
7. Repeat for the other three modules (FR=10, BL=11, BR=12)

**Why this matters:** Being off by even 5 degrees will cause modules to point
slightly wrong, making the robot crab sideways and wear tires unevenly.

Reference: https://broncbotz3481.github.io/YAGSL-Example/configuring/encoders.html

### 2.4 PID Values (Start Conservative)

The files in `swervedrive/` contain PID constants. The defaults might not match
your robot's mass, gearing, or battery voltage.

For initial testing, use these conservative values:
- Angle PID (steering): `p=0.01, i=0, d=0`
- Drive PID (velocity): `p=0.002, i=0, d=0`

These will be slow but stable. Section 6 covers tuning them properly with SysId.

Reference: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html

================================================================================
SECTION 3: PHOTONVISION SETUP
================================================================================

Vision extends your robot's awareness beyond just encoders and gyros. When set
up correctly, AprilTags let the robot know exactly where it is on the field.

### 3.1 Camera Mounting & Calibration

**Physical Mounting:**
- Mount camera securely (no wobbling during hard stops)
- Clear view of field with minimal obstructions
- Avoid facing directly into lights if possible
- For best results, angle camera down 10-30 degrees

**PhotonVision Calibration:**
1. Access PhotonVision UI (usually http://photonvision.local:5800)
2. Go to Settings → Cameras and run camera calibration
3. Print the calibration target and follow the wizard
4. This step is CRITICAL - skip it and your pose estimates will be garbage

Reference: https://docs.photonvision.org/en/latest/docs/calibration/calibration.html

### 3.2 Robot-to-Camera Transform

The code needs to know where the camera is mounted. Measure from the robot's
CENTER POINT on the floor to the camera lens.

Open `Vision.java` and look for the `Cameras` enum (currently empty). When you
add a camera, you'll need:

**Translation (position):**
- X: + forward, - backward (in meters)
- Y: + left, - right (in meters)
- Z: + up from floor to lens center (in meters)

**Rotation (orientation):**
- Roll: rotation around X (0 if camera is level)
- Pitch: rotation around Y (+ = angled up, - = angled down)
- Yaw: rotation around Z (0 = forward, 90 = left, -90 = right)

**Example entry:**
```java
FRONT_CAM("frontcam",
          new Rotation3d(0, Math.toRadians(-15), 0),  // 15° down
          new Translation3d(0.3, 0, 0.2),              // 30cm forward, 20cm up
          VecBuilder.fill(4, 4, 8),                    // Single tag uncertainty
          VecBuilder.fill(0.5, 0.5, 1))                // Multi tag uncertainty
```

Reference: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html

### 3.3 Field Layout (2025 vs 2026)

The code currently loads the 2025 Reefscape field layout because the 2026 game
hasn't been released yet.

**IMPORTANT:** When the 2026 game is announced, update Vision.java line 53:
```java
AprilTagFields.k2025ReefscapeAndyMark  →  AprilTagFields.k2026[GameName]
```

If you forget this step and run on a 2026 field, the robot will use the wrong
tag positions and think it's somewhere it's not.

### 3.4 Trust Settings (Standard Deviations)

The pose estimator fuses vision with odometry. Standard deviations control how
much to trust each measurement. Lower = more trust, higher = less trust.

**Defaults in Vision.java:**
- Single tag: `(4, 4, 8)` - less trustworthy
- Multi tag: `(0.5, 0.5, 1)` - very trustworthy

**If robot "jumps" or "teleports" when seeing tags:**
Increase the values to trust vision less and odometry more.
Example: `(6, 6, 10)` for single tag, `(1, 1, 2)` for multi tag

Reference: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html

================================================================================
SECTION 4: FIRST DRIVE - TESTING & VERIFICATION
================================================================================

Time to see if everything works. Follow these steps IN ORDER.

### Step 1: CAN Bus Health Check
1. Power on robot (on blocks, wheels can spin)
2. Open Driver Station
3. Check CAN bus utilization in the diagnostics tab
4. Should be <80%, ideally <50%
5. Zero errors or bus-off devices

**If you see errors:** Check physical connections, verify no duplicate IDs

### Step 2: Deploy Code
```bash
cd FRC-2026/2026Robot
./gradlew deploy
```

Watch for build errors. If everything compiles, code will auto-start on the RIO.

### Step 3: Module Alignment Check
1. Open AdvantageScope or SmartDashboard
2. Look for "Swerve/Modules" or similar telemetry
3. Physically point FL module forward
4. Dashboard should show ~0 degrees (±5° is acceptable)
5. If it shows ~180°, your offset is backwards
6. Check all four modules

### Step 4: Drive Direction Test
1. Enable robot in Teleop mode (KEEP IT ON BLOCKS)
2. Gently push left joystick forward
3. Observe ALL wheels:
   - ✓ All spin to push robot forward
   - ✗ All spin backward → Fix: Set `inverted.drive: true` in module JSONs
   - ✗ Mixed (some forward, some back) → Fix: Check bevel gear orientation

### Step 5: Steering Direction Test
1. Gently rotate right joystick (or left stick rotation)
2. All modules should turn in the same rotational direction:
   - Stick left → modules rotate counterclockwise (viewed from above)
   - Stick right → modules rotate clockwise
   - If backwards → Fix: Set `inverted.angle: true` (rare for MK4i)

### Step 6: Gyro Alignment
1. Place robot on ground facing AWAY from driver station
2. Enable teleop
3. Press the configured zero button (usually controller Start button)
4. Push forward on stick → robot should drive away from you

**If robot drives sideways or at an angle:** Gyro and modules disagree on what
"forward" means. Recheck module offsets in Section 2.3.

================================================================================
SECTION 5: PATHPLANNER AUTONOMOUS
================================================================================

Autonomous uses PathPlanner to generate smooth trajectories. A few settings can
make or break your auto.

### 5.1 Robot Dimensions
Open `src/main/deploy/pathplanner/settings.json`:
- `robotWidth`: Frame width + bumpers (meters)
- `robotLength`: Frame length + bumpers (meters)

**Measure with bumpers ON.** If you tell PathPlanner the robot is 0.6m wide but
it's actually 0.8m with bumpers, you'll hit walls.

### 5.2 Speed Constraints
Same file, check:
- `defaultMaxVel`: Maximum robot speed (m/s)
- `defaultMaxAccel`: Maximum acceleration (m/s²)

**Test your actual max speed first:**
1. Drive robot in teleop at full throttle
2. Check velocity in telemetry (usually 3.5-4.5 m/s for L2 gearing)
3. Set `defaultMaxVel` to 80% of measured max
4. Set `defaultMaxAccel` to 3-4 m/s²

**If paths fail or robot stops mid-auto:** Speed constraints are too aggressive.

Reference: https://pathplanner.dev/pplib-getting-started.html

### 5.3 Auto PID Tuning
The auto path follower uses PID to stay on track. These are separate from the
module PIDs. Check `Constants.java` under `AutonConstants`:

- `TRANSLATION_PID`: Controls position error (X/Y)
- `ROTATION_PID`: Controls heading error

**Symptoms:**
- Robot wanders off path → Increase TRANSLATION_P
- Robot oscillates/fishtails → Decrease TRANSLATION_P
- Doesn't face correct direction → Increase ROTATION_P

Start with `TRANSLATION_P = 5.0` and `ROTATION_P = 5.0`, tune from there.

================================================================================
SECTION 6: ADVANCED TUNING - SYSID
================================================================================

SysId (System Identification) automatically calculates the perfect PID values
for your specific robot. The difference between guessed PIDs and SysId PIDs is
the difference between "works okay" and "works great."

### 6.1 Running SysId Tests
1. Deploy code with SysId routines enabled (already in template)
2. Open Driver Station, switch to **Test Mode**
3. Clear at least 15 feet of space in front of robot
4. Hold controller X button → robot runs drive characterization
5. Hold controller Y button → robot runs angle characterization
6. Tests generate `.wpilog` files on the RoboRIO

### 6.2 Analyzing Results
1. Download WPILib SysId tool (comes with WPILib installer)
2. Open the tool and load your `.wpilog` file
3. Run the analysis wizard
4. Tool will calculate `kS`, `kV`, `kA`, and `kP` values
5. Update these in your swerve configuration files

**Where to put values:**
- Drive gains → `controllerproperties.json` drive section
- Angle gains → `controllerproperties.json` angle section

Reference: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html

================================================================================
SECTION 7: COMMON ISSUES & TROUBLESHOOTING
================================================================================

**Robot drives in circles:**
→ Check module locations match physical robot (Section 2.1)
→ Verify gyro is mounted flat and `invertedIMU` is correct

**Modules jitter/oscillate when stopped:**
→ Angle PID `p` value too high, reduce by 50%

**Robot is sluggish/doesn't reach commanded speed:**
→ Drive PID `p` value too low, increase gradually
→ Or battery is low (check voltage)

**Vision "teleports" robot position:**
→ Increase standard deviations (Section 3.4)
→ Recalibrate camera (Section 3.1)
→ Check camera mounting is secure (wobbling = bad data)

**Auto path doesn't run:**
→ Check path constraints vs actual robot speed (Section 5.2)
→ Verify path starts at robot's current pose

**CAN errors on startup:**
→ Check for duplicate IDs
→ Verify CAN wiring (twisted pairs, proper termination)
→ Update firmware on all devices

**Wheels fight each other (some forward, some backward):**
→ Bevel gears are inconsistent, fix `inverted.drive` (Section 2.2)

================================================================================
SECTION 8: COMPETITIVE ADVANTAGE ROADMAP (DRIVER ASSIST + TARGETING)
================================================================================

This section documents the early-season strategy for adding competitive
advantages while keeping the RoboRIO 1 stable and responsive.

### 8.1 Compute Placement (RoboRIO 1 Friendly)
Goal: Keep heavy vision math off the RoboRIO 1.
- Coprocessor (Intel N100 + Arducam, likely 2 cameras) handles:
  - AprilTag detection
  - Pose estimation
  - Target data (tag ID, target pose, latency, confidence)
- RoboRIO handles:
  - Fusing time-stamped vision poses into the estimator
  - Final turret aim + motion compensation
  - Driver assist commands

Reasoning: The N100 can handle the heavy vision load; the RoboRIO stays focused
on real-time control loops. Latency compensation for turret aim is done on the
RIO because it has the freshest gyro/odometry data.

### 8.2 Turret Targeting Pipeline (Clean + Expandable)
Split targeting into three layers:
1) Perception: robot pose + target pose (from coprocessor + estimator)
2) Ballistics: distance -> shooter RPM + hood angle (lookup table)
3) Aim/Lead: turret angles with optional motion compensation (shoot-on-the-move)

This keeps "fixed goal" easy and lets us enable "shooting while moving" later
by adding lead correction using predicted robot pose at shot time.

### 8.3 Shooter Lookup Table (External JSON)
We will store shooter tuning in JSON for fast iteration without rebuilding.
- Format: distance (m) -> { rpm, hoodAngleDeg }
- Use linear interpolation between points.
- Keep JSON in deploy assets so it can be updated in the pits.

### 8.4 Driver Assist Features (Selectable in Constants)
We will implement multiple driver assist modes and allow exactly one at a time:
- Auto path to pose (PathPlanner pathfind)
- Auto align heading to target
- Micro adjust (rate-limited OR position-nudge; one enabled at a time)

All bindings will be gated by a single enum in Constants to keep code clean and
make A/B testing easy.

### 8.5 Telemetry Strategy (Low Bandwidth First)
We have not chosen AdvantageScope vs Glass yet. To keep options open:
- Log full data to DataLog for AdvantageScope replay
- Publish minimal live NT fields for Glass/DS
- Throttle update rates to protect RoboRIO CPU and NT bandwidth
- Use tuning mode for short sessions to capture richer drivetrain data (module currents,
  temps, voltages, applied output) for AI-assisted analysis

### 8.6 NetworkTables Key Schema (Draft)
We will standardize keys early so multiple subsystems can rely on them:

Coprocessor -> RoboRIO (vision):
- `/Vision/robotPose` (Pose3d or array)
- `/Vision/poseTimestamp` (seconds)
- `/Vision/poseStdDevs` (x, y, theta)
- `/Vision/targets/tagId` (int)
- `/Vision/targets/fieldPose` (Pose3d or array)
- `/Vision/targets/valid` (bool)
- `/Vision/targets/latencyMs` (double)

RoboRIO -> Driver Station (status/telemetry):
- `/Robot/pose` (Pose2d)
- `/Robot/driveMode` (string/enum)
- `/Robot/assistMode` (string/enum)
- `/Robot/visionEnabled` (bool)
- `/Robot/targetLock` (bool)
- `/Robot/shotSolution/rpm`
- `/Robot/shotSolution/hoodDeg`
- `/Robot/shotSolution/flightTimeSec`
- `/Robot/driveCurrentA` (array, per module)
- `/Robot/angleCurrentA` (array, per module)
- `/Robot/driveTempC` (array, per module)
- `/Robot/angleTempC` (array, per module)
- `/Robot/driveVoltageV` (array, per module)
- `/Robot/angleVoltageV` (array, per module)
- `/Robot/driveAppliedOutput` (array, per module)
- `/Robot/angleAppliedOutput` (array, per module)

### 8.7 Shooter Lookup JSON Format (Draft)
Store in deploy assets, e.g. `src/main/deploy/shooter/shooter_table.json`:
```
{
  "version": 1,
  "units": { "distance": "m", "rpm": "rpm", "hoodAngle": "deg" },
  "points": [
    { "distance": 1.5, "rpm": 2800, "hoodAngle": 18.0 },
    { "distance": 2.5, "rpm": 3200, "hoodAngle": 22.0 },
    { "distance": 3.5, "rpm": 3600, "hoodAngle": 27.0 }
  ]
}
```

================================================================================
FINAL NOTES
================================================================================

Remember:
- Measure twice, deploy once
- Test on blocks before testing on ground
- When something goes wrong, check the logs (RoboRIO or Driver Station)
- Ask for help early - the forum and Chief Delphi are great resources

Good luck this season! 🤖

================================================================================
