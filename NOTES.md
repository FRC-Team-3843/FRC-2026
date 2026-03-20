# Working Notes - FRC-2026

> **Documentation Guide:**
> - **This file (NOTES):** Setup, tuning, troubleshooting, TODOs, game details
> - **README.md:** Project overview and quick start
> - **STANDARDS.md:** Coding standards and architecture rules

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
- Game Manual: https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
- Field Drawings: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
- Team Updates: https://firstfrc.blob.core.windows.net/frc2026/Manual/TeamUpdates/REBUILT_TeamUpdate-Combined.pdf

================================================================================
SECTION 0: 2026 GAME - REBUILT (presented by Haas)
================================================================================

### Game Overview

In REBUILT, two 3-robot alliances score FUEL into their HUB, traverse field
obstacles, and climb the TOWER before time runs out. The unique mechanic is
HUB SHIFTING -- alliance hubs alternate between active and inactive during
teleop, based on which alliance performed better in autonomous.

Theme: Archaeology-inspired -- "re-imagine the past, uncover discoveries."

### Match Structure (2 minutes 40 seconds = 160 seconds total)

```
Phase            Duration   Hub Status
-----------      --------   ------------------------------------------
Autonomous       20 sec     Both hubs ACTIVE
Transition       10 sec     Both hubs ACTIVE
Shift 1          25 sec     Alternating (Auto winner's hub INACTIVE)
Shift 2          25 sec     Alternating (Auto winner's hub ACTIVE)
Shift 3          25 sec     Alternating (Auto winner's hub INACTIVE)
Shift 4          25 sec     Alternating (Auto winner's hub ACTIVE)
Endgame          30 sec     Both hubs ACTIVE
```

**Hub Shifting Rule:** The alliance that scores MORE fuel during Auto has their
hub go INACTIVE first (Shifts 1 & 3 inactive, Shifts 2 & 4 active). The losing
alliance gets the opposite pattern. This rewards strong autonomous programs.

### Game Data (for programming)

A single character is sent via `DriverStation.getGameSpecificMessage()`:
- `'R'` = Red alliance hub goes inactive first
- `'B'` = Blue alliance hub goes inactive first
- Sent ~3 seconds after autonomous ends (empty string before that)
- Reference: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html

### Game Piece: FUEL

- **Type:** Ball
- **Diameter:** 5.91 inches (~15 cm)
- **Weight:** 0.448 - 0.5 lbs
- **Total on field:** 504 fuel
- **Robot preload:** 8 fuel per robot (24 per alliance)
- **Sources:** Preload, human player (Outpost), Depot, neutral zone floor
- **Control limit:** No limit -- robots may hold any amount of fuel at a time

### Scoring Breakdown

**Hub Scoring (Fuel):**
- Each fuel scored in an ACTIVE hub = 1 point
- Fuel scored in an INACTIVE hub = 0 match points
- No difference in point value between Auto and Teleop
- Human players can also score fuel into the hub from the Outpost
- Hub recycles scored fuel back onto the field through shoots into Neutral Zone

**Tower Climbing:**
| Phase   | Level 1 | Level 2 | Level 3 |
|---------|---------|---------|---------|
| Auto    | 15 pts  | --      | --      |
| Teleop  | 10 pts  | 20 pts  | 30 pts  |

**Climbing Level Definitions:**
- Level 1: Robot fully supported by TOWER, not touching carpet or TOWER BASE
- Level 2: Robot's BUMPERS completely above the LOW RUNG
- Level 3: Robot's BUMPERS completely above the MID RUNG

Robots earn tower points for only one LEVEL during AUTO and one during TELEOP.
There is no requirement to progress through levels sequentially -- you can
attempt Level 3 directly, though the 30" height limit makes this impractical.

### Ranking Points (Regional/District Events)

| RP               | Requirement                          |
|------------------|--------------------------------------|
| Win RP           | Win match (3 RP), Tie (1 RP each)   |
| Energized RP     | Score >=100 fuel in active hub (1 RP)|
| Supercharged RP  | Score >=360 fuel in active hub (1 RP)|
| Traversal RP     | Earn >=50 tower climbing pts (1 RP)  |

**RP Strategy Notes:**
- Energized (100 fuel) is achievable by most alliances
- Supercharged (360 fuel) requires high cycle rate (~3-4 fuel/sec sustained)
- Traversal (50 tower pts) needs at least 2 robots climbing Level 2+
  (e.g., two Level 2 = 40 pts + one Level 1 = 10 pts = 50 pts)

### Field Layout

**Dimensions:** ~317.7" x 651.2" (~8.07m x 16.54m) carpeted area

**Field Elements (mirrored for each alliance):**

| Element   | Description                                          | Key Dimensions                    |
|-----------|------------------------------------------------------|-----------------------------------|
| Hub       | Central scoring structure, rectangular prism          | 47" x 47", opening at top         |
| Tower     | 3-rung climbing structure                             | 49.25"W x 45"D x 78.25"H         |
| Trench    | Low-clearance field crossing (faster for short bots)  | 50.34"W x 22.25"H clearance       |
| Bump      | Ramp-style field crossing (any bot can use)           | ~6.5" peak height, two ramps      |
| Depot     | Enclosed fuel storage area with low walls             | 1" tall steel barriers            |
| Outpost   | Human player station for exchanging fuel              | Alliance wall area                |

**Tower Details:**
- 3 RUNGS at increasing heights, 18" apart center-to-center
- Rungs are 1-1/4" Schedule 40 pipe
- Supports available to assist climbing
- Scored level based on BUMPER position relative to rungs

**Hub Details:**
- Rectangular prism with extended opening at top surface
- Includes net blocking shots from prohibited zones
- Scored fuel is recycled back to the Neutral Zone through shoots

**Trench vs Bump:**
- Trench: 22.25" tall clearance -- robots must be short enough to fit
- Bump: ~6.5" ramps -- any robot can cross but may be slower
- Strategic choice: build a low-profile robot for trench speed advantage?

### Robot Constraints (2026)

| Rule  | Constraint                                              |
|-------|---------------------------------------------------------|
| R104  | Robot perimeter <= 110" (frame perimeter)               |
| --    | Starting height <= 3'6" (~42")                          |
| --    | Weight (robot + bumpers) <= 135 lbs                     |
| --    | Extension beyond perimeter <= 12" in one direction      |
| --    | Extension above floor <= 30" vertically                 |
| --    | Bumper height zone: 2.75" to 5.5" from floor            |
| --    | Bumper extension: <= 4.25" from perimeter               |

### Design Considerations for Team 3843

**Fuel Intake:**
- 5.91" balls -- similar to 2017 STEAMWORKS fuel but larger
- Need reliable ground pickup from depot and neutral zone
- Consider hopper/magazine for holding multiple fuel
- Simulator data shows top robots carry 14+ fuel per cycle

**Shooting/Scoring:**
- Hub is stationary with top opening -- consistent target
- No point difference for shot position (no upper/lower distinction)
- Human players can also score -- coordinate with HP strategy
- Accuracy matters: 92%+ accuracy targets from simulator data

**Tower Climbing:**
- Level 3 = 30 pts is the high-value endgame target
- Rungs are 1-1/4" pipe, 18" apart -- design hooks/arms accordingly
- 30" height limit is the primary constraint during climbing
- Consider: can the climber mechanism also help with intake/shooting?

**Field Traversal:**
- Trench clearance is 22.25" -- robot must be under this with mechanisms stowed
- Bump is 6.5" -- swerve drive handles this, but intake must survive the jolt
- Low-profile robot gains strategic advantage (trench shortcut)

**Autonomous Priority:**
- Winning auto determines favorable hub shifting schedule
- Pre-load 8 fuel per robot = 24 fuel alliance auto potential
- Auto climb to Level 1 = 15 pts (high value if achievable)
- Strong auto = hub active during Shifts 2 & 4 (more active time overall)

### Mechanism Hardware Summary

**Double Turret Shooter Design:**
The robot uses a mirrored double turret system. Each turret independently aims
and fires FUEL into the Hub.

| Component | Motor | Controller | Gear Ratio | Notes |
|-----------|-------|------------|------------|-------|
| Turret rotation (x2) | NEO 550 | SparkMax | 52.987:1 (11t>68t, 14t>120t) | Both locked together, field-centric, +/-110 deg |
| Preshooter (x2) | Kraken X44 | TalonFX | 5:8 = 0.625 (overdrive) | Runs at 66.7% of main shooter RPM |
| Main shooter (x2) | Kraken X60 | TalonFX | 66t:72t = 0.917 (overdrive) | 4" wheel, max 5500 RPM |
| Intake roller | Kraken X44 | TalonFX (CAN 30) | N/A | Ground pickup, passive deploy |
| Floor conveyor | NEO 550 | SparkMax (CAN 31) | N/A | Intake -> hopper |
| Hopper | Bag motor | TalonSRX (CAN 32) | N/A | Fuel agitation (Phoenix5) |
| Feeder | NEO 550 | SparkMax (CAN 33) | N/A | Hopper -> both shooters |
| Hood servos (x2) | Standard servo | PWM 0/1 | N/A | Binary near/far angle adjust |

**Robot Physical Properties:**
- Total mass with bumpers + battery: ~140 lbs (63.5 kg)
- Bumper-to-bumper dimensions: 33" x 33" (0.8382m x 0.8382m)
- Frame/wheelbase: 27" x 27" (0.6858m x 0.6858m)

**Swerve Drive Gearing (MK4 L1 standard, no modifications):**
- Drive gear ratio: 8.14:1
- Angle gear ratio: 12.8:1
- Theoretical max speed: 3.71 m/s
- Wheel diameter: 4" (nominal — calibrate with drive distance test)
- NEO free speed: 5676 RPM

### Key Resources

- Game Manual (PDF): https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
- Field Dimension Drawings: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
- Team Updates (Combined): https://firstfrc.blob.core.windows.net/frc2026/Manual/TeamUpdates/REBUILT_TeamUpdate-Combined.pdf
- WPILib Game Data: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
- FIRST Season Page: https://www.firstinspires.org/programs/frc/game-and-season
- Tower Build Instructions: https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/TE-26500-build-instructions.pdf
- Official FUEL (AndyMark): https://andymark.com/products/official-rebuilt-fuel
- Community Resources: https://www.chiefdelphi.com/t/all-the-rebuilt-resources/510081

================================================================================
CAN BUS ASSIGNMENTS
================================================================================

IMPORTANT: These IDs must match what is programmed into each physical device.
Use Phoenix Tuner X for TalonFX/TalonSRX and REV Hardware Client for SparkMax.

### Swerve Drive System (IDs 11-22)
| Device | CAN ID | Type | Config Tool |
|--------|--------|------|-------------|
| FL Drive Motor | 11 | SparkMax (NEO) | REV Hardware Client |
| FR Drive Motor | 12 | SparkMax (NEO) | REV Hardware Client |
| BL Drive Motor | 13 | SparkMax (NEO) | REV Hardware Client |
| BR Drive Motor | 14 | SparkMax (NEO) | REV Hardware Client |
| FL Angle Motor | 15 | SparkMax (NEO) | REV Hardware Client |
| FR Angle Motor | 16 | SparkMax (NEO) | REV Hardware Client |
| BL Angle Motor | 17 | SparkMax (NEO) | REV Hardware Client |
| BR Angle Motor | 18 | SparkMax (NEO) | REV Hardware Client |
| FL CANCoder | 19 | CANCoder | Phoenix Tuner X |
| FR CANCoder | 20 | CANCoder | Phoenix Tuner X |
| BL CANCoder | 21 | CANCoder | Phoenix Tuner X |
| BR CANCoder | 22 | CANCoder | Phoenix Tuner X |

Swerve gear ratios (MK4 L1, no modifications):
- Drive: 8.14:1, Angle: 12.8:1, Max speed: 3.71 m/s, Wheel: 4"

### Mechanism Motors (IDs 30-39)
| Device | CAN ID | Controller | Motor | Notes |
|--------|--------|------------|-------|-------|
| Intake | 30 | TalonFX | Kraken X44 | Ground intake roller |
| Floor Conveyor | 31 | SparkMax | NEO 550 | Intake -> hopper transport |
| Hopper | 32 | TalonSRX | Bag motor | Fuel agitation/feed (Phoenix5 API) |
| Feeder | 33 | SparkMax | NEO 550 | Hopper -> both shooters |
| Left Turret | 34 | SparkMax | NEO 550 | 52.987:1 (11t>68t, 14t>120t), seeds at zero, +/-90 deg |
| Right Turret | 35 | SparkMax | NEO 550 | 52.987:1 (11t>68t, 14t>120t), seeds at zero, +/-90 deg |
| Left Preshooter | 36 | TalonFX | Kraken X44 | 5:8 = 0.625 overdrive |
| Right Preshooter | 37 | TalonFX | Kraken X44 | 5:8 = 0.625 overdrive |
| Left Main Shooter | 38 | TalonFX | Kraken X60 | 66t:72t = 0.917 overdrive, 4" wheel |
| Right Main Shooter | 39 | TalonFX | Kraken X60 | 66t:72t = 0.917 overdrive, 4" wheel |

### PWM Channels (Hood Servos)
| Device | Channel | Notes |
|--------|---------|-------|
| Left Hood Servo | 0 | Near=45 deg, Far=90 deg (tune on hardware) |
| Right Hood Servo | 1 | Near=45 deg, Far=90 deg (tune on hardware) |

### NavX IMU
Connected via SPI port on RoboRIO (no CAN ID).

================================================================================
SECTION 0B: COMPETITION DAY SETUP (2026-03-18)
================================================================================

Step-by-step for first power-on at competition. Do these IN ORDER.

### Step 1: Flash CAN IDs
Using Phoenix Tuner X and REV Hardware Client, set every device to its assigned
CAN ID from the table above. Verify all devices appear on the CAN bus with no
errors before proceeding.

### Step 2: Verify Motor Directions
Power each mechanism individually at low duty cycle and observe rotation.
If a motor runs backward, set the corresponding MOTOR_INVERTED constant to true
in Constants.java and redeploy. Constants to check:
  - IntakeConstants.MOTOR_INVERTED
  - ConveyorConstants.MOTOR_INVERTED
  - HopperConstants.MOTOR_INVERTED
  - FeederConstants.MOTOR_INVERTED
  - TurretConstants.LEFT_INVERTED / RIGHT_INVERTED
  - ShooterConstants.LEFT_MAIN_INVERTED / RIGHT_MAIN_INVERTED
  - ShooterConstants.LEFT_PRE_INVERTED / RIGHT_PRE_INVERTED

### Step 3: SysId the Swerve
1. Deploy code, connect to robot
2. Manually zero gyro: driver Start button
3. Enter Test mode in Driver Station
4. Driver X = characterize drive motors (forward/back quasistatic + dynamic)
5. Driver Y = characterize angle motors
6. Open AdvantageScope -> import log -> analyze with SysId tool
7. Update pidfproperties.json with resulting kP values

### Step 4: Zero Turrets and Test
1. Physically rotate BOTH turrets to point straight forward (robot front)
2. Enable robot, operator presses Start -> zeros turret encoders to 0
3. Push operator left stick to confirm turret tracks joystick direction
4. Check direction: forward stick = turret stays forward, right stick = turret turns right
   If reversed: set TurretConstants.LEFT_INVERTED or RIGHT_INVERTED and redeploy

### Step 5: Tune Shooter
1. Press operator D-pad UP (CLOSE preset), A to fire
2. Observe ball trajectory. Adjust ShootingPositions.CLOSE.mainRpm() up/down
3. Adjust HoodConstants.NEAR_ANGLE if hood angle is wrong
4. Repeat for D-pad DOWN (FAR preset) with ShootingPositions.FAR values
5. All presets are in Constants.java -> ShootingPositions

### Step 6: Tune Hood Servos
Hood servo angles (0-180 deg WPILib):
  - HoodConstants.NEAR_ANGLE = 45.0 (starting value, tune up/down)
  - HoodConstants.FAR_ANGLE  = 90.0 (starting value, tune up/down)
Operator LBumper = near, RBumper = far for manual override anytime.

### Controller Quick Reference
DRIVER (port 0):
  Left stick     = translate (field-relative)
  Right stick X  = rotate
  Left trigger   = slow mode (50%)
  Right bumper   = intake roller + floor conveyor (hold)
  Left bumper    = reverse intake + floor conveyor (hold)
  Start          = zero gyro (face downfield first!)
  POV Up         = X-lock wheels
  Back           = center modules to 0 deg

OPERATOR (port 1):
  Left stick dir = aim both turrets (field-centric: forward = downfield always)
  Left stick mag = spin up both shooters (more push = more RPM)
  A (hold)       = feed chain (floor conveyor + hopper + feeder)
  B (hold)       = reverse feed chain
  D-pad Up       = CLOSE preset (2500 RPM, hood near)
  D-pad Down     = FAR preset (4000 RPM, hood far)
  D-pad Left     = ANGLE_LEFT preset (5000 RPM, +45 deg)
  D-pad Right    = ANGLE_RIGHT preset (5000 RPM, -45 deg)
  Y              = cancel preset, back to manual joystick
  L bumper       = hood near
  R bumper       = hood far
  Start          = zero turret encoders (align to forward first!)

TEST MODE (driver controller):
  X              = SysId swerve drive (~11s)
  Y              = SysId swerve angle (~11s)
  A              = SysId main shooter (~20s)
  B              = SysId preshooter (~20s)
  LB             = SysId turret (~22s)

### Vision (if cameras get mounted)
1. Beelink Mini S coprocessor is already prepped (PhotonVision + SSH + robot-side static IP)
2. Mount the right camera rigidly and plug it into the Beelink
3. In PhotonVision, create the camera as `right_cam`
4. Calibrate the camera and verify live AprilTag detections
5. If pose looks sane, set VisionConstants.ENABLE_VISION = true in Constants.java
6. Redeploy and validate pose estimation on the field

================================================================================
SECTION 1: PRE-FLIGHT CHECKLIST
================================================================================

Before you write a single line of code, verify the hardware is ready:

**Power & CAN Bus:**
1. All motor controllers have correct CAN IDs (see CAN Bus Assignments below)
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

**IMPORTANT: Use the WPILib JDK for builds.** The project requires Java 17.
WPILib installs its own JDK at `C:\Users\Public\wpilib\2026\jdk`.
If Gradle fails with a Java version error, set JAVA_HOME before building:
```bash
JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew build
```

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
2. Look at each bevel gear - are they all on the same sideNO
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

### 3.0 Competition Coprocessor Status (2026-03-20)

The Beelink Mini S vision computer was set up and verified before pit install.

**Current machine state:**
- Ubuntu 24.04.4 LTS
- PhotonVision installed and running as a `systemd` service on boot
- PhotonVision version: `v2026.3.2`
- OpenSSH enabled and running on boot
- Hostname set to `photonvision`
- Sleep/suspend/hibernate masked for headless operation
- GRUB timeout reduced to 5 seconds
- Wi-Fi management IP at time of setup: `192.168.100.215`
- PhotonVision networking settings verified:
  - Team number set to `3843`
  - NetworkTables server left `off` (client mode)
  - Managed robot Ethernet set to static `10.38.43.11`

**How to reach it:**
- Over current Wi-Fi: `http://192.168.100.215:5800`
- On the robot network: `http://10.38.43.11:5800`
- SSH on robot network: `ssh murray-robotics-3843@10.38.43.11`

**Important:**
- The Beelink is ready to run headless once mounted on the robot
- BIOS auto-power-on after power loss has been configured by the team
- Hotspot/Wi-Fi IPs are temporary; prefer `10.38.43.11` once the Beelink is on the robot radio
- If time allows, confirm the behavior once on the robot by cycling power through the final wiring path

### 3.1 Camera Mounting & Calibration

**Physical Mounting:**
- Mount camera securely (no wobbling during hard stops)
- Clear view of field with minimal obstructions
- Avoid facing directly into lights if possible
- For best results, angle camera down 10-30 degrees

**PhotonVision Calibration:**
1. Access PhotonVision UI (`http://10.38.43.11:5800` on the robot network)
2. Go to Settings -> Cameras and run camera calibration
3. Print the calibration target and follow the wizard
4. This step is CRITICAL - skip it and your pose estimates will be garbage

Reference: https://docs.photonvision.org/en/latest/docs/calibration/calibration.html

### 3.2 Robot-to-Camera Transform

The code needs to know where the camera is mounted. Measure from the robot's
CENTER POINT on the floor to the camera lens.

`Vision.java` already contains prewired `RIGHT_CAM` and `LEFT_CAM` entries. The
current constants assume front-corner camera mounts and should be treated as
starting estimates, not final surveyed values.

**Current estimated geometry in code:**
- Height: `19 in`
- Front offset from bumper edge: `0.25 in`
- Side offset from bumper edge: `0.25 in`
- Pitch: `-10 deg` (downward)
- Right camera yaw: `-45 deg`
- Left camera yaw: `+45 deg`
- PhotonVision names: `right_cam`, `left_cam`
- Vision enable flag remains `false` until the real camera is mounted and tested

If these mounts change, update `Constants.VisionConstants`.

When re-measuring a camera, you'll need:

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
          new Rotation3d(0, Math.toRadians(-15), 0),  // 15 deg down
          new Translation3d(0.3, 0, 0.2),              // 30cm forward, 20cm up
          VecBuilder.fill(4, 4, 8),                    // Single tag uncertainty
          VecBuilder.fill(0.5, 0.5, 1))                // Multi tag uncertainty
```

Reference: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html        

### 3.3 Field Layout (DONE)

Vision.java has been updated to load the 2026 REBUILT AprilTag field layout:
```java
AprilTagFields.k2026RebuiltAndymark
```

This was completed on 2026-02-15. No further action needed for field layout.

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
4. Dashboard should show ~0 degrees (+/-5 deg is acceptable)
5. If it shows ~180 deg, your offset is backwards
6. Check all four modules

### Step 4: Drive Direction Test
1. Enable robot in Teleop mode (KEEP IT ON BLOCKS)
2. Gently push left joystick forward
3. Observe ALL wheels:
   - OK All spin to push robot forward
   - NO All spin backward -> Fix: Set `inverted.drive: true` in module JSONs
   - NO Mixed (some forward, some back) -> Fix: Check bevel gear orientation

### Step 5: Steering Direction Test
1. Gently rotate right joystick (or left stick rotation)
2. All modules should turn in the same rotational direction:
   - Stick left -> modules rotate counterclockwise (viewed from above)
   - Stick right -> modules rotate clockwise
   - If backwards -> Fix: Set `inverted.angle: true` (rare for MK4i)

### Step 6: Gyro Alignment
1. Place robot on ground facing AWAY from driver station
2. Enable teleop
3. Press the configured zero button (usually controller Start button)
4. Push forward on stick -> robot should drive away from you

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
- `defaultMaxAccel`: Maximum acceleration (m/s^2)

**Test your actual max speed first:**
1. Drive robot in teleop at full throttle
2. Check velocity in telemetry (usually 3.5-4.5 m/s for L2 gearing)
3. Set `defaultMaxVel` to 80% of measured max
4. Set `defaultMaxAccel` to 3-4 m/s^2

**If paths fail or robot stops mid-auto:** Speed constraints are too aggressive.

Reference: https://pathplanner.dev/pplib-getting-started.html

### 5.3 Auto PID Tuning
The auto path follower uses PID to stay on track. These are separate from the
module PIDs. Check `Constants.java` under `AutonConstants`:

- `TRANSLATION_PID`: Controls position error (X/Y)
- `ROTATION_PID`: Controls heading error

**Symptoms:**
- Robot wanders off path -> Increase TRANSLATION_P
- Robot oscillates/fishtails -> Decrease TRANSLATION_P
- Doesn't face correct direction -> Increase ROTATION_P

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
4. Hold controller X button -> robot runs drive characterization
5. Hold controller Y button -> robot runs angle characterization
6. Tests generate `.wpilog` files on the RoboRIO

### 6.2 Analyzing Results
1. Download WPILib SysId tool (comes with WPILib installer)
2. Open the tool and load your `.wpilog` file
3. Run the analysis wizard
4. Tool will calculate `kS`, `kV`, `kA`, and `kP` values
5. Update these in your swerve configuration files

**Where to put values:**
- Drive gains -> `controllerproperties.json` drive section
- Angle gains -> `controllerproperties.json` angle section

Reference: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html

================================================================================
SECTION 7: COMMON ISSUES & TROUBLESHOOTING
================================================================================

**Robot drives in circles:**
-> Check module locations match physical robot (Section 2.1)
-> Verify gyro is mounted flat and `invertedIMU` is correct

**Modules jitter/oscillate when stopped:**
-> Angle PID `p` value too high, reduce by 50%

**Robot is sluggish/doesn't reach commanded speed:**
-> Drive PID `p` value too low, increase gradually
-> Or battery is low (check voltage)

**Vision "teleports" robot position:**
-> Increase standard deviations (Section 3.4)
-> Recalibrate camera (Section 3.1)
-> Check camera mounting is secure (wobbling = bad data)

**Auto path doesn't run:**
-> Check path constraints vs actual robot speed (Section 5.2)
-> Verify path starts at robot's current pose

**CAN errors on startup:**
-> Check for duplicate IDs
-> Verify CAN wiring (twisted pairs, proper termination)
-> Update firmware on all devices

**Wheels fight each other (some forward, some backward):**
-> Bevel gears are inconsistent, fix `inverted.drive` (Section 2.2)

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
SECTION 9: ELASTIC DASHBOARD WORKFLOW
================================================================================

### 9.0 Overview

The robot uses Elastic Dashboard as its primary dashboard. The layout is stored
as a JSON file in the deploy directory and served to the dashboard via a built-in
web server on port 5800.

**Layout file:** `src/main/deploy/elastic-layout.json`
**WebServer:** Started automatically in Robot.java on port 5800

### 9.1 Creating and Editing Layouts

**From Elastic Dashboard UI:**
1. Launch Elastic: VS Code -> `Ctrl+Shift+P` -> `WPILib: Start Tool` -> `Elastic`
2. Connect to robot (or use localhost for simulation)
3. Drag widgets from the NT tree onto tabs
4. Resize and position widgets on the grid
5. Export layout: `Ctrl+Shift+S` -> save as `elastic-layout.json`

**From JSON (programmatic / AI agent):**
1. Edit `src/main/deploy/elastic-layout.json` directly
2. Follow the Elastic Layout Editing Standard in STANDARDS.md
3. Validate JSON before deploying (no trailing commas, valid structure)

### 9.2 Deploying Layouts to Robot

```bash
cd FRC-2026/2026Robot
./gradlew deploy
```

The layout JSON is deployed alongside code to the RoboRIO's deploy directory.
The WebServer serves it on port 5800.

### 9.3 Loading Layouts from Robot

In Elastic Dashboard:
1. `Ctrl+D` or `File` -> `Download Layout from Robot`
2. Enter robot address (e.g., `10.38.43.2` or `roboRIO-3843-frc.local`)
3. Layout loads automatically from the WebServer

### 9.4 TunableNumber Data Flow

The bidirectional tuning system works as follows:

```
Constants.java  -->  TunableNumber(default)  -->  NetworkTables (/Tuning/...)
                                                       |
                                                  Elastic Dashboard
                                                  (user edits value)
                                                       |
                                                  NetworkTables (updated)
                                                       |
DashboardManager.periodic()  <--  TunableNumber.get()  <--  NT value
       |
  subsystem.setPID(newP, newI, newD)  -->  Motor Controller
```

**Key points:**
- Values are only published to NT when `TelemetryConstants.TUNING_MODE = true`
- When tuning mode is false, `get()` returns the compile-time default (zero overhead)
- `hasChanged()` detects when a dashboard user edited the value
- Changes take effect on the next robot periodic cycle (20ms)

### 9.5 Dashboard Tabs

| Tab | Purpose | Audience |
|-----|---------|----------|
| Competition | Field view, mode selectors, status indicators | Driver/Operator |
| Tuning | PID controllers, RPM sliders, tuning guide | Programmer |
| Motor Config | CAN IDs, gear ratios, enable/disable toggles | Programmer |
| SysId | Characterization instructions and graphs | Programmer |
| Diagnostics | Battery, CAN, currents, temps, swerve visualization | Pit crew |
| Raw Sensors | Gyro, encoders, module states, vision data | Debug |

### 9.6 Elastic Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Ctrl+D` | Download layout from robot |
| `Ctrl+Shift+S` | Export layout to file |
| `Ctrl+S` | Save layout |
| `Ctrl+Z` | Undo |
| `Ctrl+Shift+Z` | Redo |
| `Delete` | Remove selected widget |

### 9.7 Troubleshooting

**Layout not loading from robot:**
- Verify WebServer started (check console for errors)
- Check robot address is correct
- Ensure `elastic-layout.json` exists in deploy directory
- Try `http://<robot-ip>:5800/elastic-layout.json` in a browser

**Widgets show "No data":**
- Verify NT topic path matches what robot code publishes
- Check `TelemetryConstants.ENABLE_TELEMETRY` is true
- For tuning widgets, check `TelemetryConstants.TUNING_MODE` is true
- Verify robot code is running (Driver Station shows connected)

**PID widgets not updating:**
- PIDController widgets require Sendable objects published via SmartDashboard.putData()
- DashboardManager publishes these automatically
- Verify DashboardManager.periodic() is called from Robot.robotPeriodic()

================================================================================
SECTION 10: DEVELOPER UTILITIES
================================================================================

### 10.1 Alerts System
A persistent alerts system is available in `frc.robot.utils.Alert`.
Use this to display critical errors (red), warnings (yellow), or info (green)
on the dashboard that persist until resolved.

**Usage:**
```java
// Create alert in your subsystem
private final Alert m_alert = new Alert("Intake jammed!", AlertType.ERROR);

// Set active status in periodic() or when fault detected
m_alert.set(isJammed());
```

================================================================================
FINAL NOTES
================================================================================

Remember:
- Measure twice, deploy once
- Test on blocks before testing on ground
- When something goes wrong, check the logs (RoboRIO or Driver Station)
- Ask for help early - the forum and Chief Delphi are great resources

Good luck this season! robot

================================================================================
