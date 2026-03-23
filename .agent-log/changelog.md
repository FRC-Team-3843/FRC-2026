# Agent Activity Log - FRC-2026 Repository

## Purpose
Track all significant changes made by AI agents (Claude, Gemini, Codex) in the FRC-2026 repository.

**Update Protocol:**
- Log all significant changes (new features, refactors, config updates)
- Use format: `### [YYYY-MM-DD HH:MM] AGENT_NAME [ACTION_TYPE]`
- Action types: `[IMPLEMENT]`, `[REFACTOR]`, `[FIX]`, `[TEST]`, `[CONFIG]`, `[DOCS]`
- Include file paths relative to FRC-2026/ directory
- Add `PENDING:` notes for incomplete work

---

## 2026-03-21

### [2026-03-21 12:00] CLAUDE [FIX]
- Competition day 3 driver control changes
- Repo: FRC-2026
- Changes:
  - Fixed red alliance inverted controls: disabled `allianceRelativeControl` — was double-flipping with `zeroGyroWithAlliance()` causing inverted X/Y on red
  - Center modules moved from Back button to POV Down
  - Slow mode changed from binary 50% toggle (left trigger > 0.5) to progressive scaling (80% at no pull → 20% at full pull)
  - Added right trigger progressive intake (0% → 100% speed for intake roller + floor conveyor)
  - Kept right bumper full-speed intake alongside right trigger
  - Added `setSpeed()` method to IntakeSubsystem for variable-speed control
  - Removed unused `driveAngularVelocitySlow` drive stream
  - Marked completed TODO items: motor inversions verified, FR angle motor OK, shooter presets tuned, hood servo angles dialed in, PathPlanner named commands already registered, auto routines confirmed
  - Added SysId shooter & preshooter to TODO list
- Files modified:
  - `2026Robot/src/main/java/frc/robot/RobotContainer.java` (bindings, drive stream, allianceRelativeControl)
  - `2026Robot/src/main/java/frc/robot/subsystems/IntakeSubsystem.java` (added setSpeed)
  - `2026Robot/TODO-COMPETITION.md` (updated completed/pending items)
- Notes: allianceRelativeControl(true) + zeroGyroWithAlliance() = double-flip on red alliance. Only use ONE alliance compensation method.

---

## 2026-03-20

### [2026-03-20 09:00] CLAUDE [DOCS]
- Updated README.md and NOTES.md to reflect current competition state
- Repo: FRC-2026
- Files modified:
  - `README.md` (mechanisms section: replaced "design in progress" with actual hardware; vision: corrected to 2026 REBUILT layout; development status: updated to competition phase with accurate implemented/in-progress lists)
  - `NOTES.md` (Section 3.3: marked AprilTag field layout update as DONE, removed stale ACTION REQUIRED)
- Notes: Docs were stale from pre-mechanism-implementation era. Code has been competition-ready since 2026-03-18 but docs still referenced design phase.

---

## 2026-03-19

### [2026-03-19 22:00] CLAUDE [FIX]
- Competition day fixes and tuning — numerous iterative deploys throughout the day
- Repo: FRC-2026
- Fixes applied:
  - Fixed swervedrive.json module paths (removed doubled "modules/" prefix causing FileNotFoundException)
  - Fixed PathPlanner settings.json field names (driveWheelRadius, driveGearing, maxDriveSpeed, driveMotorType, module positions)
  - Fixed strafe axis inversion (removed negation on getLeftX for all drive streams)
  - Fixed one-shooter-spinning bug (both shooters now follow left stick magnitude)
  - Fixed feeder A-button not working (simplified trigger, removed complex composition)
  - Fixed SysId test mode conflicts (intake bindings guarded with !isTest())
  - Fixed CAN stale errors in test mode (disabled LiveWindow)
  - Fixed DashboardManager references to renamed constants (KP/KI/KD per-motor)
- Feature changes:
  - Turrets: both locked to left stick, field-centric aiming (compensates for robot rotation)
  - Turret gear ratio: 68:1 -> 52.987:1 (actual: 11t>68t first stage, 14t>120t second stage)
  - Turret soft limits: +/-90 deg -> +/-110 deg (16.2 motor rotations)
  - Turret PID: per-motor values from SysId (left kP=0.15/kD=0.01/70% limit, right kP=0.10/kD=0.008/50% limit)
  - Main shooter gear ratio: 0.8333 -> 0.91667 (66t:72t overdrive)
  - MAX_WHEEL_RPM: 5000 -> 5500 (achievable at low battery)
  - Shooter wheel diameter documented: 4 inches
  - D-pad presets updated with field-research-based RPMs (CLOSE 2500, FAR 4000, ANGLE 5000)
  - Driver intake: now runs intake roller + floor conveyor only (not hopper)
  - Operator A: runs conveyor + hopper + feeder (full feed chain)
  - Operator B: reverses conveyor + hopper + feeder
  - Swerve PID: drive kP=0.005 kFF=0.27, angle kP=0.0106 (from SysId)
  - CANCoder offsets set (FL=329.77, FR=359.74, BL=43.15, BR=77.70 degrees)
  - Intake motor inverted, feeder motor inverted
  - Feeder ENABLED flag: false -> true (after encoder cable repair)
  - SysId added for shooters (driver A/B in test) and turrets (driver LB in test)
  - Elastic dashboard layout rewritten (Match, Shooter, Turret, Diagnostics, Tuning tabs)
- Files modified:
  - Constants.java (gear ratios, CAN IDs, PID values, presets, inversions, MAX_WHEEL_RPM)
  - RobotContainer.java (bindings, drive streams, default commands, SysId bindings)
  - TurretSubsystem.java (field-centric, per-motor PID, heading supplier, SysId)
  - ShooterSubsystem.java (dual magnitude, separate VoltageOut, SysId, gear ratio fix)
  - FeederSubsystem.java (ENABLED flag, re-enabled)
  - HopperSubsystem.java (TalonSRX current limits added)
  - DashboardManager.java (per-motor turret PID constants)
  - Robot.java (LiveWindow disabled in test mode)
  - All swerve module JSONs (CANCoder offsets in degrees)
  - swervedrive.json (fixed module paths)
  - physicalproperties.json (MK4 L1 gear ratios)
  - pidfproperties.json (tuned drive and angle PID)
  - pathplanner/settings.json (correct field names and values)
  - elastic-layout.json (complete rewrite)
  - NOTES.md (updated controller reference, gear ratios, mechanism summary)
  - TODO-COMPETITION.md (updated with completed items and current status)
- PENDING:
  - Remaining motor inversions (conveyor, hopper, turrets, shooter motors)
  - FR angle motor investigation (high friction)
  - TalonFX 38 firmware flash
  - SysId swerve drive (needs space)
  - Tune shooter presets and hood angles on field
  - Register PathPlanner named commands for autonomous
  - Vision setup

---

## 2026-03-18

### [2026-03-18 23:00] CLAUDE [IMPLEMENT]
- Implemented full competition-ready mechanism code for all robot systems
- Repo: FRC-2026
- Files created:
  - `2026Robot/src/main/java/frc/robot/subsystems/IntakeSubsystem.java` (TalonFX, CAN 30)
  - `2026Robot/src/main/java/frc/robot/subsystems/ConveyorSubsystem.java` (SparkMax NEO 550, CAN 31)
  - `2026Robot/src/main/java/frc/robot/subsystems/HopperSubsystem.java` (TalonSRX bag motor, CAN 32, Phoenix5)
  - `2026Robot/src/main/java/frc/robot/subsystems/FeederSubsystem.java` (SparkMax NEO 550, CAN 33)
  - `2026Robot/src/main/java/frc/robot/subsystems/ShooterSubsystem.java` (4x TalonFX Krakens CAN 36-39 + 2x hood servos PWM 0-1)
  - `2026Robot/src/main/java/frc/robot/subsystems/TurretSubsystem.java` (2x SparkMax NEO 550 CAN 34-35, position PID, ±180° soft limits)
  - `2026Robot/src/main/java/frc/robot/subsystems/Superstructure.java` (multi-subsystem coordinator)
- Files modified:
  - `2026Robot/src/main/java/frc/robot/Constants.java` (full overhaul: new CAN IDs, all mechanism constants, ShootingPositions presets, MAX_SPEED 3.71)
  - `2026Robot/src/main/java/frc/robot/RobotContainer.java` (all subsystems wired, driver/operator bindings)
  - `2026Robot/src/main/java/frc/robot/util/DashboardManager.java` (updated to new constant names KP/KI/KD)
  - `2026Robot/src/main/deploy/swerve/modules/physicalproperties.json` (driveGearRatio 8.14, angleGearRatio 12.8 — MK4 L1 standard)
  - `2026Robot/src/main/deploy/swerve/modules/frontleft.json` (CAN IDs: drive 11, angle 15, encoder 19)
  - `2026Robot/src/main/deploy/swerve/modules/frontright.json` (CAN IDs: drive 12, angle 16, encoder 20)
  - `2026Robot/src/main/deploy/swerve/modules/backleft.json` (CAN IDs: drive 13, angle 17, encoder 21)
  - `2026Robot/src/main/deploy/swerve/modules/backright.json` (CAN IDs: drive 14, angle 18, encoder 22)
- Notes:
  - BUILD SUCCESSFUL (12 deprecation warnings, 0 errors)
  - All motor inversions default to false — VERIFY AND SET ON HARDWARE before first drive
  - Turret seeds at zero on boot — manually align to forward before each enable, then press operator Start to zero encoders
  - Joystick aiming math: angle = atan2(stickX, -stickY), magnitude = RPM scaling
  - Both turrets and shooters always fire together (no L/R split)
  - Tier 2 D-pad presets: all angles/RPMs are placeholders — tune on hardware
  - SparkMax configure() API deprecated (scheduled for future removal) but functional in 2026
- Also updated NOTES.md: new CAN bus table, corrected swerve gear ratios, updated mechanism summary, added Section 0B competition day setup guide (CAN ID flashing, inversion verification, SysId steps, turret zeroing, shooter tuning, controller quick reference)
- PENDING:
  - Set motor inversion constants after first hardware test (IntakeConstants, ConveyorConstants, HopperConstants, FeederConstants, TurretConstants, ShooterConstants)
  - Run SysId on swerve drive + angle motors before competition
  - Tune TurretConstants.KP, ShooterConstants.KP/KV on hardware
  - Tune ShootingPositions preset angles and RPMs for each D-pad position
  - Tune HoodConstants.NEAR_ANGLE / FAR_ANGLE for actual servo positions
  - Set physical CAN IDs on all hardware (swerve 11-22, mechanisms 30-39) via Phoenix Tuner X / REV Hardware Client
  - Vision: install PhotonVision on Ubuntu coprocessor, mount/calibrate cameras, set ENABLE_VISION=true

---

## 2026-02-15

### [2026-02-15 12:00] CLAUDE [IMPLEMENT]
- Implemented comprehensive Elastic Dashboard system with 5 phases:
  - Phase 1: WebServer utility (serves deploy dir on port 5800), TunableNumber and TunableBoolean utilities for bidirectional NT communication gated by TUNING_MODE
  - Phase 2: Expanded TelemetryPublisher with publishMechanismStatus() and publishSystemHealth(). Created DashboardManager centralizing all tunable PID/FF/RPM values, Sendable PID controllers, enable/disable flags, and system health publishing. Wired into Robot.java and RobotContainer.java
  - Phase 3: Created 6-tab elastic-layout.json (Competition, Tuning, Motor Config, SysId, Diagnostics, Raw Sensors) with properly placed widgets, tooltips, and grid layout
  - Phase 4: Added Elastic Layout Editing Standard to STANDARDS.md (widget types, NT paths, agent rules). Added dashboard workflow section to NOTES.md (deploying, loading, TunableNumber flow, shortcuts, troubleshooting)
  - Phase 5: Build verified successfully, JSON validated
- Files created:
  - `2026Robot/src/main/java/frc/robot/util/WebServer.java`
  - `2026Robot/src/main/java/frc/robot/util/TunableNumber.java`
  - `2026Robot/src/main/java/frc/robot/util/TunableBoolean.java`
  - `2026Robot/src/main/java/frc/robot/util/DashboardManager.java`
  - `2026Robot/src/main/deploy/elastic-layout.json`
- Files modified:
  - `2026Robot/src/main/java/frc/robot/Robot.java` (WebServer.start, DashboardManager.periodic)
  - `2026Robot/src/main/java/frc/robot/RobotContainer.java` (DashboardManager, drive/assist mode choosers)
  - `2026Robot/src/main/java/frc/robot/util/TelemetryPublisher.java` (mechanism + system health methods)
  - `STANDARDS.md` (Elastic layout editing standard for agents)
  - `NOTES.md` (Section 9: Elastic Dashboard Workflow)
- Notes: All tuning values gated by TelemetryConstants.TUNING_MODE for zero overhead in competition. WebServer uses Java built-in HttpServer. DashboardManager publishes Sendable PIDControllers for Elastic's dedicated PID widget.
- PENDING: Mechanism subsystems (TurretSubsystem, ShooterSubsystem) need implementation to wire actual motor controllers to DashboardManager tuned values

### [2026-02-15 00:00] CLAUDE [DOCS]
- Added comprehensive 2026 REBUILT game documentation to NOTES.md (Section 0)
- Updated README.md to reflect REBUILT game (was "[Game TBA]")
- Game details include: match structure, scoring breakdown, field elements, game pieces, robot constraints, design considerations, ranking points, and key resources
- Files modified:
  - `README.md` (updated title, overview, mechanisms, vision, development status)
  - `NOTES.md` (added Section 0: REBUILT game details, updated Section 3.3 vision field layout)
- Notes: All game data sourced from official FIRST resources, Chief Delphi, and community tools

### [2026-02-15 01:00] CLAUDE [CONFIG]
- Updated Vision.java AprilTag field layout from k2025ReefscapeAndyMark to k2026RebuiltAndymark
- Updated swerve drive gear ratios: drive 8.14 -> 5.8267, angle 12.8 -> 12
- Updated MAX_SPEED from 3.71 to 5.18 m/s across Constants.java and PathPlanner settings
- Added TurretConstants (CAN 20-21, NEO 550 / SparkMax, 68:1 ratio)
- Added expanded ShooterConstants (CAN 22-25, Kraken X44/X60, gear ratios, velocity targets)
- Added IntakeConstants placeholder (CAN 30-39 reserved)
- Updated NOTES.md CAN Bus Assignments with all new motor assignments and mechanism summary
- Files modified:
  - `2026Robot/src/main/java/frc/robot/subsystems/swervedrive/Vision.java`
  - `2026Robot/src/main/java/frc/robot/Constants.java`
  - `2026Robot/src/main/deploy/swerve/modules/physicalproperties.json`
  - `2026Robot/src/main/deploy/pathplanner/settings.json`
  - `NOTES.md`
- Notes: Drive ratio flagged as "seems high" by team, all values subject to change.
  NEO 550 turrets use SparkMax (REV). Krakens use TalonFX (Phoenix6, no Pro).
  PID and velocity targets are placeholders pending hardware testing.
- PENDING: Turret and shooter subsystem classes need implementation
- PENDING: Intake/indexer motor types and assignments TBD

---

## 2026-01-31

### [2026-01-31 12:30] CLAUDE [DOCS]
- Updated standards to specify Elastic Dashboard as default dashboard for 2026
- Files modified:
  - `STANDARDS.md` (added Dashboard Standards section after NetworkTables Naming Convention)
- Notes: Elastic Dashboard is WPILib's recommended dashboard with NT4 support and Shuffleboard widget compatibility
- Key points:
  - SmartDashboard *application* deprecated (removal 2027)
  - SmartDashboard *API class* remains valid for NetworkTables publishing
  - Shuffleboard widgets work in Elastic Dashboard
  - No code changes required - Elastic reads from NetworkTables directly

---

## 2026-01-26

### [2026-01-26 10:46] CLAUDE [DOCS]
- Moved CAN ID documentation from README.md to NOTES.md per documentation structure plan
- Files modified:
  - `README.md` (removed CAN ID table, added reference to NOTES.md)
  - `NOTES.md` (added CAN Bus Assignments section with full table)
- Notes: Aligns with documentation structure - README for overview, NOTES for operational setup details
- Content: Swerve Drive System (IDs 1-12) and Mechanisms (IDs 20-99 reserved)

---

## 2026-01-25

### [2026-01-25 14:00] CLAUDE [CONFIG]
- MAJOR: Harmonized FRC standards across repositories (FRC Standards Harmonization Plan)
- Established FRC-2026 STANDARDS.md as single source of truth for all common FRC standards
- Added 6 new sections to STANDARDS.md:
  - m_ prefix naming convention for member variables
  - Logging & Telemetry Standards
  - Vision Integration Standards
  - Autonomous Standards
  - Safety Standards (current limits, soft limits, timeouts, failsafe behavior)
  - Brake Management Pattern
- Updated all code examples to use m_ prefix consistently
- Files modified:
  - `STANDARDS.md` (major update with 6 new sections)
- Notes: All FRC repos (FRC-2024, FRC-2025, FRC-Test_Code) should now reference this file for common standards
- Benefits:
  - Single source of truth for common FRC patterns
  - Comprehensive safety standards documentation
  - Consistent naming conventions across all repos
  - Clear guidance on vision, autonomous, and logging patterns

---

## 2026-01-24

### [2026-01-24 15:28] CODEX [DOCS]
- Normalized README/NOTES formatting to ASCII-only characters (arrows, degrees, checkmarks)
- Repo: FRC-2026
- Files modified: README.md, NOTES.md
- Notes: Replaced unicode arrows/degree symbols with ASCII equivalents for portability

### [2026-01-24 14:41] CLAUDE [DOCS]
- Implemented documentation structure plan (README vs NOTES separation)
- Created README.md (was missing) with comprehensive project overview
- Created NOTES.md with complete migration of 477-line setup guide from Notes.txt
- Deleted Notes.txt after migration
- Files created:
  - `README.md` (project overview - 2026 season, swerve drive, vision system, modern architecture)
  - `NOTES.md` (complete setup guide - YAGSL config, PhotonVision setup, PathPlanner tuning, SysId, troubleshooting, competitive advantage roadmap)
- Files deleted:
  - `Notes.txt` (all content migrated to NOTES.md per user preference)
- Notes: Follows FRC documentation structure plan - stable project info in README, operational/setup notes in NOTES
- Content: Pre-season robot with swerve drive, vision fusion, PathPlanner autonomous, targeting pipeline strategy

### [2026-01-24 16:30] CLAUDE [CONFIG]
- Added project-level IDE redirect files to configuration hierarchy
- Created 3 project-level redirect files for 2026Robot: CLAUDE.md, GEMINI.md, AGENTS.md
- Updated hierarchy diagrams in root and repo-level configs to show project-level redirect layer
- Files created:
  - `2026Robot\CLAUDE.md` (lightweight redirect to root and repo configs)
  - `2026Robot\GEMINI.md` (lightweight redirect to root and repo configs)
  - `2026Robot\AGENTS.md` (lightweight redirect to root and repo configs)
- Files updated:
  - `CLAUDE.md` (added project redirect layer to Repository Structure diagram)
  - `GEMINI.md` (added project redirect layer to Repository Structure diagram)
  - `AGENTS.md` (added project redirect layer to Repository Structure diagram)
- Benefits:
  - IDE agents opened at project level now have clear entry point
  - Redirects keep project directories lightweight
  - Maintains single source of truth at repo level
  - No duplicate configuration content

### [2026-01-24 00:00] CLAUDE [CONFIG]
- MAJOR: Restructured agent configuration system to repo-level logging
- Moved all changelog/handoff tracking from root to repo-level `.agent-log\` directories
- Updated agent system to reflect 6 agents: Claude (CLI+IDE), Gemini (CLI+IDE), Codex (CLI+IDE)
- Deleted all `.cursorrules` files (not using Cursor)
- Eliminated project-level agent configs (IDE agents can reach repo root)
- Created root `AGENTS.md` as the Codex coordination protocol
- Files created:
  - `.agent-log\changelog.md` (this file - merged from root and project-level)
  - `.agent-log\handoffs.md` (repo-level task handoffs)
- Files updated:
  - `CLAUDE.md` (updated hierarchy, removed project-level references, removed .cursorrules)
  - `GEMINI.md` (updated hierarchy, removed project-level references, removed .cursorrules)
  - `AGENTS.md` (restructured from architecture docs to Codex behavior config)
- Files deleted:
  - `.cursorrules` (not using Cursor)
  - `2026Robot\CLAUDE.md` (project-level configs eliminated)
  - `2026Robot\GEMINI.md` (project-level configs eliminated)
  - `2026Robot\.cursorrules` (not using Cursor)
  - `2026Robot\.agent-log\` (merged to repo level)
- New structure:
  - Root: CLAUDE.md, GEMINI.md, AGENTS.md (coordination protocols only)
  - Repo: STANDARDS.md, CLAUDE.md, GEMINI.md, AGENTS.md, .agent-log\
  - No project-level configs (eliminated)
- Benefits:
  - Simpler hierarchy (2 levels instead of 3)
  - Repo-level logging for better organization
  - 6 agents supported (CLI and IDE variants)
  - No duplication between CLI and IDE configs

---

## 2026-01-23

### [2026-01-23 17:30] CLAUDE [CONFIG]
- MAJOR: Restructured entire multi-agent configuration system
- Implemented three-level hierarchy (root/repo/project)
- Created STANDARDS.md as single source of truth for FRC-2026
- Files created:
  - `STANDARDS.md` (single source of truth for 2026 standards)
  - `CLAUDE.md` (repo-level Claude config)
  - `GEMINI.md` (repo-level Gemini config)
  - `.cursorrules` (repo-level Cursor config)
  - `2026Robot\CLAUDE.md` (project-specific with included standards)
  - `2026Robot\GEMINI.md` (project-specific with included standards)
  - `2026Robot\.cursorrules` (project-specific with included standards)
  - `AGENTS.md` (comprehensive architecture guide)
- Architecture:
  - Level 1 (Root): Cross-agent protocol ONLY (no repo-specific content)
  - Level 2 (Repo): STANDARDS.md + agent behavior configs
  - Level 3 (Project): Self-contained configs (include repo standards)
- Benefits:
  - No duplication between agent configs (each reads only their own file)
  - Root is generic (applies to all repos, not just FRC)
  - IDE agents have self-contained configs (guaranteed access)
  - Single source of truth (STANDARDS.md) for all technical rules
- IMPORTANT: All agents must now read STANDARDS.md for technical rules
- BREAKING: Old config hierarchy replaced - re-read your agent's config file

### [2026-01-23 14:30] CLAUDE [CONFIG]
- Set up project-level agent coordination system (2026Robot)
- Files created:
  - `2026Robot\CLAUDE.md` (project-specific context)
  - `2026Robot\GEMINI.md` (project-specific patterns)
  - `2026Robot\.cursorrules` (Cursor configuration)
  - `2026Robot\.agent-log\changelog.md` (project activity log)
- NOTE: All agents working in 2026Robot should log changes to project-level changelog

### [2026-01-23 15:45] CLAUDE [CONFIG]
- Updated 2026Robot\GEMINI.md to comprehensive self-contained configuration
- Files modified:
  - `2026Robot\GEMINI.md` (complete rewrite with all FRC-2026 standards)
- Notes: Modeled after CLAUDE.md structure but with Gemini-specific IDE workflow
- Contains: Cross-agent protocol, full 2026 motor APIs, project CAN IDs, robot constants, IDE integration tips
- Now self-contained - Gemini IDE has everything needed without reading parent folders

### [2026-01-23 14:00] CLAUDE [CONFIG]
- Initial multi-agent workflow system setup
- Created baseline configs and changelog structure
- NOTE: Superseded by 17:30 restructuring above

---

## Log Entry Template

```markdown
### [YYYY-MM-DD HH:MM] AGENT_NAME [ACTION_TYPE]
- Brief description
- Files: <paths relative to FRC-2026/>
- Notes: Important context
- PENDING: (optional) What needs follow-up
```

### Example Entries

```markdown
### [2026-01-24 10:15] CODEX [IMPLEMENT]
- Created IntakeSubsystem with command factories
- Files created:
  - `2026Robot/src/main/java/frc/robot/subsystems/IntakeSubsystem.java`
  - `2026Robot/src/test/java/frc/robot/subsystems/IntakeSubsystemTest.java`
- Files modified:
  - `2026Robot/src/main/java/frc/robot/RobotContainer.java` (added subsystem instantiation)
  - `2026Robot/src/main/java/frc/robot/Constants.java` (added IntakeConstants)
- Notes: Using SparkMax with 2026 API, CAN ID 20
- PENDING: Needs PID tuning on real hardware

### [2026-01-24 14:30] GEMINI [TEST]
- Added unit tests for DriveSubsystem
- Files created:
  - `2026Robot/src/test/java/frc/robot/subsystems/DriveSubsystemTest.java`
- Notes: All tests passing with `./gradlew test`

### [2026-01-24 16:00] CLAUDE [FIX]
- Fixed encoder conversion factor in swerve configuration
- Files modified:
  - `2026Robot/src/main/deploy/swerve/swervedrive.json`
- Notes: Drive distance now reports correctly in meters
- Affected: Autonomous odometry, driver display
```
### [2026-01-25 07:02] CODEX [DOCS]
- Clarified Phoenix Pro is not used and documented Phoenix6 non-Pro API expectations
- Files: STANDARDS.md, README.md
- Notes: Added explicit guidance to avoid Pro-only control modes

### [2026-01-25 20:30] GEMINI [FIX]
- Added Alert.java utility
- Repo: FRC-2026
- Files modified: src/main/java/frc/robot/utils/Alert.java
- Notes: Imported persistent Alert class for dashboard feedback

### [2026-03-19 16:38] CODEX [REVIEW]
- Performed static competition-readiness review of 2026 robot code without modifying robot code or configs
- Repo: FRC-2026
- Files modified: .agent-log/changelog.md
- Notes: Findings focused on match-risk issues: no autonomous routines present, operator control mismatches in RobotContainer, untuned live shooting presets, no feed gating on shooter/turret readiness, dashboard controls that do not affect behavior, and vision path not competition-ready while disabled

### [2026-03-20 10:25] CODEX [CONFIG]
- Added approximate front-corner PhotonVision camera transforms for current competition mounting plan
- Repo: FRC-2026
- Files modified: 2026Robot/src/main/java/frc/robot/Constants.java, 2026Robot/src/main/java/frc/robot/subsystems/swervedrive/Vision.java
- Notes: Vision remains globally disabled until cameras are mounted/calibrated. Right and left camera names are prewired as `right_cam` and `left_cam`; missing cameras simply produce no results until present in PhotonVision.
- PENDING: Verify left camera yaw sign on hardware, re-measure final transforms after mounting, then set ENABLE_VISION=true and validate pose estimation on the field

### [2026-03-20 10:31] CODEX [CONFIG]
- Corrected left camera yaw sign to mirror the right camera based on updated mounting description
- Repo: FRC-2026
- Files modified: 2026Robot/src/main/java/frc/robot/Constants.java
- Notes: Front-right camera yaw remains `-45 deg`; front-left is now `+45 deg` because both cameras look outward along the robot-center-to-camera line

### [2026-03-20 11:04] CODEX [DOCS]
- Documented the Beelink/PhotonVision competition setup and updated the vision TODO sequence to match the current staged bring-up plan
- Repo: FRC-2026
- Files modified: NOTES.md, README.md, 2026Robot/TODO-COMPETITION.md
- Notes: Added the verified machine state (PhotonVision on boot, SSH on boot, headless sleep disabled, robot-side static IP, GRUB timeout change, BIOS power-loss note) and updated docs to reflect right-camera-first deployment

### [2026-03-20 11:06] CODEX [DOCS]
- Updated Beelink notes after final reboot verification and BIOS power-on confirmation
- Repo: FRC-2026
- Files modified: NOTES.md, .agent-log/changelog.md
- Notes: Verified SSH and PhotonVision auto-start after reboot; BIOS power-loss behavior is now documented as configured by the team
### [2026-03-20 13:56] CODEX [CONFIG]
- Verified PhotonVision networking against the official docs and corrected the saved settings on the Beelink
- Repo: FRC-2026
- Files modified: NOTES.md, 2026Robot/TODO-COMPETITION.md, .agent-log/changelog.md
- Notes: PhotonVision is now confirmed in NT client mode with team `3843`, managed static robot Ethernet `10.38.43.11`, and HTTP/SSH still reachable after service restart. Hotspot Wi-Fi remains usable but temporary.
