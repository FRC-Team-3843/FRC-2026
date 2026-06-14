# FRC-2026 — Entity Index (per-scope)

Per-scope entity surface for the FRC-2026 repo. Same 5-column shape as the personal index; only `status: active` entities get a row (§3.2 guard). Second-resolution detail lives in `.notes\`, never in these rows. Cross-cutting entities (team, build env, coprocessor host) are owned by `PersonalContext` and proposed for its index — not duplicated here.

| entity | aliases | scope | notes/sources | status |
|---|---|---|---|---|
| frc-2026 | 2026 robot, rebuilt, 2026Robot | FRC-2026 | `FRC-2026\.notes\frc-2026-repo-source-202606120630.md`; robot code in `2026Robot\` | active |
| yagsl | yet another generic swerve library, yagsl-2026.1.20 | FRC-2026 | swerve library driving SwerveSubsystem; JSON config in `deploy/swerve/` `.notes\yagsl.md` | active |
| photonvision | photonlib, photon vision | FRC-2026 | vision library / AprilTag pose fusion on RIO `.notes\photonvision.md` | active |
| photonvision-coprocessor | vision coprocessor, beelink mini s, vision pi | FRC-2026 | Beelink Mini S on robot at `10.38.43.11:5800`; NOT the home-lab beelink `.notes\photonvision-coprocessor.md` | active |
| pathplanner | pathplannerlib, pathplannerlib-2026.1.2 | FRC-2026 | autonomous path library; settings + NamedCommands in `deploy/pathplanner/` `.notes\pathplanner.md` | active |
| phoenix6-ctre | phoenix6, phoenix 6, ctre phoenix6, phoenix6-replay-26.1.0 | FRC-2026 | TalonFX/CANcoder library; non-Pro control only per team standard `.notes\phoenix6-ctre.md` | active |
| phoenix5-ctre | phoenix5, phoenix 5, ctre phoenix5, phoenix5-replay-5.36.0 | FRC-2026 | legacy TalonSRX library; used only for hopper Bag motor (CAN 32) `.notes\phoenix5-ctre.md` | active |
| revlib-sparkmax | revlib, rev lib, sparkmax, spark max, neo, neo550, rev robotics | FRC-2026 | REV library for NEO/NEO550 SparkMax controllers; SparkMax/SparkMaxConfig API only `.notes\revlib-sparkmax.md` | active |
| roborio | rio, roborio-3843-frc.local | FRC-2026 | FRC robot controller; deploy target at `10.38.43.2` `.notes\roborio.md` | active |
| navx-gyro | navx, navx imu, studica | FRC-2026 | NavX IMU on RoboRIO SPI; alliance-aware zeroing required before field-relative `.notes\navx-gyro.md` | active |
| sds-mk4-swerve-modules | mk4, sds mk4, swerve modules, mk4 l1, swerve drive specialties | FRC-2026 | SDS MK4 L1 stock modules (8.14:1 drive, 12.8:1 angle, 4" wheel) `.notes\sds-mk4-swerve-modules.md` | active |
| phoenix-tuner-x | phoenix tuner x, tuner x, phoenix tuner | FRC-2026 | CTRE desktop config/flash tool; competition-setup Step 1 `.notes\phoenix-tuner-x.md` | active |
| rev-hardware-client | rev hardware client, rev client, sparkmax config tool | FRC-2026 | REV desktop config/flash tool for SparkMax; paired with Phoenix Tuner X in pit `.notes\rev-hardware-client.md` | active |
| elastic-dashboard | elastic, elastic-layout | FRC-2026 | driver dashboard; live-tuning surface via NT `/Tuning/`; "Save Hood" persists `hood-config.json` `.notes\elastic-dashboard.md` | active |
| advantagescope | advantage scope, wpilog viewer | FRC-2026 | telemetry/SysId log viewer for `.wpilog` files `.notes\advantagescope.md` | active |
| wpilib-gradlerio | gradlerio, gradle rio, wpilib build | FRC-2026 | Gradle plugin for build/deploy/test/simulate; run from `2026Robot/` dir `.notes\wpilib-gradlerio.md` | active |
| frc-2026-rebuilt | rebuilt, 2026 game, rebuilt game, hub shifting, frc rebuilt, frc 2026 rebuilt | FRC-2026 | 2026 FRC game (Haas); Hub Shifting mechanic; alliance from `getGameSpecificMessage()` `.notes\frc-2026-rebuilt.md` | active |
| choreolib | choreo, choreolib2026 | FRC-2026 | trajectory vendordep present alongside PathPlanner; active use unconfirmed `.notes\choreolib.md` | active |
| frc-2026-can-bus-map | can map, can bus map, can id table, device map | FRC-2026 | full CAN device table (CAN 11–39 + off-bus); authoritative cross-ref `.notes\frc-2026-can-bus-map.md` | active |
| phoenix5-6-coexistence | phoenix5-6 coexistence, dual phoenix api, phoenix import gotcha | FRC-2026 | gotcha: both CTRE APIs in same project; wrong namespace import breaks build `.notes\phoenix5-6-coexistence.md` | active |
| hood-config-json | hood-config.json, hood config, hood angles | FRC-2026 | hood angles in `deploy/hood-config.json` (not Constants); "Save Hood" workflow `.notes\hood-config-json.md` | active |
| sparkmax-loop-time | loop_time, loop time, sparkmax delay, odometry loop time | FRC-2026 | `LOOP_TIME = 0.13` folds SparkMax ~110ms CAN delay; do not lower naively `.notes\sparkmax-loop-time.md` | active |
| superstructure-pattern | superstructure, feed chain coordinator | FRC-2026 | Superstructure.java is command-factory coordinator; bypass causes requirement conflicts `.notes\superstructure-pattern.md` | active |
