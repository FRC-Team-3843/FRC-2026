# Competition TODO — Team 3843

Status: At competition, driving and shooting operational.
Last updated: 2026-03-23 (day 3 Smokey Mountain Regional)

---

## COMPLETED

- [x] Fix intake CAN ID (set to 30)
- [x] Verify swerve CAN IDs (11-22)
- [x] Set CANCoder offsets (FL=329.77, FR=359.74, BL=43.15, BR=77.70)
- [x] Intake motor inverted (MOTOR_INVERTED = true)
- [x] Feeder motor inverted (MOTOR_INVERTED = true)
- [x] SysId swerve angle motors — angle kP=0.0106
- [x] Drive kP=0.005, kFF=0.27 (estimated, no SysId room)
- [x] SysId turret motors — per-motor PID (left kP=0.15, right kP=0.10)
- [x] Field-centric turret aiming enabled
- [x] Both turrets locked to left stick
- [x] Both shooters follow left stick magnitude
- [x] Turret gear ratio corrected to 52.987:1 (11t>68t, 14t>120t)
- [x] Turret soft limits set to +/-110 deg (16.2 motor rotations)
- [x] Main shooter gear ratio corrected to 66/72 = 0.917
- [x] MAX_WHEEL_RPM set to 5500
- [x] LiveWindow disabled in test mode (fixes CAN stale)
- [x] Strafe axis fixed (removed negation)
- [x] Driver intake split: RB = intake + conveyor only
- [x] Operator feed split: A = conveyor + hopper + feeder, B = reverse
- [x] Elastic dashboard layout updated

## STILL NEEDED — Between Matches

- [x] **Verify remaining motor inversions** — all verified good on hardware (day 3)
- [x] **FR angle motor** — friction was transient during tuning, operating normally (day 3)
- [ ] **Fix main shooter CAN 38** — firmware error on TalonFX 38, needs Phoenix6 firmware flash
- [ ] **SysId swerve drive** — need space for a run, update pidfproperties.json
- [x] **Tune shooter presets** — D-pad RPMs tuned for actual hub shots (day 3)
- [x] **Tune hood servo angles** — servo angles dialed in (day 3)
- [x] **Register PathPlanner named commands** — autoShoot and stopShooter registered
- [x] **Create autonomous routines** — "Just Shoot Auto" and "Main Auto" configured with auto chooser (day 3)
- [ ] **SysId shooter & preshooter** — need characterization runs for velocity PID tuning

## STRETCH — Vision

- [x] Install PhotonVision on Ubuntu coprocessor
- [x] Configure Beelink for headless use (SSH + PhotonVision on boot, sleep disabled)
- [x] Verify PhotonVision networking settings (team `3843`, NT server off, robot Ethernet static `10.38.43.11`)
- [x] Prewire `right_cam` / `left_cam` names and approximate transforms in code
- [ ] Mount and wire right global shutter camera
- [ ] Create `right_cam` in PhotonVision and verify live detections
- [ ] Calibrate right camera (checkerboard pattern, ~15 min)
- [ ] Verify right camera transform on hardware (current values are estimates)
- [ ] Set VisionConstants.ENABLE_VISION = true
- [ ] Test and verify single-camera pose estimation
- [ ] Mount and wire left camera later
- [ ] Create/calibrate `left_cam` and re-verify mirrored transforms

---

## CAN ID Reference

### Swerve (REV Hardware Client + Phoenix Tuner X)
| Device | CAN ID |
|--------|--------|
| FL Drive | 11 |
| FR Drive | 12 |
| BL Drive | 13 |
| BR Drive | 14 |
| FL Angle | 15 |
| FR Angle | 16 |
| BL Angle | 17 |
| BR Angle | 18 |
| FL CANCoder | 19 |
| FR CANCoder | 20 |
| BL CANCoder | 21 |
| BR CANCoder | 22 |

### Mechanisms (Phoenix Tuner X for TalonFX/SRX, REV Hardware Client for SparkMax)
| Device | CAN ID | Controller |
|--------|--------|------------|
| Intake | 30 | TalonFX (inverted) |
| Conveyor | 31 | SparkMax |
| Hopper | 32 | TalonSRX |
| Feeder | 33 | SparkMax (inverted) |
| L Turret | 34 | SparkMax |
| R Turret | 35 | SparkMax |
| L Preshooter | 36 | TalonFX |
| R Preshooter | 37 | TalonFX |
| L Main Shooter | 38 | TalonFX |
| R Main Shooter | 39 | TalonFX |

### PWM
| Device | Channel |
|--------|---------|
| L Hood Servo | 0 |
| R Hood Servo | 1 |

---

## Tuned Values (current in code)

| Constant | Value | Source |
|----------|-------|--------|
| Swerve drive kP | 0.005 | Estimated (no SysId) |
| Swerve drive kFF | 0.27 | Calculated (1/3.71) |
| Swerve angle kP | 0.0106 | SysId avg of BL/BR/FL |
| Turret left kP/kD | 0.15 / 0.01 | SysId analysis |
| Turret left output limit | 70% | High friction compensation |
| Turret right kP/kD | 0.10 / 0.008 | SysId analysis |
| Turret right output limit | 50% | Normal |
| Turret gear ratio | 52.987:1 | 11t>68t, 14t>120t |
| Turret soft limits | +/-110 deg | 16.2 motor rotations |
| Main shooter gear ratio | 0.917 | 66t:72t overdrive |
| Preshooter gear ratio | 0.625 | 5:8 overdrive |
| MAX_WHEEL_RPM | 5500 | Achievable at low battery |
| Shooter kP/kV | 0.1 / 0.12 | Placeholder — needs tuning |
| IntakeConstants.MOTOR_INVERTED | true | Verified on hardware |
| FeederConstants.MOTOR_INVERTED | true | Verified on hardware |

---

## Controller Reference

### DRIVER (port 0)
| Control | Action |
|---------|--------|
| Left stick | Translate (field-relative) |
| Right stick X | Rotate |
| Left trigger | Progressive slow mode (80% → 20%) |
| Right trigger | Intake + conveyor (progressive 0% → 100%) |
| Right bumper (hold) | Intake + conveyor (full speed) |
| Left bumper (hold) | Reverse intake + floor conveyor |
| Start | Zero gyro (face downfield first!) |
| POV Up | X-lock wheels |
| POV Down | Center modules |

### OPERATOR (port 1)
| Control | Action |
|---------|--------|
| Left stick direction | Aim both turrets (field-centric) |
| Left stick magnitude | Spin up both shooters |
| A (hold) | Feed chain (conveyor + hopper + feeder) |
| B (hold) | Reverse feed chain |
| D-pad Up | CLOSE preset (2500 RPM, hood near) |
| D-pad Down | FAR preset (4000 RPM, hood far) |
| D-pad Left | ANGLE_LEFT (5000 RPM, +45°) |
| D-pad Right | ANGLE_RIGHT (5000 RPM, -45°) |
| Y | Cancel preset, back to manual |
| Left bumper | Hood near |
| Right bumper | Hood far |
| Start | Zero turret encoders |

### TEST MODE (driver)
| Control | Action |
|---------|--------|
| X | SysId swerve drive (~11s) |
| Y | SysId swerve angle (~11s) |
| A | SysId main shooter (~20s) |
| B | SysId preshooter (~20s) |
| LB | SysId turret (~22s) |
