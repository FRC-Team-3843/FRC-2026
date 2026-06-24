---
id: game-docs-hardware-constants-elastic-dashboard
model: claude-sonnet-4-6
model_basis: confirmed
original_session_model: unattributed
original_session_model_basis: unattributed
title: "recovered: 2026 game docs + hardware constants + Elastic Dashboard system"
schema_version: 2
created: 2026-06-21T00:00:00Z
updated: 2026-06-21T00:00:00Z
valid_until: null
author: claude
session: recovered-eb68377f
original_session_date: 2026-02-15
tags: [recovered, reconstructed, frc, game-docs, hardware-config, elastic-dashboard, swerve]
aliases: [recovered-2026-02-15-eb68377f]
related: []
status: active
supersedes: null
confidence: 52
source_basis: transcript
human_edited: false
sensitivity: normal
decisions: []
artifact_kind: memory
memory_class: episodic
semantic_kind: state
scope: FRC-2026
---

# recovered: 2026 game docs + hardware constants + Elastic Dashboard system

> ⚠ RECOVERED/RECONSTRUCTED — NOT a verbatim transcript. The assistant side of this session
> was permanently deleted; only the user's prompts + project artifacts survive. Ground truth =
> the verbatim user intent + the artifact-cited (COMMIT/CHANGELOG/NOTE/PLAN) facts below.
> A claim in a faithful section that lacks an artifact citation is NOT ground truth — treat it
> as prompt-derived (user intent) or narrative, never as a confirmed outcome. Inferred items are
> labeled and must NOT be distilled as fact. Reconstructing model: claude-sonnet-4-6 (confirmed);
> original session model: unattributed. See recovered-transcripts/CALIBRATION.md.

## From the user's prompts (ground truth — intent + user-stated facts)

- User reviewed the state of the 2026 code and asked for research on the 2026 FIRST game (REBUILT) to be added to documentation.
- User stated robot design facts (ground truth at this date — later revised; see [[frc-2026-shooter-swerve-hardware]]):
  - Double turret shooter: two turrets, each NEO 550 powered.
  - Preshooter: Kraken X44. Main shooter: Kraken X60.
  - Swerve: SDS MK4 with custom inversion plate; user stated drive ratio 5.82666 (repeating); steer 12:1. User explicitly flagged both values as uncertain ("drive ratio seems high, we can change it").
  - Turret ratio: 68:1. Preshooter ratio: 0.625:1. Main shooter ratio: 0.8333 repeating (60/72). All "subject to change."
  - Intake/indexing motors: not yet determined.
- PROVENANCE CAUTION: The 5.82666 drive ratio belongs to a planned custom inversion plate that was never installed. The production swerve is SDS MK4 L1, 8.14:1 drive / 12.8:1 steer, stock orientation. See [[frc-2026-shooter-swerve-hardware]] for resolution.
- User asked to document all hardware and push values to JSON config files.
- User explicitly asked to add a note that the WPI JDK (at `C:\Users\Public\wpilib` — user-stated) must be used to compile. See [[frc-2026-build-env-java17]] (already curated).
- User raised an Elastic Dashboard problem: layout not loading from robot in prior sessions; believed the issue was incorrect file placement/upload. Asked to verify the workflow for uploading to the robot and pulling from the robot.

## Artifact-cited outcomes (COMMIT / CHANGELOG / NOTE / PLAN)

- CHANGELOG `[2026-02-15 00:00] CLAUDE [DOCS]` (`FRC-2026\.changelog.md`): Added comprehensive 2026 REBUILT game documentation to NOTES.md Section 0 (match structure, scoring, field elements, game pieces, robot constraints, design considerations, ranking points, key resources); updated README.md from "[Game TBA]" to REBUILT; updated NOTES.md Section 3.3 (vision field layout). Files: `README.md`, `NOTES.md`.
- CHANGELOG `[2026-02-15 01:00] CLAUDE [CONFIG]` (`FRC-2026\.changelog.md`): Updated Vision.java AprilTag layout to `k2026RebuiltAndymark`; swerve drive ratio updated 8.14→5.8267, angle 12.8→12; MAX_SPEED updated 3.71→5.18 m/s; TurretConstants added (CAN 20-21, NEO 550/SparkMax, 68:1); ShooterConstants added (CAN 22-25, Kraken X44/X60, gear ratios + velocity targets); IntakeConstants placeholder (CAN 30-39 reserved); NOTES.md CAN Bus Assignments updated. Files: `Vision.java`, `Constants.java`, `physicalproperties.json`, PathPlanner `settings.json`, `NOTES.md`. Note: drive ratio flagged as uncertain; all values "subject to change."
- CHANGELOG `[2026-02-15 12:00] CLAUDE [IMPLEMENT]` (`FRC-2026\.changelog.md`): Implemented 5-phase Elastic Dashboard system: WebServer utility serving deploy directory on port 5800 (resolves load-from-robot problem); TunableNumber/TunableBoolean utilities for bidirectional NT; DashboardManager; 6-tab `elastic-layout.json` (Competition, Tuning, Motor Config, SysId, Diagnostics, Raw Sensors); Elastic Layout Editing Standard added to STANDARDS.md; dashboard workflow added to NOTES.md. Build verified, JSON validated. Files created: `WebServer.java`, `TunableNumber.java`, `TunableBoolean.java`, `DashboardManager.java`, `elastic-layout.json`. PENDING at time: TurretSubsystem, ShooterSubsystem still needed. See [[elastic-dashboard]] for additional curated context.
- CAN IDs in 01:00 changelog entry (20-21 turrets, 22-25 shooters, 30-39 intake) are partially superseded by the final production CAN map — see [[frc-2026-can-bus-map]] for the resolved final map.

## Inferred (low-confidence — do not distill as fact)

- INFERRED (DOMAIN): The Elastic Dashboard "can't load from robot" problem was resolved by a WebServer serving the deploy directory on port 5800, enabling the "Download from Robot" workflow in the Elastic client.
- INFERRED (DOMAIN): WPI Java build note was likely added to NOTES.md or STANDARDS.md but the exact location is not in the changelog.

## Likely missing

Exact WPI Java documentation location, the rate-limit/exit/resume sequence outcome (prompts 6-8), and any code review steps run on the hardware constants changes are unrecoverable.
