---
id: can-id-docs-migration-2025-mechanisms
model: claude-sonnet-4-6
model_basis: confirmed
original_session_model: unattributed
original_session_model_basis: unattributed
title: "recovered: CAN ID docs migration to NOTES + 2025 robot mechanism descriptions"
schema_version: 2
created: 2026-06-21T00:00:00Z
updated: 2026-06-21T00:00:00Z
valid_until: null
author: claude
session: recovered-7babc4c3
original_session_date: 2026-01-26
tags: [recovered, reconstructed, frc, documentation, can, frc-2025]
aliases: [recovered-2026-01-26-7babc4c3]
related: []
status: active
supersedes: null
confidence: 25
source_basis: transcript
human_edited: false
sensitivity: normal
decisions: []
artifact_kind: memory
memory_class: episodic
semantic_kind: state
scope: FRC-2026
load_profile: on_demand
---

# recovered: CAN ID docs migration to NOTES + 2025 robot mechanism descriptions

> ⚠ RECOVERED/RECONSTRUCTED — NOT a verbatim transcript. The assistant side of this session
> was permanently deleted; only the user's prompts + project artifacts survive. Ground truth =
> the verbatim user intent + the artifact-cited (COMMIT/CHANGELOG/NOTE/PLAN) facts below.
> A claim in a faithful section that lacks an artifact citation is NOT ground truth — treat it
> as prompt-derived (user intent) or narrative, never as a confirmed outcome. Inferred items are
> labeled and must NOT be distilled as fact. Reconstructing model: claude-sonnet-4-6 (confirmed);
> original session model: unattributed. See recovered-transcripts/CALIBRATION.md.

## From the user's prompts (ground truth — intent + user-stated facts)

- User passed a plan to move CAN ID tables from README.md to NOTES.md across FRC-2024, FRC-2025, FRC-2026. FRC-Test_Code excluded (uses runtime CAN IDs). STANDARDS.md base CAN ranges to remain unchanged.
- User directed that FRC-2025 encoder positions move from README to NOTES, and the Auto section show only robot capability types (what it can do), not path technical details.
- User directed removal of the Subsystems section from FRC-2025 README as redundant with Mechanisms and Drivetrain sections.
- User provided definitive FRC-2025 mechanism descriptions (ground truth, user-stated):
  - **Lifter**: triple-purpose — (1) algae intake (with claw arm, elevator, and claw intake to pull algae for barge scoring); (2) coral intake to bottom reef tray; (3) cage climb hook; servo-driven parking brakes to prevent backdrive/lowering after power-off.
  - **Claw Intake**: algae intake (limited coral capability, not used in competition).
  - **Intake/Scorer**: bidirectional — one direction pulls algae in (from lifter or directly off reef); opposite direction pushes out to score in barge; lifter could also hold algae for low-goal scoring, and claw could pass ball back to lifter for low-goal after getting it off the reef.
  - **Claw Elevator**: additional height needed for barge scoring.
  - **Swerve Drive**: drivetrain.
- Documentation structure decision: README = overview; NOTES = operational setup details (CAN IDs, positions, etc.); STANDARDS = coding conventions/base ranges.

## Artifact-cited outcomes (COMMIT / CHANGELOG / NOTE / PLAN)

- None found. No commits on 2026-01-26 in FRC-2024, FRC-2025, or FRC-2026 directly confirm the CAN ID migration. The commits closest to this date (FRC-2026: `ded5dfd` 2026-01-26 "Fix: Add Alerts, update STANDARDS to forbid suppression"; FRC-Test_Code: `e6c85b8` 2026-01-26 "Fix: 2026 Motor API, standardize Motor_Test, Alerts") are not topically consistent with the CAN ID documentation migration.
- CAN IDs for FRC-2026 in the plan (1-4 drive, 5-8 steer, 9-12 CANcoder) are plan-era values; see [[frc-2026-can-bus-map]] for the final production CAN map.

## Inferred (low-confidence — do not distill as fact)

- INFERRED (DOMAIN): CAN ID tables were moved from README.md to NOTES.md in FRC-2024, FRC-2025, FRC-2026, and FRC-2025 mechanism descriptions were updated per user's stated descriptions. Whether these changes were committed during this session or later is unknown.

## Likely missing

Commit evidence for the CAN ID migration, whether the 2025/2024 NOTES.md changes landed in this session or a subsequent one, and the actual updated NOTES.md content are unrecoverable.
