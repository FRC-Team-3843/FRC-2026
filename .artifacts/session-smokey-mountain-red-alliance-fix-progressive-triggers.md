---
id: smokey-mountain-red-alliance-fix-progressive-triggers
model: claude-sonnet-4-6
model_basis: confirmed
original_session_model: unattributed
original_session_model_basis: unattributed
title: "recovered: Smokey Mountain Day 3 — red alliance fix, progressive triggers, final commit"
schema_version: 2
created: 2026-06-21T00:00:00Z
updated: 2026-06-21T00:00:00Z
valid_until: null
author: claude
session: recovered-ef7e1f2d
original_session_date: 2026-03-21
tags: [recovered, reconstructed, frc, competition]
aliases: [recovered-2026-03-21-ef7e1f2d]
related: [smokey-mountain-mechanism-swerve-sysid-tuning, smokey-mountain-shootercalculator-hoodconfig-alliance-bug]
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

# recovered: Smokey Mountain Day 3 — red alliance fix, progressive triggers, final commit

> ⚠ RECOVERED/RECONSTRUCTED — NOT a verbatim transcript. The assistant side of this session
> was permanently deleted; only the user's prompts + project artifacts survive. Ground truth =
> the verbatim user intent + the artifact-cited (COMMIT/CHANGELOG/NOTE/PLAN) facts below.
> A claim in a faithful section that lacks an artifact citation is NOT ground truth — treat it
> as prompt-derived (user intent) or narrative, never as a confirmed outcome. Inferred items are
> labeled and must NOT be distilled as fact. Reconstructing model: claude-sonnet-4-6 (confirmed);
> original session model: unattributed. See recovered-transcripts/CALIBRATION.md.

## From the user's prompts (ground truth — intent + user-stated facts)

**Session context (prompt [1]):** Third day of Smokey Mountain Regional; continues directly from session 0a6281e5. Short session — 17 prompts.

**Status check (prompts [2-6]):**
- Motor inversions confirmed good; FR angle motor issue resolved (hardware during tuning, not code)
- Shooter presets and servo angles confirmed done
- PathPlanner named commands: user thought done — prompts check if still needed
- Two auto routines required; user confirmed they need to add shooter/preshooter SysId back to the list

**Control layout changes (prompt [7]):**
- Center modules moved to POV down
- Slow mode changed to progressive scaling: 20-80% between trigger pull range (binary → progressive)
- Right trigger: turns on intake roller + floor conveyor, percent output scaled 0% (no pull) → 100% (full pull)
- Right bumper 100% (binary full-on intake/conveyor) kept in addition to progressive trigger (prompt [9])

**Red alliance bug fix (prompts [11-12]):**
- User described symptom: on red alliance, left is right, right is left, back is forward, forward is back
- User decision (prompt [12]): "lets remove allianceRelativeControl also lets note all the changes and this issue as something to remember for next year"

**Push flow (prompts [13-16]):** Two `/login` attempts after prompt [13] before successful push (authentication issue during session).

## Artifact-cited outcomes (COMMIT / CHANGELOG / NOTE / PLAN)

**`1791bac`** (2026-03-23, FRC-2026) — "Day 3 Smokey Mountain: fix red alliance controls, progressive triggers, control layout"
- Direct match for this session's core changes: red alliance fix (prompts [11-12]), progressive triggers (prompt [7]), control layout (prompts [7-10])
- Fix: disabled `allianceRelativeControl` — was double-flipping with `zeroGyroWithAlliance()` causing inverted X/Y on red alliance; center modules moved from Back to POV Down; slow mode → progressive scaling; added right trigger progressive intake with `setSpeed()` added to `IntakeSubsystem`

**[2026-03-21 12:00] CLAUDE [FIX]** (`FRC-2026\.changelog.md`) — "Competition day 3 driver control changes"
- Verbatim lesson recorded: "allianceRelativeControl(true) + zeroGyroWithAlliance() = double-flip on red alliance. Only use ONE alliance compensation method."
- Note: timestamp [12:00] is after session prompts span ends at 09:12 Mar 21; the fix was committed after `/login` auth resolved in prompts [14-16]; the diagnosis was done in session 0a6281e5 prompts [80-104].

**`92e663d`** (2026-03-21 10:02, FRC-2026) — "Competition tuning: drive inversion, duty cycle trigger, heading lock, RPM presets"
- Prior-session commit; provides codebase state at Day 3 session start (referenced in reconstructed note [1]).

**Workspace memory:** `feedback_frc_alliance_control.md` — "never combine allianceRelativeControl(true) with alliance-aware gyro zero" — ratified lesson derived from this session's prompts [11-12] and commit 1791bac. Cross-reference [[frc-2026-shooter-swerve-hardware]] (does not cover alliance control pitfall — net-new here).

## Inferred (low-confidence — do not distill as fact)

- The PathPlanner named-command registration status (prompt [5]) is unverified at the code level — inferred confirmed from prior-day commit 8136977
- Push failure cause in prompts [14-15] `/login` is unrecoverable; GitHub auth issue is the most likely explanation

## Likely missing

Match results from Day 3, whether the red alliance fix was deployed before or after competition matches on Day 3, and the specific doc content added to capture the alliance-control lesson beyond what's in the changelog entry.
