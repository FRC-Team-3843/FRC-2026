---
id: recovered-2026-03-20-0a6281e5
model: claude-sonnet-4-6
model_basis: confirmed
original_session_model: unattributed
original_session_model_basis: unattributed
title: "recovered: Smokey Mountain Day 2 — ShooterCalculator, HoodConfig, drive tuning, alliance bug diagnosis"
schema_version: 2
created: 2026-06-21T00:00:00Z
updated: 2026-06-21T00:00:00Z
valid_until: null
author: claude
session: recovered-0a6281e5
original_session_date: 2026-03-20
tags: [recovered, reconstructed, frc, competition]
aliases: []
related: [recovered-2026-03-17-aa7616b8, recovered-2026-03-21-ef7e1f2d]
status: active
supersedes: null
confidence: 57
source_basis: transcript
human_edited: false
sensitivity: normal
decisions: []
artifact_kind: memory
memory_class: episodic
semantic_kind: state
---

# recovered: Smokey Mountain Day 2 — ShooterCalculator, HoodConfig, drive tuning, alliance bug diagnosis

> ⚠ RECOVERED/RECONSTRUCTED — NOT a verbatim transcript. The assistant side of this session
> was permanently deleted; only the user's prompts + project artifacts survive. Ground truth =
> the verbatim user intent + the artifact-cited (COMMIT/CHANGELOG/NOTE/PLAN) facts below.
> A claim in a faithful section that lacks an artifact citation is NOT ground truth — treat it
> as prompt-derived (user intent) or narrative, never as a confirmed outcome. Inferred items are
> labeled and must NOT be distilled as fact. Reconstructing model: claude-sonnet-4-6 (confirmed);
> original session model: unattributed. See recovered-transcripts/CALIBRATION.md.

## From the user's prompts (ground truth — intent + user-stated facts)

**Session context (prompt [1]):** Day 2 Smokey Mountain Regional; robot wiring not finished until the day before load-in; all coding on the fly. Prompt [2] establishes session opens with a catch-up: "I forgot to have you update docs and commit from last session."

**PathPlanner auto setup (prompts [5-12]):**
- Robot ~140 lb with bumper + battery; bumper 33×33 inches
- Starting position placed near human player station, ~1 foot from side wall
- Two auto routines: "Main Auto" (deploy intake → shoot → move to HPS → shoot again) and "Just Shoot" (deploy + shoot)
- turret starting angle ~-45° from front (later corrected: ~45° clockwise from front, heading -90° then corrected to ~45°)

**ShooterCalculator design (prompts [15-18]):**
- Uses wheel speed, launch angle, robot location, heading; slippage factor user-confirmed
- Servo pos 1: 30° from vertical (70° from horizontal); pos 2: 45°/45°; shooter tip ~30 inches height; goal height 56 inches
- Air resistance consideration raised by user; optimize for vertical landing angle; independent slip tuning per hood position
- Shoot-on-move required; 8 balls in auto; hopper/feeder run until auto ends but gated on at-speed
- Alliance-aware coordinate flip required; auto-shoot on separate operator button; loose at-speed + turret-position thresholds
- "Just Shoot" location needs ~3800 RPM high shot; slip factor 0.26 used to produce this (prompt [117])

**Drive tuning (prompts [27], [34], [44-48]):**
- Both forward/backward + side-to-side inverted after deploy (prompt [27]); iteratively corrected
- Ramp rates: user confirmed slow=0.4, fast=0.1 (prompt [34])
- Heading lock spinning without input → too aggressive → reduce further; final values in commits

**HoodConfig (prompts [70-78]):**
- Servo positions from JSON file; 4 positions (2 per servo); update from tuning dashboard; save button required
- User confirmed values: L near=120, L far=35, R near=120, R far=35
- Right bumper (far position) not moving → fixed

**D-pad RPM presets (prompts [50-61]):**
- "Hands on the clock": up=slowest, left=fastest; no hood change on D-pad
- First useful RPM=4000, capped at 6000; final presets: 3000/4000/5000/6000

**Alliance control debugging (prompts [80-104]):**
- Robot drove wrong direction in auto (red alliance), opposite on blue — user walked through symptom in detail
- Bevel gear orientation discussed (prompt [101-102]): user confirmed "bevel in right orientation is just having 0 correct"
- `allianceRelativeControl` + `zeroGyroWithAlliance` combination = double-flip on red (diagnosed here; fix committed in next session)
- Right trigger = voltage mode full send (prompt [112])

## Artifact-cited outcomes (COMMIT / CHANGELOG / NOTE / PLAN)

**`dbf42ec`** (2026-03-20 07:54, FRC-2026) — "Competition day: full mechanism code, tuning, and fixes"
- Catch-up commit: contains prior session (aa7616b8) work; triggered by prompt [2] "I forgot to have you update docs and commit from last session." Attribution to prior session confirmed by commit timing (63 min after aa7616b8 ended).

**`8136977`** (2026-03-20 07:57, FRC-2026) — "Update docs for competition state, add PathPlanner navgrid"
- Second catch-up: docs + PathPlanner navgrid; same attribution note as dbf42ec.

**`5c41288`** (2026-03-20 15:28, FRC-2026) — "Smokey Mountain Day 2: auto-shoot calculator, hood config, drive tuning"
- **Primary new-work commit for this session.** Confirmed contents: `ShooterCalculator` (trajectory with air drag, alliance flip, shoot-on-move, per-hood-position slip); `HoodConfig` JSON-backed per-servo angles + dashboard save; auto-shoot NamedCommand "autoShoot"; feeder gated on atSpeed+turretAtPosition; drive: translation axis fix, heading lock (driver A toggle), drive current limit 150A; right trigger = shooter speed; D-pad = RPM presets; manual hood via LB/RB; Elastic layout fixed + hood sliders; PathPlanner paths (Deploy Intake, Move to HPS), bumper dimensions updated.

**`92e663d`** (2026-03-21 10:02, FRC-2026) — "Competition tuning: drive inversion, duty cycle trigger, heading lock, RPM presets"
- New work done in this session; committed at 10:02 Mar 21, triggered by prompt [121] "Make sure everything is documented and lets commit and push." Confirmed contents: all 4 modules drive:true; joystick standard negation restored; heading lock p=0.4/d=0.01, 0.05 rad deadband, toggle driver A; right trigger = raw duty cycle; D-pad RPMs 3000/4000/5000/6000; hood manual-only via LB/RB; auto-shoot X toggles on, Y cancels; slip factor 0.26; NOTES.md controller quick reference updated.

**[2026-03-20 09:00] CLAUDE [DOCS]** (`FRC-2026\.changelog.md`) — "Updated README.md and NOTES.md to reflect current competition state." Topically consistent with catch-up commits in this session.

**Adjacent Codex entries (NOT this session):** `[2026-03-20 10:25-13:56] CODEX` — 5 entries covering PhotonVision camera transforms, Beelink docs, networking verification. Parallel Codex session; not this Claude session.

**OUT OF WINDOW:** `1791bac` (2026-03-23) — red alliance fix commit; diagnosis occurred in prompts [80-104] of this session but the code fix was in session ef7e1f2d.

## Inferred (low-confidence — do not distill as fact)

- ShooterCalculator's specific air-drag coefficient, slip factor initial default, and per-position ball-speed estimates are not in user prompts and not cited in commit messages — inferred from prompt intent and `5c41288`'s description
- Heading lock PID values tried-and-abandoned during the [39-48] block (before settling at p=0.4/d=0.01 in 92e663d) are unrecoverable

## Likely missing

On-field match results, specific heading lock PID values tried and reverted during the [39-48] multi-deploy block, whether the ShooterCalculator was actually activated in matches vs manual RPM presets, and which auto routine was used in each match.
