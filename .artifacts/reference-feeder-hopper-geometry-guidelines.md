---
id: feeder-hopper-geometry-guidelines
artifact_kind: reference
schema_version: 2
title: Feeder/hopper opening geometry guidelines for 6" foam balls (FRC-2026)
created: 2026-07-08T00:00:00Z
updated: 2026-07-08T00:00:00Z
author: claude
model: claude-sonnet-5
model_basis: confirmed
status: active
source: web chat consultation (session web-claude-web-1180341e-6a65-4a27-baba-729f8589fc99), design advice not verified against a built mechanism
tags: [frc, frc-2026, mechanical-design, feeder, hopper, reference]
aliases: [feeder throat width, ball bridging, anti-jam geometry]
source_basis: transcript
confidence: 55
human_edited: false
sensitivity: normal
---

# Feeder/hopper opening geometry guidelines for 6" foam balls (FRC-2026)

> Reference parameters for a single-file feeder throat handling 6" foam-ball game pieces from a ~27x27" bulk storage box, per an AI design consultation (not yet validated against a built/tested mechanism).

## Context
FRC-2026 game piece = 6" foam ball. Storage/hopper geometry needs to convert bulk storage (approx. 27"x27" box, ~4 balls across) into a reliable single-file feed without bridging/jamming.

## Guidelines
- Single-file throat width target: ~7-9" (roughly 1.2-1.5x ball diameter). The 1.5-2x range (9-12") is flagged as a bridging "danger zone" for compressible, high-friction foam balls - avoid landing there.
- Funnel wall angle: 60-70 deg from horizontal for the bulk-to-single-file transition (foam balls are high-friction; shallower angles stall).
- Offset/asymmetric outlet (not centered) breaks the symmetry that lets stable arches/bridges form.
- "Chisel"/wedge transition: narrow the 27" box down to ~8" in one dimension first (keep height), then transition height downstream - easier to de-bridge a slot than a square funnel.
- Internal corner radii: >=2-3" everywhere walls meet; sharp corners anchor jams.
- Active-assist fallbacks if passive geometry alone bridges: flexible flap/brush at the throat, single rotating paddle/agitator, or polycord/tubing "fingers" to disrupt arches.
- Test with a full load - behavior with 3 balls vs. 15 balls differs materially.

## Notes for Future Sessions
This is unvalidated AI design advice, not measured/tested data - treat as a starting point for the FeederSubsystem/HopperSubsystem physical design, confirm against prototype testing.

## Relations
- derived_from [[session-feeder-opening-geometry-foam-balls]] (source conversation, FRC-2026 episodic note)
- relates_to [[FeederSubsystem]]
- relates_to [[HopperSubsystem]]
