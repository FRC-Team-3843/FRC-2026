---
id: feeder-opening-geometry-foam-balls
artifact_kind: memory
schema_version: 2
title: Feeder opening geometry for 6" foam balls (FRC-2026)
created: 2026-07-07T00:00:00Z
updated: 2026-07-07T00:00:00Z
author: claude
model: claude-sonnet-5
model_basis: confirmed
status: active
memory_class: episodic
session: web-claude-web-1180341e-6a65-4a27-baba-729f8589fc99
surface: web
platform: claude-web
source_url: https://claude.ai/chat/1180341e-6a65-4a27-baba-729f8589fc99
derived_from: C:\Users\dover\AppData\Local\acc\transcripts\web\claude-web\1180341e-6a65-4a27-baba-729f8589fc99.jsonl
entities: []
tags: [frc, frc-2026, mechanical-design, feeder, hopper]
aliases: [feeder geometry, foam ball feeder, single-file feeder throat]
source_basis: transcript
confidence: 70
human_edited: false
sensitivity: normal
---

# Feeder opening geometry for 6" foam balls (FRC-2026)

> Web-chat design consultation on feeder/hopper opening geometry for 6" foam-ball game pieces on the FRC-2026 robot, converging on a single-file throat width plus funnel-wall/outlet-placement recommendations for a 27"x27" bulk storage box.

## Context
Short (6-message) web conversation, 2026-01-20, asking for feeder-opening geometry advice for feeding 6" foam balls (the FRC-2026 game piece) from bulk storage into a hopper. No CAD tool use — pure design-advice Q&A, later relevant to the repo's `FeederSubsystem`/`HopperSubsystem`.

## Discussion
- Opening-size guidance: minimum opening ~1.5-2x ball diameter (9-12") to avoid bridging; for single-file metering, aim ~7-9" wide.
- User pushed back citing a different AI's rule of thumb: "exactly 1.5x or significantly larger than 2x" — flagging 1.5-2x as a bridging "danger zone" (2 balls can wedge side-by-side near the top of that range). Assistant agreed this framing is more precise than its own first answer.
- Shape options discussed: tapered/funnel (45-60 deg from horizontal for gravity feed), rectangular slot (7-8" wide x 6-8" tall), circular (10-12" min, but bridging-prone with compressible foam).
- General tips: avoid 90 deg internal corners (use >=1-2" radii), consider low-friction wall material (polycarbonate/HDPE) for visibility + reduced friction, add active agitation (paddle/flap) if passive geometry alone bridges, test with a full load (small vs. large ball counts behave differently).
- Given the actual constraint - single-file downstream feed from a ~27"x27" bulk storage box - the assistant recommended: steep funnel walls (60-70 deg from horizontal, since foam balls are high-friction), an asymmetric/offset outlet (breaks bridging symmetry vs. a centered exit), a "chisel"/wedge transition (narrow to ~8" in one dimension first, keep the other tall, then transition height), and 2-3" corner radii throughout. Active-assist fallback options: flexible flap/brush at the throat, single rotating paddle/agitator, or polycord/tubing "fingers" to disrupt arch formation.
- Conversation ends with the assistant offering to sketch transition-geometry options; no reply captured (last turn in mirror is the assistant's offer).

## Open Questions
- Was a sketch ever requested/produced in a follow-up conversation (not present in this transcript)?
- Which final geometry (steep-funnel + offset outlet vs. chisel-wedge, with or without active agitation) was actually chosen for the physical feeder/hopper design?

## Notes for Future Sessions
This is design-advice content only — no repo files, CAD documents, or code were touched in this conversation. The concrete numeric guidance (throat width, funnel angle, corner radii, offset-outlet trick) is durable and worth surfacing to whoever designs/tunes the physical feeder mechanism; see the companion reference proposal for the extracted parameters.

## Relations
- relates_to [[FeederSubsystem]] (this conversation's geometry guidance targets the mechanism this subsystem drives)
- relates_to [[HopperSubsystem]] (adjacent mechanism in the same ball-handling path)
- relates_to [[feeder-hopper-geometry-guidelines]] (The reference artifact already declares derived_from this episodic session, but the session's Relations section doesn't link back to the companion ref) <!-- @dreaming 2026-07-08 -->
- relates_to [[smokey-mountain-mechanism-swerve-sysid-tuning]] (This episodic note's own Open Questions ask whether a final feeder/hopper geometry was ever chosen and built; this session is the corpus's actual buil) <!-- @dreaming 2026-07-08 -->
