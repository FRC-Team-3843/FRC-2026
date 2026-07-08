---
id: photonvision-4-camera-coprocessor-selection
artifact_kind: memory
schema_version: 2
title: Budget Linux board for PhotonVision — 4-camera coprocessor sourcing research
created: 2026-07-07T00:00:00Z
updated: 2026-07-07T00:00:00Z
author: claude
model: claude-sonnet-5
model_basis: confirmed
status: active
memory_class: episodic
session: web-claude-web-a3356878-70ec-43f1-b91b-43a28f140adf
surface: web
platform: claude-web
source_url: https://claude.ai/chat/a3356878-70ec-43f1-b91b-43a28f140adf
derived_from: C:\Users\dover\AppData\Local\acc\transcripts\web\claude-web\a3356878-70ec-43f1-b91b-43a28f140adf.jsonl
entities: [photonvision-coprocessor, photonvision]
tags: [frc, vision, hardware, coprocessor]
aliases: [4 camera photonvision board, thriftycam coprocessor sizing, orange pi vs beelink vision]
source_basis: transcript
confidence: 70
human_edited: false
sensitivity: normal
---

# Budget Linux board for PhotonVision — 4-camera coprocessor sourcing research

> 2026-02-09 web-chat research trail for FRC Team 3843's PhotonVision vision-coprocessor hardware, walking from a single cheap SBC up to a 4-ThriftyCam setup and settling on a "test what we already own first" plan.

## Context

Conversation with the FRC team mentor (single monochrome global-shutter camera question, escalating to a 4-camera AprilTag/ThriftyCam rig). This predates the currently-committed hardware recorded in [[photonvision-coprocessor]] (a Beelink Mini S / Intel N100, `ENABLE_VISION = false` pending camera mount) — this session is the earlier exploratory pass that fed into that decision; the N100 finally chosen is the same Alder-Lake-N family as the N95 unit tested here, so this is very likely the direct predecessor research to the current hardware pick.

## Discussion

- Baseline: PhotonVision's stated minimum is 2GB RAM + ARM Cortex-A53-class CPU; Raspberry Pi 3 (1GB RAM) falls short, Raspberry Pi 4 2GB (~$45-55) is the cheapest board that officially qualifies for one camera.
- **Rejected — Radxa Zero 3E 2GB (~$21-29):** an FRC community (Chief Delphi) field report showed only 424x240@40fps 3D tracking and tracking failure while moving; verdict "not suitable for competition," teaching/bench use only. RK3566 CPU too weak despite meeting the nominal minimum.
- **Rejected — Orange Pi 4 Pro 4GB (~$35-49):** brand-new board (late 2025), uses Allwinner A733 (not RK3588 like Orange Pi 5); zero documented FRC/PhotonVision usage, not on PhotonVision's supported-board list, no RKNN object-detection support for that SoC. Price gap to Orange Pi 5 too small to justify the risk.
- For 4 cameras, PhotonVision community guidance caps at 1-2 cameras per coprocessor; 4 cameras on one small SBC is described as untested ("I haven't seen anyone try plugging in 4 cameras into an Orange Pi. Many mini PCs don't even have suitable USB bandwidth for that" — Chief Delphi).
- **Jetson Orion clarification:** ThriftyCam's own product page states "up to 5 per SBC" on a Jetson Orion module vs. "2 per SBC" on an Orange Pi 5 — this is a camera-count spec, not a benchmark result; ruled out anyway on cost (~$300-500+) and unclear PhotonVision support.
- **Pivot to owned hardware:** the team already owns Beelink mini PCs (an S12 with Intel N95, and a Ryzen-based SER5, currently in other use) plus Dell mini PCs — all zero marginal cost. A published Chief Delphi guide documents Beelink+Ubuntu+PhotonVision competition use (60+ fps reported on an older Celeron N5095 unit); the N95 is a newer, faster Alder-Lake-N part, so expected to do at least as well.
- **USB-bandwidth reasoning (the substantive technical thread):** the user correctly flagged that "4x USB 3.2 ports" likely share one xHCI controller (confirmed: Alder-Lake-N PCH has a single USB 3.2 xHCI host controller). Worked bandwidth math: ThriftyCam (OV9281 sensor) runs MJPG-compressed at up to 100fps@1280x800, estimated ~8-15 MB/s/camera compressed (vs. ~123 MB/s/camera if uncompressed YUY2) — so 4 cameras (~32-60 MB/s) sit well inside a single controller's real-world ~400-600 MB/s budget on paper. Community consensus (PhotonVision docs + Chief Delphi threads) still caps recommended practice at 1-2 cameras/coprocessor, attributed to protocol overhead, frame-timing collisions and interrupt contention rather than raw throughput — i.e. the bottleneck is empirical/architectural, not something the bandwidth arithmetic alone predicts.
- **Used-eBay tangent, then reversed:** briefly proposed used enterprise "TinyMiniMicro" boxes (Dell OptiPlex Micro / HP EliteDesk / Lenovo ThinkCentre Tiny, ~$60-200/pair used) as a cheap-and-powerful alternative, then walked it back once the user pushed back on size/power: those boxes are ~1L (7"x7"x1.4") vs. Beelink's much smaller ~4.5"x4"x1.6" or Orange Pi 5's ~4"x2.5", and draw 35-65W (T-series) vs. 15W (Beelink N95) vs. 5-10W (Orange Pi 5) — called a "false economy" for a robot with tight space/power budgets.

## Open Questions

- Whether the single owned Beelink S12 (N95) actually held up running all 4 ThriftyCams simultaneously, or whether the team fell back to splitting 2+2 across a second board — the conversation ends on a proposed test plan (Ubuntu 24.04 + PhotonVision + `lsusb -t` bandwidth check + frame-drop monitoring under simulated motion), not a result.
- The team's currently-committed hardware ([[photonvision-coprocessor]]) is a single Beelink Mini S / Intel N100 with vision still gated off — consistent with a single-board-worked outcome, but not confirmed by this transcript; worth reconciling against `2026Robot\TODO-COMPETITION.md` / a future robot session once vision bring-up actually happens.

## Relations

- relates_to [[photonvision-coprocessor]] (this session's hardware-selection research plausibly precedes/feeds the currently-recorded Beelink Mini S N100 coprocessor)
- relates_to [[photonvision]] (software the sourced hardware runs)
- relates_to [[task-frc-2026-live-todo]] (open "Vision bring-up: mount/wire/calibrate right camera" item this research supports)
