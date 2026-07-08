---
id: photonvision-coprocessor
title: PhotonVision coprocessor (Beelink Mini S, robot vision)
schema_version: 2
artifact_kind: memory
memory_class: semantic
created: 2026-06-13T21:55:41Z
updated: 2026-06-23T00:00:00Z
valid_until: null
author: claude
session: null
tags: [frc, vision, hardware, coprocessor]
aliases: [photonvision coprocessor, vision coprocessor, beelink mini s, vision pi]
status: active
supersedes: null
confidence: 58
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
model: claude-opus-4-8
model_basis: confirmed
provenance:
  harvest: deterministic
  recall-extract: claude-sonnet-4-6
  find-missing: claude-sonnet-4-6
  precision-judge: claude-opus-4-8
lifecycle: active
scope: FRC-2026
load_profile: on_demand
---

# PhotonVision coprocessor (Beelink Mini S, robot vision)

> The headless vision compute box on the robot — a Beelink Mini S running PhotonVision; DISTINCT from the home-lab Beelink Proxmox host.

## Context

A dedicated coprocessor on the robot runs the AprilTag vision pipeline so the RoboRIO only does pose fusion. It is a Beelink Mini S (Intel N100) running Ubuntu 24.04 with PhotonVision as a headless systemd service. On the robot network it answers at `10.38.43.11:5800`. The vision system is wired but gated off (`VisionConstants.ENABLE_VISION = false`) until the right camera is mounted and calibrated (see [[frc-2026-live-todo]] / `2026Robot\TODO-COMPETITION.md` TODO).

IMPORTANT — this is NOT the same machine as the PersonalContext `beelink` entity. That Beelink is the home-lab Proxmox host (tailnet `100.71.104.115`, runs the HomeHub HA VM). This one is a separate physical box living on the robot for vision only.

## Observations

- [registry] Hardware: Beelink Mini S / Intel N100, Ubuntu 24.04, headless PhotonVision systemd service. #vision
- [registry] Robot-network address: `10.38.43.11:5800` (PhotonVision dashboard/stream). #vision
- [registry] SSH access cited as `murray-robotics-3843@10.38.43.11`; setup-time Wi-Fi address was `192.168.100.215`. #vision
- [registry] DISTINCT from PersonalContext `beelink` (Proxmox host `100.71.104.115`) — different machine, different purpose. #hardware
- [registry] Vision is disabled (`ENABLE_VISION = false`) pending right-camera mount/wire/calibrate (open TODO in [[frc-2026-live-todo]] / `2026Robot\TODO-COMPETITION.md`). #vision

## Open Questions

PhotonVision version was cited as v2026.3.2 in the candidate harvest but not re-verified against the box at seed time — confirm on next robot session.

- A 2026-02-09 sourcing-research conversation ([[photonvision-4-camera-coprocessor-selection]]) explored a 4-ThriftyCam-camera setup (Radxa Zero 3E rejected on performance, Orange Pi 4 Pro rejected as unsupported/wrong-SoC, Jetson Orion ruled out on cost) and converged on testing an owned Beelink S12 (Intel N95) for all 4 cameras before buying anything new — same Alder-Lake-N family as the N100 in this profile; unclear whether that research directly led to this specific N100 unit, or whether the final camera count actually fits on one board. Worth confirming against `2026Robot\TODO-COMPETITION.md` when vision bring-up actually happens. <!-- @claude 2026-07-07T00:00:00Z -->

## Relations

- relates-to [[frc-2026]] (vision coprocessor for the 2026 robot)
- relates-to [[photonvision]] (runs the PhotonVision software / pipeline)
- different-from [[beelink]] (NOT the home-lab Proxmox host — same vendor, different machine)
- relates_to [[photonvision-4-camera-coprocessor-selection]] (earlier hardware-sourcing research session, FRC-2026 scope) <!-- @claude 2026-07-07T00:00:00Z -->
