---
id: advantagescope
title: AdvantageScope (telemetry / SysId log viewer)
type: reference
schema_version: 1
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, telemetry, tool, sysid]
aliases: [advantagescope, advantage scope, wpilog viewer]
status: active
supersedes: null
confidence: 55
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
---

# AdvantageScope (telemetry / SysId log viewer)

> The tool the team uses to view `.wpilog` telemetry and analyze SysId runs (NOTES §4, §6).

## Context

AdvantageScope is the desktop viewer for `.wpilog` telemetry files and SysId analysis on the 2026 robot. SysId runs (logged by the RIO to `/home/lvuser/`) are pulled and analyzed here to produce drive/shooter/preshooter velocity-PID gains. It is referenced in NOTES §4 and §6 as the telemetry/analysis tool. The deferred SysId runs (drive, shooter, preshooter) noted in `.context.md` would be analyzed in AdvantageScope.

## Observations

- [registry] Views `.wpilog` telemetry files; used for SysId analysis. #telemetry
- [registry] Referenced in NOTES §4 and §6 as the analysis tool. #telemetry
- [registry] Consumes SysId logs the RIO writes to `/home/lvuser/`. #sysid

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (telemetry/SysId viewer for the 2026 robot)
- relates-to [[roborio]] (source of the `.wpilog` SysId logs)
