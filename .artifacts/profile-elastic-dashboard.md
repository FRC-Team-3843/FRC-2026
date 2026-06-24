---
id: elastic-dashboard
title: Elastic dashboard (driver dashboard + live tuning)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, dashboard, telemetry, tool]
aliases: [elastic, elastic dashboard, elastic-layout]
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
---

# Elastic dashboard (driver dashboard + live tuning)

> The robot's driver dashboard; its layout is deployed as `elastic-layout.json`, and it's the surface for live PID/value tuning via NetworkTables `/Tuning/`.

## Context

Elastic is the driver-station dashboard for the 2026 robot. Its layout ships with the robot code as `2026Robot/src/main/deploy/elastic-layout.json`. Beyond driving, it's the live-tuning surface: `TunableNumber`/`TunableBoolean` widgets publish under the NetworkTables `/Tuning/` table, and PID gains are exposed as Sendables through a DashboardManager. The hood-tuning workflow runs here: open the Shooter tab, adjust, hit "Save Hood", which persists to `deploy/hood-config.json`. Live tuning only works when `TelemetryConstants.TUNING_MODE` is true — see `elastic-tuning-mode`.

## Observations

- [registry] Layout deployed as `deploy/elastic-layout.json`. #dashboard
- [registry] Live tuning via `TunableNumber`/`TunableBoolean` on NT `/Tuning/`; PID as Sendable via DashboardManager. #tuning
- [registry] Hood-tuning workflow: Shooter tab → "Save Hood" → writes `deploy/hood-config.json`. #tuning
- [registry] Tuning widgets are no-ops unless `TelemetryConstants.TUNING_MODE` is true (see `elastic-tuning-mode`). #tuning

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (driver dashboard for the 2026 robot)
- relates-to [[hood-config-json]] (the "Save Hood" persistence target)
- relates-to [[elastic-tuning-mode]] (the TUNING_MODE gate on live tuning)
