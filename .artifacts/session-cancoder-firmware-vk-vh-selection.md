---
id: cancoder-firmware-vk-vh-selection
artifact_kind: memory
schema_version: 2
title: CANcoder firmware version selection (vK vs vH) — FRC-2026
created: 2026-07-07T00:00:00Z
updated: 2026-07-07T00:00:00Z
author: claude
model: claude-sonnet-5
model_basis: confirmed
status: active
memory_class: episodic
session: web-claude-web-63dae62b-d2f9-4a7a-aad8-65245ab3556e
surface: web
platform: claude-web
source_url: https://claude.ai/chat/63dae62b-d2f9-4a7a-aad8-65245ab3556e
derived_from: C:\Users\dover\AppData\Local\acc\transcripts\web\claude-web\63dae62b-d2f9-4a7a-aad8-65245ab3556e.jsonl
entities: [phoenix-tuner-x, phoenix6-ctre]
tags: [frc, hardware, ctre, tool]
aliases: [cancoder firmware, vk vh firmware, phoenix tuner firmware update, cancoder hardware revision]
source_basis: transcript
confidence: 70
human_edited: false
sensitivity: normal
---

# CANcoder firmware version selection (vK vs vH) — FRC-2026

> Working out how to pick the right CANcoder firmware (vK vs vH) in Phoenix Tuner X on Phoenix 6 (25.2.2.0 / 2025-2026 season), including finding the tool's hidden firmware-selection dropdown.

## Context
Conversation from 2026-01-17 (imported 2026-07-05). User was updating CANcoder firmware for the FRC-2026 robot's swerve sensors and got stuck between two firmware options, "vK" and "vH", with no clear guidance from Phoenix Tuner X itself.

## Discussion
- Assistant initially (incorrectly) treated vK/vH as sequential firmware-version letters ("K is newer than H") — user corrected this: they are actually **hardware revision** designations, not firmware versions.
- Assistant then guessed the user would have to physically inspect the CANcoder board for a revision marking; user pushed back asking whether a web search had actually been done. It had not — a genuine research gap, not a hard technical limit.
- A web search (CTRE docs + GitHub + a Chief Delphi forum thread) found: CTRE only **officially documents two hardware generations** — the original CANcoder, and **CANcoder vH** (introduced late 2022 due to the chip shortage). **"vK" appears only in CTRE's GitHub firmware-file names — it is NOT documented anywhere as an official hardware revision**; other FRC teams on Chief Delphi were equally confused about it.
- Resolution method: Phoenix Tuner X's device list has a **"Hardware" column** that shows the actual connected device's hardware generation. If it does NOT say "vH", the device is original/non-vH hardware, and (per the assistant's best inference, since vK is undocumented) the vK-labeled firmware is the correct match — confirmed only by trying it (CTRE's flasher rejects a firmware file for the wrong hardware rather than bricking the device).
- Separately, the user discovered the ACTUAL cause of "Phoenix Tuner X shows every firmware file with no filtering": the firmware-family/version-letter picker is a **blank, unlabeled dropdown** that appears only after selecting Phoenix version + year and clicking "Update Firmware" — it is easy to miss entirely, which is why earlier in the conversation the tool appeared to dump every firmware file with no way to filter by compatibility.
- End state: user selected vK (matching non-vH hardware) via the hidden dropdown and confirmed the flash succeeded.

## Notes for Future Sessions
- Team gotcha for next time anyone re-flashes a CANcoder: in Phoenix Tuner X, pick Phoenix version + year -> click "Update Firmware" -> click the **blank dropdown that appears** (easy to miss) -> the vK/vH options appear there. Do not assume the vK/vH choice is a firmware-recency letter; it tracks the physical CANcoder hardware generation, and only "vH" (2022 chip-shortage revision) is officially documented by CTRE. If the device's Hardware column in Tuner X doesn't say vH, vK is the better first try.
- Open low-confidence point (not user-verified, only inferred from an undocumented naming pattern): whether "vK" always means "non-vH/original hardware" for every CANcoder unit, since CTRE has never documented it — worth a fresh CTRE-docs/Chief-Delphi check before relying on this for a different device.
- This complements the existing `[[phoenix-tuner-x]]` and `[[phoenix6-ctre]]` profiles (proposed as a reconcile-append; see distill proposals) rather than duplicating them.

## Relations
- relates_to [[phoenix-tuner-x]] (the tool used to determine hardware revision + flash firmware) <!-- @dreaming 2026-07-08 reciprocal -->
- relates_to [[phoenix6-ctre]] (Phoenix 6 framework the CANcoder firmware must match) <!-- @dreaming 2026-07-08 reciprocal -->