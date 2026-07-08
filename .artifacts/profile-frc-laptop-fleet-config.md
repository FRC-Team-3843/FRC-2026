---
id: frc-laptop-fleet-config
artifact_kind: memory
schema_version: 2
title: FRC laptop fleet setup (MSU1-4) — visual identity, power, and debloat config
created: 2026-07-08T00:00:00Z
updated: 2026-07-08T00:00:00Z
author: claude
model: claude-sonnet-5
model_basis: confirmed
status: active
memory_class: semantic
semantic_kind: configuration
entities: [frc-team-3843]
tags: [frc, laptop-setup, windows-config]
aliases: [MSU1-4 laptop config, team laptop image]
source_basis: transcript
confidence: 70
human_edited: false
sensitivity: normal
---

# FRC laptop fleet setup (MSU1-4) — visual identity, power, and debloat config

> Current configuration standard for the team's 4 competition laptops (Dell 7710s), covering color identity, power management, and OS debloat.

## Observations
- [config] Laptop color/name scheme: MSU1 = Black / Dark Grey #4A4A4A, MSU2 = White / Light Grey #B0B0B0, MSU3 = Navy Blue #002144, MSU4 = Gold #FDB736 #frc-laptop (final scheme after two revision rounds; supersedes any earlier Black/White/Navy/Gold-only or per-laptop dark/light-mode split)
- [config] All four laptops run Dark mode (final decision — an earlier per-laptop light/dark split by contrast was tried then reverted) #frc-laptop
- [config] Taskbar accent color: ON. Window borders (accent on borders): OFF #frc-laptop
- [config] Power plan — plugged in: Best Performance, screen off 30min, sleep after 4h, hibernate never, fast startup disabled. Battery: Balanced, screen off 10min, sleep after 1h, hibernate after 5h. Energy Saver at 30%. Battery % shown in taskbar. Close-lid = do nothing (both). Power button = sleep (both) #frc-laptop
- [config] Fast Startup disable: canonical documented method is Control Panel > Hardware and Sound > Power Options > Choose what the power buttons do > Change settings that are currently unavailable > uncheck "Turn on fast startup" — note the checkbox only appears when hibernation is enabled at all. NI Game Tools installer also has an install-time checkbox that disables fast startup automatically (documented as the easier alternative; `powercfg /h off` and the registry `HiberbootEnabled` method were considered and deliberately NOT documented, to avoid doc bloat) #frc-laptop
- [config] Windows Start/taskbar: More-pins layout; "recently added apps" and "most used apps" OFF; "recent/recommended files" ON in Start + File Explorer; lock-screen widgets removed; desktop kept clean (no icons); all apps pinned to Start; Chrome pinned to taskbar launching with `--incognito` #frc-laptop
- [config] Windows apps removed as standard debloat: OneDrive (conflicts with the team's Google Drive workflow), Xbox App + Game Bar, personal Teams, Clipchamp, News, Weather, Tips, Solitaire/Casual Games #frc-laptop
- [config] Google Drive workflow: Chrome + Google Drive for Desktop installed; configured to MIRROR (not selectively sync) Drive contents to `C:\Users\<username>\Google Drive` #frc-laptop

## Relations
- relates_to [[session-frc-laptop-setup-doc-review]] (source conversation)
- relates_to [[frc-laptop-setup-doc-review]] (frc-laptop-fleet-config already relates_to this session as its source conversation; the session's own Relations section doesn't link back to the confi) <!-- @dreaming 2026-07-08 -->
- relates_to [[frc-team-3843]] (Cross-scope (FRC-2026 to PersonalContext): frc-laptop-fleet-config declares entities:[frc-team-3843] but has no link to the team's entity profile; the) <!-- @dreaming 2026-07-08 -->
