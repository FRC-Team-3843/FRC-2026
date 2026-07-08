---
id: frc-laptop-setup-doc-review
artifact_kind: memory
schema_version: 2
title: FRC laptop setup documentation review
created: 2026-07-07T00:00:00Z
updated: 2026-07-07T00:00:00Z
author: claude
model: claude-sonnet-5
model_basis: confirmed
status: active
memory_class: episodic
session: web-claude-web-a3e6d545-a937-4d23-9122-a345b19efdbe
surface: web
platform: claude-web
source_url: https://claude.ai/chat/a3e6d545-a937-4d23-9122-a345b19efdbe
derived_from: C:\Users\dover\AppData\Local\acc\transcripts\web\claude-web\a3e6d545-a937-4d23-9122-a345b19efdbe.jsonl
entities: [frc-team-3843]
tags: [frc, laptop-setup, windows-config, software-stack, onboarding]
aliases: ["FRC laptop setup doc review", "team laptop image review"]
source_basis: transcript
confidence: 70
human_edited: false
sensitivity: normal
---

# FRC laptop setup documentation review

> Bryant iteratively reviewed and revised his team's (FRC 3843) shared Google Doc for setting up the 4 competition laptops (MSU1-4), covering software stack, Windows personalization/power settings, and a Windows debloat pass, over a Jan 2026 web conversation.

## Context
Source doc: a Google Doc ("FRC laptop setup") Bryant shared read-only via the Drive connector (Claude cannot edit Google Docs directly — read-only integration). Because Claude can't edit Drive files, the assistant instead worked from a pulled-verbatim copy and produced revised `.docx` drops for Bryant to paste back in, at his request ("pull all the information verbatim and then make edits" rather than a from-scratch rebuild, to preserve his original voice/structure).

## Discussion

**Round 1 — initial review + software stack additions.** Assistant reviewed the doc; praised the color-coded visual-identity system and the power/BIOS section. Suggested additions: Angry IP Scanner (finding non-.local-responsive coprocessors/radios on 10.TE.AM.x), Wireshark (CAN-over-Ethernet/radio diagnosis), BalenaEtcher (coprocessor imaging), WinDirStat/TreeSize (SSD space). Suggested clarifications: uninstall legacy REV Hardware Client before RHC2 (they conflict); remove legacy Phoenix Tuner v1 before Phoenix Tuner X; consolidate Npcap notes (needed by both Wireshark and OpenMesh WinPcap-compat mode); fix a mangled Java 8 JRE link; note Choreo as PathWeaver's recommended replacement. Also proposed a separate "Pre-Competition Verification Checklist" apart from routine maintenance.

**Round 2 — research pass + big expansion.** At Bryant's request (link team GitHub to VS Code, separate setup-how-to from what-it-does descriptions, add workflows), the assistant researched current (2025/2026) setup specifics per tool and produced a ~3x-expanded doc: full descriptions for every tool, a new Section 4 (Software Setup Procedures: WPILib/VS Code + team-repo clone + Git identity, Driver Station, REV Hardware Client 2, Phoenix Tuner X, PathPlanner/Choreo, PhotonVision, Elastic Dashboard), and a new Section 5 (Common Workflows: deploy-to-robot, pre-season firmware update order, PID tuning session, post-match log analysis via AdvantageScope).

**Round 3 — small edits (software stack + naming).** Bryant set the final laptop naming/color scheme: 1-Black, 2-White, 3-Navy Blue, 4-Gold. Corrections: Npcap is bundled with the OpenMesh Radio Utility installer (no separate pre-install needed); Choreo is not on the Microsoft Store (points to GitHub releases instead); slicer choice changed to Bambu Studio + PrusaSlicer (Creality Print removed); WinDirStat removed entirely (team resets laptops between seasons, so disk-space creep isn't a real problem); removed a "7710 is power-heavy" comment; added a generic second swerve vendor library (YAGSL) to the libraries-to-import list.

**Round 4 — Windows debloat + Chrome/Drive.** Added removal of default Windows apps that cause trouble: OneDrive (conflicts with the team's Google Drive workflow), Xbox app/Game Bar, personal Teams, Clipchamp, News, Weather, Tips, Solitaire/Casual Games. Added Chrome + Google Drive for Desktop install — explicitly **mirroring** Drive contents to `C:\Users\<user>\Google Drive` rather than syncing selected folders.

**Round 5 — personalization + power management.** Bryant specified, in one message, a full personalization/power spec that the assistant wrote into the doc: remove lock-screen widgets, apply his pre-made team-colored backgrounds to desktop + lock screen, clean desktop (no icons), pin all apps to Start, pin Chrome to taskbar starting `--incognito`. Power plan: Best Performance (plugged in) / Balanced (battery); hibernate never (plugged in) / after 5h (battery); screen off 30min (plugged in) / 10min (battery); sleep after 4h (plugged in) / 1h (battery); battery % shown in taskbar; Energy Saver at 30%; close-lid = do nothing; power button = sleep.

**Round 6 — color hex codes + dark/light mode.** Added Murray State hex codes: MSU1 Dark Grey `#4A4A4A`, MSU2 Light Grey `#B0B0B0`, MSU3 Navy `#002144`, MSU4 Gold `#FDB736`. Taskbar color accent ON, window borders OFF (Bryant's stated preference). Initially split dark/light mode by laptop (gold/white laptops dark for contrast) — **later (round 8) simplified to dark mode for all four laptops**, removing the per-laptop split.

**Round 7 — Start menu tuning.** Layout → "More pins"; "show recently added apps" and "show most used apps" both OFF; kept "show recent files/recommended files" ON in Start and File Explorer.

**Round 8 — Fast Startup fix (multi-pass).** Bryant reported the documented Control-Panel method for disabling Fast Startup didn't actually work. Assistant researched and found the checkbox only appears when hibernation is enabled at all, and offered 4 methods (NI Game Tools install-time checkbox, `powercfg /h off`, Control Panel, registry `HiberbootEnabled`). **Bryant's final call: keep only the Control Panel method in the doc, plus a note that NI Game Tools has an install-time checkbox that does this automatically** — deliberately rejected the command-line/registry alternatives as unnecessary doc bloat.

**Round 9 — closing Q&A.** Bryant asked how WPILib updates; assistant explained WPILib installs to per-year folders (no uninstall of the prior year needed) — running the new year's installer in place is sufficient, and the installer (not just opening VS Code) is required to also get updated desktop tools (Elastic, AdvantageScope).

## Notes for Future Sessions
- The underlying Google Doc itself was never actually edited by Claude (read-only Drive access) — Bryant pasted the assistant's `.docx` outputs back in himself. If a future session needs the CURRENT doc state, don't assume this transcript's content is still what's live; verify against the doc.
- Software stack net decisions worth remembering if this doc/topic resurfaces: Angry IP Scanner + Wireshark + BalenaEtcher IN; WinDirStat OUT (laptops reset between seasons); Bambu Studio + PrusaSlicer IN, Creality Print OUT; YAGSL added alongside whatever primary swerve library the doc already had.
- Fast Startup: Bryant explicitly prefers ONE documented method (Control Panel) over exhaustively documenting all four — a stated "don't over-document" preference for this doc specifically.
- Related durable facts (legacy-version conflicts for RHC2/Phoenix Tuner X, Choreo's actual distribution channel, WPILib's per-year-install update mechanism) are proposed as reconcile-appends to this scope's existing tool profiles rather than duplicated here.

## Relations
- relates_to [[reference-frc-2026-repo-source]] (team GitHub repo the laptops are configured to clone/push, per this doc's VS Code setup section)
- relates_to [[phoenix-tuner-x]] (Profile's 'remove legacy Phoenix Tuner v1' gotcha is cited inline as sourced from this session, but neither artifact's Relations section carries the l) <!-- @dreaming 2026-07-08 -->
- relates_to [[rev-hardware-client]] (Profile's 'legacy RHC1 conflicts with RHC2' gotcha is cited inline as sourced from this session, but neither Relations section links them.) <!-- @dreaming 2026-07-08 -->
- relates_to [[wpilib-gradlerio]] (Profile's per-year-install/no-uninstall-needed fact is cited inline as sourced from this session, but neither Relations section links them.) <!-- @dreaming 2026-07-08 -->
- relates_to [[choreolib]] (Profile's 'Choreo distributed via GitHub releases, not the Microsoft Store' correction is cited inline as sourced from this session, but neither Relat) <!-- @dreaming 2026-07-08 -->
- relates_to [[frc-laptop-fleet-config]] (frc-laptop-fleet-config already relates_to this session as its source conversation; the session's own Relations section doesn't link back to the confi) <!-- @dreaming 2026-07-08 -->
- relates_to [[frc-team-3843]] (Cross-scope (FRC-2026 to PersonalContext): session entities:[frc-team-3843] documents team-owned equipment setup but isn't linked from the team's enti) <!-- @dreaming 2026-07-08 -->
