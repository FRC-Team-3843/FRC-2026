# FRC-2026 Repository - Claude Configuration

## How to Use This Configuration

This file is for Claude-specific behavior when working in the FRC-2026 repository.

**For technical standards:** Read `STANDARDS.md` in this directory.

---

## Technical Standards

**READ THIS FIRST:** `FRC-2026\STANDARDS.md`

That file contains ALL technical rules for FRC-2026:
- 2026 Motor APIs (REVLib, Phoenix6, Phoenix5)
- Command-based architecture requirements
- Naming conventions
- CAN bus base assignments
- Build commands
- Legacy warnings

---

## Claude-Specific Behavior

### When Working in FRC-2026

1. **Before starting work:**
   - Check `.agent-log\changelog.md` for recent activity
   - Read `STANDARDS.md` for all technical rules
   - Check `.agent-context.md` for current sprint status and team decisions

2. **During work:**
   - Follow all standards in STANDARDS.md strictly
   - Use 2026 APIs exclusively (SparkMax, Phoenix6)
   - Never copy code from FRC-2024 or FRC-2025 without refactoring to 2026 standards
   - Maintain command-based architecture with dependency injection

3. **After completing work:**
   - Log changes to `.agent-log\changelog.md`
   - Use this format:
     ```
     ### [YYYY-MM-DD HH:MM] CLAUDE [ACTION_TYPE]
     - Description of changes
     - Files: <paths from repo root>
     - Notes: Important context for other agents
     - PENDING: (optional) What needs follow-up
     ```

### Claude Workflow Tips

- **File Navigation:** You can read any file in the repo. Use this to verify patterns across multiple files.
- **Search Capabilities:** Use grep/glob to find patterns across the codebase before making changes.
- **Testing:** After implementing changes, suggest running `./gradlew build` and `./gradlew test` from the project directory.
- **Documentation:** When explaining code, reference specific line numbers (e.g., `DriveSubsystem.java:45`).

---

## Cross-Agent Protocol

### Activity Logging

**Location:** `.agent-log\changelog.md`

**Before work:** Check changelog for recent changes by other agents (Gemini, Codex).
**After work:** Log all significant changes with `[CLAUDE]` tag.

### Handoff Tracking

**Location:** `.agent-log\handoffs.md`

If you leave work incomplete or encounter blockers:
1. Update handoffs.md with task status
2. Note what was completed and what's pending
3. Document any blockers or issues
4. Suggest which agent should continue (or mark as `ANY`)

---

## Repository Structure

```
FRC-2026/
├── STANDARDS.md               ← READ THIS for all technical rules
├── CLAUDE.md (this file)      ← Claude behavior for repo
├── GEMINI.md                  ← Gemini behavior for repo
├── AGENTS.md                  ← Codex behavior for repo
├── .agent-context.md          ← Current sprint status
├── .agent-log/
│   ├── changelog.md           ← All activity for this repo
│   └── handoffs.md            ← Task handoffs
│
└── 2026Robot/                 ← Project folder
    ├── CLAUDE.md              ← Project-level redirect
    ├── GEMINI.md              ← Project-level redirect
    ├── AGENTS.md              ← Project-level redirect
    ├── src/main/java/...      ← Source code
    └── src/test/java/...      ← Tests
```

---

## Key Reminders

- **Always read STANDARDS.md** before making code changes
- **Use 2026 APIs exclusively** in this repo
- **Never copy legacy code** from FRC-2024 or FRC-2025 without refactoring
- **Log all significant changes** to help other agents coordinate
- **Check changelog regularly** to avoid duplicate work

---

For cross-agent coordination protocol, see: `C:\github\CLAUDE.md`
