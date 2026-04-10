# FRC-2026 Repository - Agent Protocol

Shared workflow for all agents (Claude, Gemini, Codex) working in this repository.

## How to Use This Configuration

1. **This file:** Agent workflow, coordination, and structure
2. **`STANDARDS.md`:** ALL technical rules (motor APIs, architecture, naming, safety)
3. **`.agent-context.md`:** Current sprint status, team decisions, blockers
4. **`C:\GitHub\PROTOCOL.md`:** Root cross-agent coordination protocol

## Technical Standards Summary

**READ THIS FIRST:** `STANDARDS.md` in this directory.

It contains ALL technical rules for FRC-2026:
- 2026 Motor APIs (REVLib SparkMax, Phoenix6 TalonFX, Phoenix5 TalonSRX)
- Command-based architecture (mandatory dependency injection)
- Modern command factories (preferred pattern)
- Naming conventions (PascalCase, camelCase, m_ prefix, UPPER_SNAKE_CASE)
- CAN bus base assignments (0-12 reserved, 20-99 mechanisms)
- Build commands (gradlew build/deploy/test/simulate)
- Legacy warnings (DO NOT copy from FRC-2024 or FRC-2025)

## Before / During / After Work

### Before Starting Work

1. **Read `.agent-log\changelog.md`** (by direct path) for recent activity
2. **Read `.agent-log\handoffs.md`** (by direct path) for pending tasks or blockers
3. **Read `.agent-context.md`** for current sprint status and team decisions
4. **Read `STANDARDS.md`** for all technical rules

### During Work

- Follow all standards in STANDARDS.md strictly
- Use 2026 APIs exclusively (SparkMax, Phoenix6)
- Never copy code from FRC-2024 or FRC-2025 without refactoring to 2026 standards
- Maintain command-based architecture with dependency injection
- Check for duplicate work before implementing new features

### After Completing Work

1. **Log to `.agent-log\changelog.md`** with this format:
   ```
   ### [YYYY-MM-DD HH:MM] AGENT_TAG [ACTION_TYPE]
   - Description of changes
   - Files: <paths from repo root>
   - Notes: Important context for other agents
   - PENDING: (optional) What needs follow-up
   ```
2. **Action types:** `[IMPLEMENT]`, `[REFACTOR]`, `[FIX]`, `[TEST]`, `[CONFIG]`, `[DOCS]`, `[REVIEW]`
3. **If handing off incomplete work:** Update `.agent-log\handoffs.md` with status, blockers, and next steps.
4. **Agent log files are append-only.** Always Read first, then append. Never overwrite.

## Cross-Agent Protocol

### Activity Logging

**Location:** `.agent-log\changelog.md`

**Before work:** Check changelog for recent changes by all agents.
**After work:** Log all significant changes with your agent tag.

### Handoff Tracking

**Location:** `.agent-log\handoffs.md`

If you leave work incomplete or encounter blockers:
1. Update handoffs.md with task status
2. Note what was completed and what's pending
3. Document any blockers or issues
4. Suggest which agent should continue (or mark as `ANY`)

## Repository Structure

```
FRC-2026/
  PROTOCOL.md (this file)       <- Shared agent workflow
  STANDARDS.md                  <- Technical rules (READ THIS!)
  .agent-context.md             <- Current sprint status
  CLAUDE.md                     <- Claude pointer + tag
  GEMINI.md                     <- Gemini pointer + tag
  AGENTS.md                     <- Codex pointer + tag
  .agent-log/
    changelog.md                <- All activity for this repo
    handoffs.md                 <- Task handoffs

  2026Robot/                    <- Project folder
    CLAUDE.md                   <- Project-level redirect
    GEMINI.md                   <- Project-level redirect
    AGENTS.md                   <- Project-level redirect
    src/main/java/...           <- Source code
    src/test/java/...           <- Tests
```

## Configuration Architecture

### Standards Single Source of Truth

**STANDARDS.md** is the authoritative source for all FRC-2026 technical rules:
- Motor APIs (REVLib SparkMax, Phoenix6 TalonFX, Phoenix5 TalonSRX)
- Command-based architecture (mandatory dependency injection)
- Modern command factories (preferred pattern)
- Naming conventions (PascalCase, camelCase, UPPER_SNAKE_CASE)
- CAN bus base ranges (0-12 reserved, 20-99 mechanisms)
- Build commands (gradlew build/deploy/test/simulate)
- Legacy warnings (DO NOT copy from 2024/2025)

### Agent Configuration Files

Each agent (Claude, Gemini, Codex) has a thin pointer file:
- **CLAUDE.md** - Claude tag + pointer to this file
- **GEMINI.md** - Gemini tag + pointer to this file
- **AGENTS.md** - Codex tag + pointer to this file

All agent configs point here for workflow and to STANDARDS.md for technical rules.

### Activity Tracking

All agents log activity to `.agent-log\changelog.md` at the repo level:
- Prevents duplicate work
- Enables cross-agent coordination
- Tracks configuration changes
- Documents decisions

### When Standards Change

1. **Update `STANDARDS.md`** (source of truth)
2. **Test changes** (build/deploy)
3. **Log in changelog** with `[CONFIG]` action type
4. **Note what changed** so other agents re-read configs

## Key Reminders

- **Always read STANDARDS.md** before making code changes
- **Use 2026 APIs exclusively** in this repo
- **Never copy legacy code** from FRC-2024 or FRC-2025 without refactoring
- **Log all significant changes** to help other agents coordinate
- **Check changelog regularly** to avoid duplicate work

---

For root cross-agent coordination protocol, see: `C:\GitHub\PROTOCOL.md`
