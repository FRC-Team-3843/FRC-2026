# FRC-2026 — Entity Index (per-scope)

Per-scope entity surface for the FRC-2026 repo. Same 5-column shape as the personal index; only `status: active` entities get a row (§3.2 guard). Second-resolution detail lives in `.notes\`, never in these rows. Cross-cutting entities (team, build env, coprocessor host) are owned by `PersonalContext` and proposed for its index — not duplicated here.

| entity | aliases | scope | notes/sources | status |
|---|---|---|---|---|
| frc-2026 | 2026 robot, rebuilt, 2026Robot | FRC-2026 | `FRC-2026\.notes\frc-2026-repo-source-202606120630.md`; robot code in `2026Robot\` | active |
