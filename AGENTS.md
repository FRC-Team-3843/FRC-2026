# Repository Guidelines

## Project Structure & Module Organization
- Active code is in `2026Robot/`.
- Java sources: `2026Robot/src/main/java/frc/robot` (subsystems, commands, RobotContainer).
- Deploy assets: `2026Robot/src/main/deploy/` with `pathplanner/` and `swerve/` configurations.
- Vendor libraries: `2026Robot/vendordeps/`.
- Team number and WPILib prefs: `2026Robot/.wpilib/wpilib_preferences.json`.
- This repo follows the standards in `C:\GitHub\CLAUDE.md` and the strict 2026 rules in `C:\GitHub\GEMINI.md`.

## Build, Test, and Development Commands
Run from `2026Robot/`:
- `./gradlew build` — compile and package.
- `./gradlew test` — run JUnit 5 tests (add `src/test/java`).
- `./gradlew deploy` — deploy to RoboRIO.
- `./gradlew simulateJava` — run simulation with GUI.
- `./gradlew clean` — clean outputs.

## Coding Style & Naming Conventions
- Java 17 + WPILib/GradleRIO.
- Indentation: 2 spaces.
- Names: `PascalCase` classes, `camelCase` methods/fields, `UPPER_SNAKE_CASE` constants; include units in names (e.g., `targetPosMeters`).
- **Command-based only**: subsystems own hardware, commands own logic; inject subsystems via constructors; no global statics.
- Prefer subsystem command-factory methods (inline `Command` builders) for simple actions.
- Use 2026 APIs: `SparkMax` + `SparkMaxConfig`, Phoenix 6 control request objects, modern `CommandXboxController` triggers.

## Testing Guidelines
- JUnit 5 is configured; create tests under `2026Robot/src/test/java` and name `*Test.java`.
- Keep hardware calls behind interfaces so tests can run on desktop.

## Commit & Pull Request Guidelines
- Recent commits use short, descriptive, imperative subjects; keep messages concise and specific.
- PRs should include purpose, affected subsystems, and simulation/driver-station notes; add screenshots for dashboard/UI changes.

## Configuration & Deployment Notes
- `src/main/deploy` is copied to `/home/lvuser/deploy` on the RoboRIO.
- Verify team number and CAN IDs before deploy.
