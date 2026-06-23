---
id: frc-2026-elastic-layout-editing
title: FRC-2026 Elastic Dashboard layout-JSON editing standard
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-23T00:54:31Z
updated: 2026-06-23T00:54:31Z
valid_until: null
author: claude
session: acc-monolith-decomp-pilot-20260623
model: claude-opus-4-8
model_basis: confirmed
tags: [frc, reference, dashboard, elastic, json]
aliases: [elastic layout editing, elastic-layout.json, widget type reference, nt path conventions, argb colors]
status: active
supersedes: null
confidence: 58
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 170
source_rel: FRC-2026\.standards.md
---

# FRC-2026 Elastic Dashboard layout-JSON editing standard

> How to edit the Elastic Dashboard layout JSON: file location, JSON structure, the widget-type reference, ARGB color format, NT path conventions, and the agent editing rules.

## Context
Decomposed from the `.standards.md` "Elastic Dashboard Layout Editing Standard" section during the ACC monolith-decomposition pilot. The dashboard entity + live-tuning workflow are covered by `elastic-dashboard`; this reference carries the layout-editing mechanics.

## Layout file location
Stored at `2026Robot/src/main/deploy/elastic-layout.json`, served to the dashboard via the WebServer on port 5800.

## JSON structure
```json
{
  "version": 1.0,
  "tabs": [
    {
      "name": "Tab Name",
      "grid_layout": {
        "layouts": [
          {
            "title": "Widget Title",
            "x": 0, "y": 0,
            "width": 6, "height": 4,
            "type": "WidgetTypeName",
            "properties": { },
            "topic": "/SmartDashboard/key"
          }
        ]
      }
    }
  ]
}
```

## Widget type reference

| Widget Type | Use Case | Key Properties | Topic Type |
|-------------|----------|----------------|------------|
| `Text Display` | Labels, read-only values | `fontSize` | String/Number |
| `Number Slider` | Tunable numbers | `min_value`, `max_value`, `divisions` | Number |
| `Toggle Switch` | Enable/disable flags | — | Boolean |
| `Graph` | Time series data | `time_displayed` | Number |
| `Radial Gauge` | Temp, voltage gauges | `min_value`, `max_value`, `start_angle`, `end_angle` | Number |
| `Boolean Box` | Status indicators | `colorWhenTrue`, `colorWhenFalse` | Boolean |
| `PIDController` | PID tuning widgets | — | Sendable PIDController |
| `Field` | Robot position on field | `field_game`, `robot_width`, `robot_length` | Sendable Field2d |
| `SwerveDrive` | Module state visualization | — | Sendable |
| `Camera Stream` | Video feed | `url` | — |
| `ComboBox Chooser` | Mode selection | — | Sendable Chooser |
| `Network Alerts` | Fault reporting | — | Alerts topic |
| `Match Time` | Match countdown | — | — |

## Color format
Colors use ARGB integer format:
- Blue: `4283215696`
- Red: `4294198070`
- Green: `4283215696`

## NT path conventions

| Prefix | Purpose | Direction |
|--------|---------|-----------|
| `/SmartDashboard/` | Standard WPILib publishing | Read-only |
| `/Tuning/` | Bidirectional tuning values (DashboardManager) | Read/Write |
| `/Control/` | Enable/disable flags | Read/Write |
| `/Sensors/` | Read-only sensor data | Read-only |
| `/System/` | System health metrics | Read-only |
| `/LiveWindow/` | WPILib test mode Sendables | Read-only |

## Agent editing rules
1. Always preserve the `version` field.
2. Do not remove existing tabs unless explicitly asked.
3. Use consistent grid positioning — no overlapping widgets.
4. Include `title` on every widget for clarity.
5. Use descriptive NT paths matching DashboardManager conventions.
6. Validate JSON before committing (no trailing commas, valid JSON).
7. Widget `width + x` must not exceed grid width (36 columns).
8. Standard widget sizes: small displays 3x2, graphs 10x5, gauges 4x4, PID widgets 6x5.

## Relations
- relates-to [[frc-2026]] (dashboard layout for the 2026 robot)
- relates-to [[elastic-dashboard]] (the dashboard entity + live-tuning workflow)
- relates-to [[frc-2026-logging-telemetry-standards]] (the NT naming convention)
