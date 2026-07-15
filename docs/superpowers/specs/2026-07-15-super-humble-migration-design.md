# SUPER Project ROS2 Humble Migration — Design Spec

**Date:** 2026-07-15
**Status:** Approved
**Scope:** Build & Fix — get all 6 SUPER packages compiling on ROS2 Humble

## 1. Goal

Migrate the SUPER project (`src/SUPER/`) from its current ROS1 (catkin) configuration to ROS2 Humble (colcon + ament_cmake), fixing all compilation errors until `colcon build` passes cleanly.

## 2. System Overview

SUPER contains 6 ROS packages, each with dual ROS1/ROS2 templates in a `ros/` subdirectory:

| Package | Type | Description |
|---------|------|-------------|
| `mars_quadrotor_msgs` | Messages | Custom quadrotor message definitions |
| `rog_map` | Library | Robocentric occupancy grid map |
| `super_planner` | Core | Main planning module (libsuper + fsm_node) |
| `marsim_render` | Visualization | MARSIM renderer |
| `perfect_drone_sim` | Simulation | Perfect drone simulator |
| `mission_planner` | Application | Waypoint mission planner |

## 3. Dependency Order

```
mars_quadrotor_msgs  (no internal deps)
    │
rog_map  (depends on mars_quadrotor_msgs)
    │
super_planner  (depends on rog_map, mars_quadrotor_msgs)
    │
marsim_render + perfect_drone_sim  (depend on mars_quadrotor_msgs)
    │
mission_planner  (depends on super_planner, rog_map)
```

## 4. Migration Steps

### Phase 1: Environment Setup
1. Verify ROS2 Humble environment sourced
2. Install system deps: `libglfw3-dev`, `libglew-dev`, `libncurses5-dev`, `libncursesw5-dev`, `libeigen3-dev`, `libdw-dev`
3. Install ROS deps: `ros-humble-pcl-ros`, `ros-humble-tf2-ros`, `ros-humble-pcl-conversions`, `ros-humble-yaml-cpp-vendor`

### Phase 2: ROS2 Template Swap
1. Run `bash scripts/select_ros_version.sh ROS2` to swap all 6 packages from ROS1 to ROS2 templates
2. Verify each package now has ROS2-style CMakeLists.txt and package.xml

### Phase 3: Incremental Build & Fix (per package, dependency order)
For each package:
1. `colcon build --packages-select <pkg>` 
2. Collect errors → classify (header missing, API change, cmake issue, etc.)
3. Fix → rebuild → repeat until package builds
4. Log all fixes to `docs/super-migration-errors.md`

### Phase 4: Full Build Verification
1. `colcon build --symlink-install` on all packages
2. Verify zero errors

## 5. Dual-Agent Architecture

### Planner Agent (Controller)
- **Role:** Task planning, error triage, final verification
- **Responsibilities:**
  - Maintain task queue in dependency order
  - Dispatch one package-build task at a time to Executor
  - Receive build output (success/failure + error log)
  - Log all errors to `docs/super-migration-errors.md`
  - Re-plan fix strategy based on error type
  - Run final `colcon build` verification
- **Implementation:** Main conversation loop, uses `SendMessage` + `Agent` tool

### Executor Agent (Worker)
- **Role:** Execute build tasks, apply fixes
- **Responsibilities:**
  - Wait for task from Planner
  - Execute: swap templates, modify CMakeLists/package.xml/source, run colcon build
  - Report back: exact build output, error messages, success confirmation
  - Wait for next task
- **Implementation:** Subagent spawned via `Agent` tool with `ecc:cpp-build-resolver` type

### Communication Protocol
```
Planner → Executor:  "Build package <X>: attempt colcon build, fix errors, report back"
Executor → Planner:  "PACKAGE <X>: SUCCESS" | "PACKAGE <X>: FAILED\n<error details>"
Planner → Executor:  (if failed) "Fix the following error in <file>: <specific error>"
Executor → Planner:  "FIX APPLIED: <summary>" | "NEED HELP: <blocker description>"
```

## 6. Known Risks

| Risk | Mitigation |
|------|-----------|
| Humble API differs from Foxy | Fix incrementally per error |
| Missing system dependencies | Install via apt before build |
| `mars_quadrotor_msgs` custom msgs | Build this package first, alone |
| Hardcoded paths in CMakeLists | Replace with ament macros |
| PCL version incompatibility | Use `pcl_conversions` bridge |

## 7. Success Criteria

- [ ] `colcon build --symlink-install` completes with 0 errors
- [ ] All 6 packages compile successfully
- [ ] `docs/super-migration-errors.md` documents all fixes applied

## 8. Out of Scope

- Runtime verification (launch and test)
- ROS2 Humble best-practice refactoring
- Functional modifications to planning algorithms
- Visualization/RVIZ fixes
