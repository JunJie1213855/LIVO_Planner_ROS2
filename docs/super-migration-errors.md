# SUPER Migration Error Log

**Date:** 2026-07-15
**Target:** ROS2 Humble
**Base:** src/SUPER (HKU MaRS Lab)
**Result:** ✅ ALL 6 PACKAGES COMPILE SUCCESSFULLY

---

## Phase 1: Environment Setup

| Item | Status |
|------|--------|
| ROS2 Humble sourced | ✅ |
| libglfw3-dev, libglew-dev, libncurses5-dev, libncursesw5-dev | ✅ |
| libeigen3-dev, libdw-dev | ✅ |
| ros-humble-mavros-msgs | ✅ (installed manually) |
| ROS2 deps (pcl-ros, tf2-ros, pcl-conversions) | ✅ |

## Phase 2: ROS2 Template Swap

```bash
bash scripts/select_ros_version.sh ROS2
```

Swapped all 6 packages from ROS1 (catkin) to ROS2 (ament_cmake):
- super_planner, rog_map, mars_quadrotor_msgs, marsim_render, perfect_drone_sim, mission_planner

## Phase 3: Package Build & Fix

### 1. mars_quadrotor_msgs — ✅ No errors
- Built clean on first attempt

### 2. rog_map — ✅ 1 fix
- **File:** `rog_map/CMakeLists.txt` line 70
- **Issue:** `${THIRD_PARTY}` (uppercase, undefined) → `${third_party_libs}` (lowercase, defined)
- **Root cause:** Typo in variable name

### 3. super_planner — ✅ 10 fixes across 5 files
- **Files modified:**
  - `src/super_core/super_planner.cpp` (lines 350, 356, 794, 1047)
  - `src/super_core/fsm.cpp` (line 205)
  - `src/utils/polytope.cpp` (line 122)
  - `src/utils/geometry_utils.cpp` (line 271)
  - `include/ros_interface/ros2/ros2_adapter.hpp` (lines 179, 316)
- **Issue:** Bare `isnan()` calls incompatible with C++17 + Eigen
- **Fix:** `isnan(...)` → `std::isnan(...)` (10 occurrences)
- **Root cause:** C++17 standard — Eigen defines `isnan` in global namespace via `GlobalFunctions.h`, causing ambiguity

### 4. marsim_render — ✅ No errors
- Built clean on first attempt

### 5. perfect_drone_sim — ✅ 3 fixes across 2 files
- **File 1:** `perfect_drone_sim/CMakeLists.txt`
  - Commented out `set(CMAKE_PREFIX_PATH ../../../../install)` (line 21)
  - Added `find_package(pcl_conversions REQUIRED)` (line 34)
  - Removed duplicate `find_package(marsim_render REQUIRED)` (line 54)
  - Added `pcl_conversions` to dependencies list
- **File 2:** `perfect_drone_sim/package.xml`
  - Added `<depend>pcl_conversions</depend>`
  - Added `<depend>visualization_msgs</depend>`
  - Added `<depend>tf2_ros</depend>`
- **Root cause:** Missing dependency declarations; hardcoded path interfering with colcon

### 6. mission_planner — ✅ 3 fixes across 2 files
- **File 1:** `mission_planner/CMakeLists.txt`
  - Commented out `set(CMAKE_PREFIX_PATH ../../../../install)` (line 23)
  - Removed unused `${OpenCV_LIBRARIES}` from THIRD_PARTY (line 49)
- **File 2:** `mission_planner/package.xml`
  - Added `<depend>visualization_msgs</depend>`
- **Root cause:** Stale path configuration; unnecessary dependency on OpenCV; missing package manifest entry

## Phase 4: Full Build Verification

```bash
$ colcon build --symlink-install --packages-select \
    mars_quadrotor_msgs rog_map super_planner marsim_render perfect_drone_sim mission_planner

Summary: 6 packages finished [1.79s]  ✅
```

## Summary

| Metric | Value |
|--------|-------|
| Total packages | 6 |
| Packages passed | 6 (100%) |
| Files modified | 10 |
| Total fixes | 17 |
| ROS1→ROS2 template swap | 1 script execution |
| External dep installed | 1 (ros-humble-mavros-msgs) |
