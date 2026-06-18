# YYP Dual Robot Adaptation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve `_YYP` volleyball robot behavior while adapting the merged project for R1/R2 dual-robot builds.

**Architecture:** `_YYP` command, location, and vision/navigation behavior remains the baseline. Dual-robot differences are exposed through `robot_def.h` macros and small conditional compilation hooks around Delta/Serve and build paths.

**Tech Stack:** STM32H723 C firmware, CMake/Ninja, GCC ARM embedded toolchain, FreeRTOS, project message center.

---

### Task 1: Establish Red Build

**Files:**
- Reference: `application/cmd/robot_cmd.c`
- Reference: `modules/master_machine/master_process.c`
- Reference: `modules/master_machine/master_process.h`

- [ ] **Step 1: Run the failing build**

Run:

```powershell
cmake --build --preset Debug
```

Expected: FAIL in `application/cmd/robot_cmd.c` because the merged project still has the old vision API while `_YYP` cmd expects the volleyball navigation API.

### Task 2: Migrate Vision Navigation Protocol

**Files:**
- Modify: `modules/master_machine/master_process.h`
- Modify: `modules/master_machine/master_process.c`
- Reference: `D:/STM32_Tools/project/FZSD_VOLLEYBALL_YYP/modules/master_machine/master_process.h`
- Reference: `D:/STM32_Tools/project/FZSD_VOLLEYBALL_YYP/modules/master_machine/master_process.c`

- [ ] **Step 1: Replace the old self-aim protocol with `_YYP` protocol**

Use the `_YYP` definitions for `Vision_Recv_s`, `Vision_Send_s`, `VisionInit`, `VisionSend(Vision_Send_s *)`, and `VisionIsOnline`.

- [ ] **Step 2: Build**

Run:

```powershell
cmake --build --preset Debug
```

Expected: vision API errors disappear; remaining failures point to dual-robot macro or cmd integration gaps.

### Task 3: Complete R1/R2 Macro Surface

**Files:**
- Modify: `application/robot_def.h`

- [ ] **Step 1: Add robot-specific navigation constants**

Add R1/R2 constants for `NAV_MAX_SPEED`, `NAV_SPEED_GAIN`, and `NAV_ARRIVAL_DIST`, then map the common `_YYP` macro names inside the existing `ROBOT_R1` / `ROBOT_R2` selection block.

- [ ] **Step 2: Build**

Run:

```powershell
cmake --build --preset Debug
```

Expected: `NAV_*` undefined errors are gone.

### Task 4: Preserve YYP Cmd Flow and Reconnect Delta

**Files:**
- Modify: `application/cmd/robot_cmd.c`

- [ ] **Step 1: Keep `_YYP` switch and navigation behavior**

Do not change left-switch manual/auto behavior, right-switch heading/launcher behavior, or automatic navigation math.

- [ ] **Step 2: Re-add only the current project's Delta command publication**

Register `delta_cmd` and `delta_feed`, map `g_launcher_status` plus `robot_state` to `Delta_Action_e`, publish `Delta_Ctrl_Cmd_s`, and keep debug feedback local.

- [ ] **Step 3: Use `_YYP` vision send API names**

Send pose through `Vision_Send_s` fields from `_YYP`: `mode`, `state`, `robot_x`, `robot_y`, and `robot_yaw`.

- [ ] **Step 4: Build**

Run:

```powershell
cmake --build --preset Debug
```

Expected: `robot_cmd.c` compiles without old/new API mismatches.

### Task 5: Finish Serve and Location Build Adaptation

**Files:**
- Modify: `application/Serve/serve.c`
- Modify: `Makefile`

- [ ] **Step 1: Use dual-robot serve motor macro**

Change `ServeInit` to use `SERVE_MOTOR_ID` instead of a hard-coded motor ID.

- [ ] **Step 2: Point Makefile at restored location paths**

Replace deleted `modules/imu/dm_imu.c` with `application/location/dm_imu.c`, add `application/location/optical_flow.c`, and include `application/location`.

- [ ] **Step 3: Build**

Run:

```powershell
cmake --build --preset Debug
```

Expected: the CMake build succeeds for the currently selected robot.

### Task 6: Verify Both Robots

**Files:**
- Temporarily modify and restore: `application/robot_def.h`

- [ ] **Step 1: Verify R1**

Ensure `ROBOT_R1` is enabled and `ROBOT_R2` is disabled, then run:

```powershell
cmake --build --preset Debug
```

Expected: PASS.

- [ ] **Step 2: Verify R2**

Temporarily switch to `ROBOT_R2`, run:

```powershell
cmake --build --preset Debug
```

Expected: PASS and no Serve compile path errors.

- [ ] **Step 3: Restore intended robot selection**

Restore the original robot selection from before verification.
