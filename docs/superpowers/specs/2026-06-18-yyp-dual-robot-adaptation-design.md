# YYP Dual Robot Adaptation Design

## Decision

The merged project adapts to the new `_YYP` design. `_YYP` is the main reference for the volleyball robot command flow, location layout, and vision/navigation protocol. The current merged project keeps its R1/R2 dual-robot conditional compilation work, but adapts those pieces to the `_YYP` interfaces instead of changing `_YYP` back to the older local interfaces.

## Scope

This change covers:

- `modules/master_machine/master_process.c/h`: migrate to the `_YYP` volleyball navigation protocol.
- `application/cmd/robot_cmd.c`: keep the `_YYP` command flow and adapt current project additions around it.
- `application/robot_def.h`: keep R1/R2 selection and map common macros used by `_YYP` code to robot-specific parameters.
- `application/chassis/chassis.c`: preserve existing R1/R2 chassis macro use.
- `application/delta` and `application/Serve`: keep dual-robot motor count and serve/no-serve behavior, while reconnecting the cmd-to-delta chain.
- Build files: ensure the restored `application/location` layout is used by both CMake and Makefile paths.

Out of scope:

- Reworking the Delta and Serve state machines beyond the minimal command-chain reconnection.
- Restoring the old self-aim vision protocol as a parallel protocol.
- Changing mechanical tuning values except where a missing R1/R2 mapping is required.

## Architecture

The project will have one active volleyball navigation protocol. The lower controller receives planned catch targets from the upper computer as `target_x`, `target_y`, `target_yaw`, and `target_time`, and reports robot pose/status back as `robot_x`, `robot_y`, `robot_yaw`, `mode`, and `state`.

`robot_cmd.c` remains the coordinator for remote-control mode selection, automatic navigation velocity generation, front-heading lock, launcher status, and outbound chassis commands. Robot-specific differences stay in `robot_def.h`, exposed through common macros such as `WHEEL_BASE`, `NAV_MAX_SPEED`, `DELTA_MOTOR_NUM`, and `ROBOT_HAS_SERVE`.

## Data Flow

1. `master_process` receives the `_YYP` plan packet and updates `Vision_Recv_s`.
2. `robot_cmd` reads `Vision_Recv_s`, optical-flow global position, and IMU yaw.
3. In manual mode, `robot_cmd` maps remote-control rocker values to chassis `vx/vy/wz`.
4. In automatic mode, `robot_cmd` computes `vx/vy` from target position minus optical-flow position.
5. `robot_cmd` publishes `Chassis_Ctrl_Cmd_s` to chassis.
6. `robot_cmd` publishes `Delta_Ctrl_Cmd_s` to delta, preserving the existing R1/R2 delta/serve compile-time split.
7. `master_process` sends robot pose/status back to the upper computer.

## Error Handling

- Automatic navigation outputs zero velocity when vision is offline, optical flow is offline, target is `(0, 0)`, or the robot has arrived within `NAV_ARRIVAL_DIST`.
- Emergency stop keeps chassis in `CHASSIS_ZERO_FORCE`.
- R2 compiles without Serve initialization, task execution, or Serve command publishing through `ROBOT_HAS_SERVE == 0`.
- R1/R2 conflict or missing robot selection remains a compile-time error.

## Verification

Primary verification is `cmake --build --preset Debug`.

Additional compile checks should cover:

- `ROBOT_R1` selected, `ROBOT_R2` disabled.
- `ROBOT_R2` selected, `ROBOT_R1` disabled.
- Makefile source/include paths reference `application/location` instead of deleted `modules/imu/dm_imu.*` or `modules/optical_flow/*`.

Warnings that predate this integration may remain, but new hard compile errors in the adapted files must be fixed.
