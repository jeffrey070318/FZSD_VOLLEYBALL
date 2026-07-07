/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

// Jeffrey070318修改：整理robot_def.h宏定义顺序，按“选择开关-公共参数-R1参数-R2参数-统一映射”分区。
/* ============================== 编译选择 ============================== */
/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
                  // #define CHASSIS_BOARD // 只编译底盘板程序
                  // #define GIMBAL_BOARD  // 只编译云台板程序

// Jeffrey070318增加：整车类型条件编译开关，R1为三击球电机+发球拨杆，R2为双击球电机且无发球拨杆。
// #define ROBOT_R1 // 当前编译R1整车
#define ROBOT_R2 // 当前编译R2整车

#define VISION_USE_VCP // 视觉数据走USB虚拟串口
// #define VISION_USE_UART // 视觉数据走硬件串口

// Jeffrey070318增加：LCD屏幕任务为R1/R2共通调试显示，置1创建StartScreenTask，置0只编译不运行。
#define ROBOT_ENABLE_SCREEN_TASK 0

// Jeffrey070318修改：USB默认任务栈已加大，恢复视觉VCP调用用于算法联调。
#define ROBOT_ENABLE_VISION 1

// Jeffrey070318增加：光流计暂未连接，置0跳过OpticalFlowInit和周期读取，接好光流后改回1。
#define ROBOT_ENABLE_OPTICAL_FLOW 1

// Jeffrey070318增加：底盘调参用VOFA输出，走UART7 DMA；分频值按ChassisTask调用周期折算输出频率。
#define ROBOT_ENABLE_VOFA_CHASSIS_DEBUG 1
#define ROBOT_VOFA_CHASSIS_DEBUG_DIVIDER 20u

// Jeffrey070318增加：CMD整车遥控入口开关，置1启用RobotCMDInit/RobotCMDTask，置0保留单模块直测。
#define R2_DEBUG_ENABLE_CMD_APP 1

// Jeffrey070318增加：临时锁定pitch轴，不再由遥控器左摇杆控制；恢复遥控时置0。
#define ROBOT_LOCK_PITCH_TARGET 1
#define ROBOT_LOCK_PITCH_TARGET_POS PITCH_REMOTE_BACK_POS

// Jeffrey070318修改：进入遥控器整车测试阶段，CMD接管底盘和机械臂，单模块直测默认关闭。
#define R2_DEBUG_ENABLE_INS_TASK 1    // 1: 创建INS任务
#define R2_DEBUG_ENABLE_MOTOR_TASK 1  // 1: 创建全局电机控制任务(DJI底盘等)
#define R2_DEBUG_ENABLE_DAEMON_TASK 1 // 1: 创建守护/蜂鸣器任务
#define R2_DEBUG_ENABLE_CHASSIS_APP 1 // 1: 初始化并运行底盘应用
#define R2_DEBUG_ENABLE_DELTA_APP 1   // 1: 初始化并运行Delta机械臂应用

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// Jeffrey070318增加：检查R1/R2定义冲突，确保同一次编译只生成一种整车固件。
#if defined(ROBOT_R1) && defined(ROBOT_R2)
#error Conflict robot type definition! You can only define ROBOT_R1 or ROBOT_R2.
#endif

/* ============================== 公共参数 ============================== */
/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
#define VISION_MODE_COORDINATE 0 // Jeffrey070318增加：视觉target_x/y表示世界坐标，底盘按光流当前位置导航。
#define VISION_MODE_OFFSET 1     // Jeffrey070318增加：视觉target_x/y表示画面偏移，底盘用PID追踪偏移。

#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘正对时的yaw编码器值
#define YAW_ECD_GREATER_THAN_4096 0 // yaw对齐编码值是否跨过4096
#define PITCH_HORIZON_ECD 3412      // pitch水平位置对应的编码器值
#define PITCH_MAX_ANGLE 0           // pitch允许的最大角度
#define PITCH_MIN_ANGLE 0           // pitch允许的最小角度

#define ONE_BULLET_DELTA_ANGLE 36    // 拨盘发一球需要转过的角度
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机减速比
#define NUM_PER_CIRCLE 10            // 拨盘转一圈对应的装球数量

#define GYRO2GIMBAL_DIR_YAW 1   // IMU yaw方向相对云台的符号
#define GYRO2GIMBAL_DIR_PITCH 1 // IMU pitch方向相对云台的符号
#define GYRO2GIMBAL_DIR_ROLL 1  // IMU roll方向相对云台的符号

/* ============================== R1参数 ============================== */
// Jeffrey070318修改：R1参数按底盘、导航、CMD、机械臂分组，避免不同车种参数混在一起。
/* R1 chassis */
#define CHASSIS_R1_WHEEL_BASE 400.0f                        // R1前后轮中心距
#define CHASSIS_R1_TRACK_WIDTH 400.0f                       // R1左右轮中心距
#define CHASSIS_R1_CENTER_OFFSET_X 0.0f                     // R1云台中心相对底盘中心X偏移
#define CHASSIS_R1_CENTER_OFFSET_Y 0.0f                     // R1云台中心相对底盘中心Y偏移
#define CHASSIS_R1_RADIUS_WHEEL 60.0f                       // R1轮子半径
#define CHASSIS_R1_REDUCTION_RATIO_WHEEL 19.0f              // R1轮组电机减速比
#define CHASSIS_R1_MOTOR_LF_ID 1u                           // R1左前轮电机CAN ID
#define CHASSIS_R1_MOTOR_RF_ID 2u                           // R1右前轮电机CAN ID
#define CHASSIS_R1_MOTOR_LB_ID 4u                           // R1左后轮电机CAN ID
#define CHASSIS_R1_MOTOR_RB_ID 3u                           // R1右后轮电机CAN ID
#define CHASSIS_R1_MOTOR_TYPE M3508                         // R1底盘电机型号
#define CHASSIS_R1_MOTOR_LF_REVERSE MOTOR_DIRECTION_REVERSE // R1左前轮电机方向
#define CHASSIS_R1_MOTOR_RF_REVERSE MOTOR_DIRECTION_REVERSE // R1右前轮电机方向
#define CHASSIS_R1_MOTOR_LB_REVERSE MOTOR_DIRECTION_REVERSE // R1左后轮电机方向
#define CHASSIS_R1_MOTOR_RB_REVERSE MOTOR_DIRECTION_REVERSE // R1右后轮电机方向
#define CHASSIS_R1_SPEED_PID_KP 10.0f                       // R1底盘速度环P
#define CHASSIS_R1_SPEED_PID_KI 0.0f                        // R1底盘速度环I
#define CHASSIS_R1_SPEED_PID_KD 0.0f                        // R1底盘速度环D
#define CHASSIS_R1_SPEED_PID_INTEGRAL_LIMIT 3000.0f         // R1底盘速度环积分限幅
#define CHASSIS_R1_SPEED_PID_MAX_OUT 12000.0f               // R1底盘速度环输出限幅
#define CHASSIS_R1_CURRENT_PID_KP 0.5f                      // R1底盘电流环P
#define CHASSIS_R1_CURRENT_PID_KI 0.0f                      // R1底盘电流环I
#define CHASSIS_R1_CURRENT_PID_KD 0.0f                      // R1底盘电流环D
#define CHASSIS_R1_CURRENT_PID_INTEGRAL_LIMIT 3000.0f       // R1底盘电流环积分限幅
#define CHASSIS_R1_CURRENT_PID_MAX_OUT 15000.0f             // R1底盘电流环输出限幅
#define CHASSIS_R1_HEADING_PID_KP 70.0f                     // R1车头保持角度环P
#define CHASSIS_R1_HEADING_PID_KI 1.5f                      // R1车头保持角度环I
#define CHASSIS_R1_HEADING_PID_KD 120.0f                    // R1车头保持角度环D
#define CHASSIS_R1_HEADING_PID_MAX_OUT 2500.0f              // R1车头保持角度环输出限幅
#define CHASSIS_R1_HEADING_PID_DEADBAND 0.3f                // R1车头保持角度死区
#define CHASSIS_R1_HEADING_PID_INTEGRAL_LIMIT 400.0f        // R1车头保持角度环积分限幅
#define CHASSIS_R1_KEEP_FRONT_STATIC_WZ_DEADBAND 120.0f     // R1静止车头保持时小于该wz认为不足以驱动有效纠偏，直接置零防抖
#define CHASSIS_R1_ROTATE_WZ 2000.0f                        // R1小陀螺旋转角速度
#define CHASSIS_R1_VX_DIRECTION -1                          // R1底盘X方向符号, 1=正向 -1=反向
#define CHASSIS_R1_VY_DIRECTION -1                          // R1底盘Y方向符号, 1=正向 -1=反向

/* R1 navigation and cmd */
#define NAV_R1_MAX_SPEED 10000.0f                            // R1导航速度上限
#define NAV_R1_SPEED_GAIN 6000.0f                            // R1导航距离到速度的比例
#define NAV_R1_ARRIVAL_DIST 0.15f                            // R1导航到点判定距离
#define CMD_R1_REMOTE_MOVE_SCALE 30.0f                       // R1遥控器平移摇杆比例
#define CMD_R1_REMOTE_YAW_SCALE 4.0f                         // R1遥控器旋转摇杆比例
#define CMD_R1_REMOTE_YAW_MAX_WZ 2000.0f                     // R1遥控器旋转二次曲线满杆输出上限
#define CMD_R1_REMOTE_YAW_STICK_MAX 660.0f                   // R1遥控器旋转摇杆满量程
#define CMD_R1_REMOTE_DEADBAND 50                            // R1遥控器摇杆死区
#define CMD_R1_REMOTE_STOP_DIAL_THRESHOLD 300                // R1遥控器拨轮急停阈值
#define VISION_R1_MODE VISION_MODE_COORDINATE                // Jeffrey070318增加：R1视觉模式，坐标模式/偏移模式二选一。
#define VISION_R1_PID_X_KP 0.5f                              // Jeffrey070318增加：R1视觉偏移模式X轴PID比例系数。
#define VISION_R1_PID_X_KI 0.01f                             // Jeffrey070318增加：R1视觉偏移模式X轴PID积分系数。
#define VISION_R1_PID_X_KD 0.0f                              // Jeffrey070318增加：R1视觉偏移模式X轴PID微分系数。
#define VISION_R1_PID_X_MAXOUT 10000.0f                      // Jeffrey070318增加：R1视觉偏移模式X轴速度输出限幅。
#define VISION_R1_PID_Y_KP 0.5f                              // Jeffrey070318增加：R1视觉偏移模式Y轴PID比例系数。
#define VISION_R1_PID_Y_KI 0.01f                             // Jeffrey070318增加：R1视觉偏移模式Y轴PID积分系数。
#define VISION_R1_PID_Y_KD 0.0f                              // Jeffrey070318增加：R1视觉偏移模式Y轴PID微分系数。
#define VISION_R1_PID_Y_MAXOUT 10000.0f                      // Jeffrey070318增加：R1视觉偏移模式Y轴速度输出限幅。
#define VISION_R1_PID_DEADBAND 2.0f                          // Jeffrey070318增加：R1视觉偏移PID死区，过滤小像素误差。
#define VISION_R1_PID_INTEGRAL_RATIO 0.3f                    // Jeffrey070318增加：R1视觉偏移PID积分限幅相对MaxOut的比例。
#define OPTICAL_FLOW_R1_PROTOCOL OPTICAL_FLOW_UPIXELS_NO_TOF // Jeffrey070318增加：R1光流协议，学长版使用无TOF帧。
#define OPTICAL_FLOW_R1_SCALE_X OPTICAL_FLOW_DEFAULT_SCALE   // Jeffrey070318增加：R1光流传感器X轴角位移缩放。
#define OPTICAL_FLOW_R1_SCALE_Y 20000.0f                     // Jeffrey070318增加：R1光流传感器Y轴角位移缩放。
#define OPTICAL_FLOW_R1_SWAP_XY 1u                           // Jeffrey070318增加：R1光流安装方向是否交换X/Y。
#define OPTICAL_FLOW_R1_X_DIRECTION 1                        // Jeffrey070318增加：R1光流X方向符号修正。
#define OPTICAL_FLOW_R1_Y_DIRECTION -1                       // Jeffrey070318增加：R1光流Y方向符号修正。
#define OPTICAL_FLOW_R1_ENABLE_GLOBAL_FRAME 1u               // Jeffrey070318增加：R1光流是否启用yaw旋转到世界坐标。

/* R1 arm */
#define DELTA_R1_MOTOR_NUM 3u                             // R1击球机构电机数量
#define DELTA_R1_MOTOR1_ID 1u                             // R1击球电机1 CAN ID
#define DELTA_R1_MOTOR2_ID 2u                             // R1击球电机2 CAN ID
#define DELTA_R1_MOTOR3_ID 3u                             // R1击球电机3 CAN ID
#define PITCH_R1_MOTOR_ID 4u                              // R1机械臂pitch电机CAN ID
#define SERVE_R1_MOTOR_ID 6u                              // R1发球拨杆电机CAN ID
#define DELTA_R1_SPEED 16.0f                              // R1击球机构目标运动速度
#define DELTA_R1_POSITION_THRESHOLD 0.15f                 // R1击球机构到位误差阈值
#define SERVE_R1_POSITION_THRESHOLD 0.15f                 // R1发球拨杆到位误差阈值
#define MIT_DELTA_R1_HIT_KP 300.0f                        // R1击球动作delta电机MIT位置P
#define MIT_DELTA_R1_HIT_KD 3.0f                          // R1击球动作delta电机MIT速度D
#define MIT_DELTA_R1_HIT_TORQ 5.0f                        // R1击球动作delta电机MIT前馈力矩
#define MIT_DELTA_R1_GET_KP 200.0f                        // R1接球动作delta电机MIT位置P
#define MIT_DELTA_R1_GET_KD 3.0f                          // R1接球动作delta电机MIT速度D
#define MIT_DELTA_R1_GET_TORQ 3.0f                        // R1接球动作delta电机MIT前馈力矩
#define MIT_DELTA_R1_SLOW_KP 50.0f                        // R1慢速动作delta电机MIT位置P
#define MIT_DELTA_R1_SLOW_KD 3.0f                         // R1慢速动作delta电机MIT速度D
#define MIT_DELTA_R1_SLOW_TORQ 0.0f                       // R1慢速动作delta电机MIT前馈力矩
#define MIT_PITCH_R1_HIT_KP 250.0f                        // R1击球动作pitch电机MIT位置P
#define MIT_PITCH_R1_HIT_KD 2.0f                          // R1击球动作pitch电机MIT速度D
#define MIT_PITCH_R1_HIT_TORQ 8.0f                        // R1击球动作pitch电机MIT前馈力矩
#define MIT_PITCH_R1_GET_KP 100.0f                        // R1接球动作pitch电机MIT位置P
#define MIT_PITCH_R1_GET_KD 1.0f                          // R1接球动作pitch电机MIT速度D
#define MIT_PITCH_R1_GET_TORQ 2.0f                        // R1接球动作pitch电机MIT前馈力矩
#define PITCH_R1_POSITION_THRESHOLD 0.03f                 // Jeffrey070318增加：R1 pitch位置速度模式到位误差占位值，R1实车调参时再改。
#define PITCH_R1_TEST_FRONT_POS 0.6f                      // Jeffrey070318增加：R1 pitch前向安全测试位置占位值。
#define PITCH_R1_TEST_BACK_POS -0.6f                      // Jeffrey070318增加：R1 pitch背向安全测试位置占位值。
#define PITCH_R1_TEST_SPEED 1.0f                          // Jeffrey070318增加：R1 pitch位置速度模式测试速度占位值。
#define PITCH_R1_REMOTE_ZERO_POS 0.0f                     // Jeffrey070318增加：R1遥控器控制pitch时的机械零点。
#define PITCH_R1_REMOTE_FRONT_POS PITCH_R1_TEST_FRONT_POS // Jeffrey070318增加：R1左摇杆上推对应的pitch前向目标。
#define PITCH_R1_REMOTE_BACK_POS PITCH_R1_TEST_BACK_POS   // Jeffrey070318增加：R1左摇杆下拉对应的pitch背向目标。
#define PITCH_R1_REMOTE_SPEED PITCH_R1_TEST_SPEED         // Jeffrey070318增加：R1遥控器控制pitch的位置速度模式速度。
#define PITCH_R1_REMOTE_STICK_MAX 660.0f                  // Jeffrey070318增加：R1遥控器pitch摇杆满量程，用于比例映射。
#define PITCH_R1_STICK_DIRECTION 1                        // Jeffrey070318增加：R1 pitch摇杆方向, 1=上推前倾 -1=上推后仰
#define PITCH_R1_REMOTE_MODE 0                            // Jeffrey070318增加：R1 pitch摇杆映射模式, 0=原逻辑(中心=零点) 1=中心=最前点
#define DELTA_R1_ORIGINAL_POS 0.0f                        // R1击球机构初始目标位置
#define DELTA_R1_HIT_1_POS 0.0f                           // R1击球机构击球目标位置
#define DELTA_R1_BACK_POS 0.0f                            // R1击球机构回收目标位置
#define DELTA_R1_TEST_DOWN_POS -0.8f                      // R1测试下压目标位置
#define DELTA_R1_TEST_BACK_POS -0.4f                      // R1测试回收目标位置
#define DELTA_R1_TEST_TRIGGER_POS -0.18f                  // R1测试触发目标位置

/* ============================== R2参数 ============================== */
// Jeffrey070318修改：R2参数独立成块，后续调参时只进入R2区域修改。
/* R2 chassis */
#define CHASSIS_R2_WHEEL_BASE 400.0f                        // R2前后轮中心距
#define CHASSIS_R2_TRACK_WIDTH 400.0f                       // R2左右轮中心距
#define CHASSIS_R2_CENTER_OFFSET_X 0.0f                     // R2云台中心相对底盘中心X偏移
#define CHASSIS_R2_CENTER_OFFSET_Y 0.0f                     // R2云台中心相对底盘中心Y偏移
#define CHASSIS_R2_RADIUS_WHEEL 60.0f                       // R2轮子半径
#define CHASSIS_R2_REDUCTION_RATIO_WHEEL 19.0f              // R2轮组电机减速比
#define CHASSIS_R2_MOTOR_LF_ID 1u                           // R2左前轮电机CAN ID
#define CHASSIS_R2_MOTOR_RF_ID 2u                           // R2右前轮电机CAN ID
#define CHASSIS_R2_MOTOR_LB_ID 4u                           // R2左后轮电机CAN ID
#define CHASSIS_R2_MOTOR_RB_ID 3u                           // R2右后轮电机CAN ID
#define CHASSIS_R2_MOTOR_TYPE M3508                         // R2底盘电机型号
#define CHASSIS_R2_MOTOR_LF_REVERSE MOTOR_DIRECTION_REVERSE // R2左前轮电机方向
#define CHASSIS_R2_MOTOR_RF_REVERSE MOTOR_DIRECTION_REVERSE // R2右前轮电机方向
#define CHASSIS_R2_MOTOR_LB_REVERSE MOTOR_DIRECTION_REVERSE // R2左后轮电机方向
#define CHASSIS_R2_MOTOR_RB_REVERSE MOTOR_DIRECTION_REVERSE // R2右后轮电机方向
#define CHASSIS_R2_SPEED_PID_KP 10.0f                       // R2底盘速度环P
#define CHASSIS_R2_SPEED_PID_KI 0.1f                        // R2底盘速度环I
#define CHASSIS_R2_SPEED_PID_KD 0.0f                        // R2底盘速度环D
#define CHASSIS_R2_SPEED_PID_INTEGRAL_LIMIT 3000.0f         // R2底盘速度环积分限幅
#define CHASSIS_R2_SPEED_PID_MAX_OUT 12000.0f               // R2底盘速度环输出限幅
#define CHASSIS_R2_CURRENT_PID_KP 0.5f                      // R2底盘电流环P
#define CHASSIS_R2_CURRENT_PID_KI 0.0f                      // R2底盘电流环I
#define CHASSIS_R2_CURRENT_PID_KD 0.0f                      // R2底盘电流环D
#define CHASSIS_R2_CURRENT_PID_INTEGRAL_LIMIT 3000.0f       // R2底盘电流环积分限幅
#define CHASSIS_R2_CURRENT_PID_MAX_OUT 15000.0f             // R2底盘电流环输出限幅
#define CHASSIS_R2_HEADING_PID_KP 70.0f                     // R2车头保持角度环P
#define CHASSIS_R2_HEADING_PID_KI 1.5f                      // R2车头保持角度环I
#define CHASSIS_R2_HEADING_PID_KD 120.0f                    // R2车头保持角度环D
#define CHASSIS_R2_HEADING_PID_MAX_OUT 2500.0f              // R2车头保持角度环输出限幅
#define CHASSIS_R2_HEADING_PID_DEADBAND 0.3f                // R2车头保持角度死区
#define CHASSIS_R2_HEADING_PID_INTEGRAL_LIMIT 400.0f        // R2车头保持角度环积分限幅
#define CHASSIS_R2_KEEP_FRONT_STATIC_WZ_DEADBAND 120.0f     // R2静止车头保持时小于该wz认为不足以驱动有效纠偏，直接置零防抖
#define CHASSIS_R2_ROTATE_WZ 2000.0f                        // R2小陀螺旋转角速度
#define CHASSIS_R2_VX_DIRECTION 1                           // R2底盘X方向符号, 1=正向 -1=反向
#define CHASSIS_R2_VY_DIRECTION 1                           // R2底盘Y方向符号, 1=正向 -1=反向

/* R2 navigation and cmd */
#define NAV_R2_MAX_SPEED 16000.0f             // R2导航速度上限，按YYP视觉坐标导航参数对齐。
#define NAV_R2_SPEED_GAIN 60000.0f            // R2导航距离到速度的比例，按YYP视觉坐标导航参数对齐。
#define NAV_R2_ARRIVAL_DIST 0.10f             // R2导航到点判定距离，按YYP视觉坐标导航参数对齐。
#define R2_RIGHT2_FIXED_MOVE_TEST_Y 0.50f       // R2右二临时测试：按下后用光流固定向Y方向移动的距离(m)。
#define R2_RIGHT2_FIXED_MOVE_TEST_DEADBAND 0.02f // R2右二临时测试米制PID死区，避免复用视觉像素/误差死区。
#define CMD_R2_REMOTE_MOVE_SCALE 30.0f        // R2遥控器平移摇杆比例
#define CMD_R2_REMOTE_YAW_SCALE 4.0f          // R2遥控器旋转摇杆比例
#define CMD_R2_REMOTE_YAW_MAX_WZ 1600.0f      // R2遥控器旋转二次曲线满杆输出上限，优先保证角度微调精度。
#define CMD_R2_REMOTE_YAW_STICK_MAX 660.0f    // R2遥控器旋转摇杆满量程
#define CMD_R2_REMOTE_DEADBAND 80             // R2遥控器摇杆死区
#define CMD_R2_REMOTE_STOP_DIAL_THRESHOLD 300 // R2遥控器拨轮急停阈值

// 最终挑战
#define VISION_R2_MODE VISION_MODE_COORDINATE // Jeffrey070318增加：R2视觉模式，坐标模式/偏移模式二选一。
#define VISION_R2_PID_X_KP 120.0f             // Jeffrey070318增加：R2视觉偏移模式X轴PID比例系数。
#define VISION_R2_PID_X_KI 0.5f               // Jeffrey070318增加：R2视觉偏移模式X轴PID积分系数。
#define VISION_R2_PID_X_KD 30.0f              // Jeffrey070318增加：R2视觉偏移模式X轴PID微分系数。
#define VISION_R2_PID_X_MAXOUT 25000.0f       // Jeffrey070318增加：R2视觉偏移模式X轴速度输出限幅。
#define VISION_R2_PID_Y_KP 100.0f             // Jeffrey070318增加：R2视觉偏移模式Y轴PID比例系数。
#define VISION_R2_PID_Y_KI 0.5f               // Jeffrey070318增加：R2视觉偏移模式Y轴PID积分系数。
#define VISION_R2_PID_Y_KD 45.0f              // Jeffrey070318增加：R2视觉偏移模式Y轴PID微分系数。
#define VISION_R2_PID_Y_MAXOUT 20000.0f       // Jeffrey070318增加：R2视觉偏移模式Y轴速度输出限幅。
#define VISION_R2_PID_DEADBAND 2.0f           // Jeffrey070318增加：R2视觉偏移PID死区，过滤小像素误差。
#define VISION_R2_PID_INTEGRAL_RATIO 0.3f     // Jeffrey070318增加：R2视觉偏移PID积分限幅相对MaxOut的比例。
#define VISION_R2_SERVE_TRIGGER_DELAY_MS 80u  // R2视觉flag=1后延迟触发机械臂，便于调击球时机。

#define OPTICAL_FLOW_R2_PROTOCOL OPTICAL_FLOW_UPIXELS_NO_TOF // Jeffrey070318增加：R2光流协议，学长版使用无TOF帧。
#define OPTICAL_FLOW_R2_SCALE_X OPTICAL_FLOW_DEFAULT_SCALE   // Jeffrey070318增加：R2光流传感器X轴角位移缩放。
#define OPTICAL_FLOW_R2_SCALE_Y 20000.0f                     // Jeffrey070318增加：R2光流传感器Y轴角位移缩放。
#define OPTICAL_FLOW_R2_SWAP_XY 0u                           // Jeffrey070318增加：R2光流安装方向是否交换X/Y。
#define OPTICAL_FLOW_R2_X_DIRECTION 1                        // Jeffrey070318增加：R2光流X方向符号修正。
#define OPTICAL_FLOW_R2_Y_DIRECTION -1                       // Jeffrey070318增加：R2光流Y方向符号修正。
#define OPTICAL_FLOW_R2_ENABLE_GLOBAL_FRAME 1u               // Jeffrey070318增加：R2光流是否启用yaw旋转到世界坐标。

/* R2 arm */
#define DELTA_R2_MOTOR_NUM 2u                             // R2击球机构电机数量
#define DELTA_R2_MOTOR1_ID 1u                             // Jeffrey070318修改：R2击球机构电机1恢复正常CAN ID。
#define DELTA_R2_MOTOR2_ID 2u                             // Jeffrey070318修改：R2击球机构电机2恢复正常CAN ID。
#define DELTA_R2_MOTOR3_ID 3u                             // R2预留击球电机3 CAN ID
#define PITCH_R2_MOTOR_ID 4u                              // R2机械臂pitch电机CAN ID
#define DELTA_R2_SPEED 16.0f                              // R2击球机构目标运动速度
#define DELTA_R2_POSITION_THRESHOLD 0.15f                 // R2击球机构到位误差阈值
#define MIT_DELTA_R2_HIT_KP 300.0f                        // R2击球动作delta电机MIT位置P
#define MIT_DELTA_R2_HIT_KD 3.0f                          // R2击球动作delta电机MIT速度D
#define MIT_DELTA_R2_HIT_TORQ 5.0f                        // R2击球动作delta电机MIT前馈力矩
#define MIT_DELTA_R2_GET_KP 300.0f                        // R2接球动作delta电机MIT位置P
#define MIT_DELTA_R2_GET_KD 3.0f                          // R2接球动作delta电机MIT速度D
#define MIT_DELTA_R2_GET_TORQ 5.0f                        // R2接球动作delta电机MIT前馈力矩
#define MIT_DELTA_R2_SLOW_KP 10.0f                        // R2慢速动作delta电机MIT位置P
#define MIT_DELTA_R2_SLOW_KD 3.0f                         // R2慢速动作delta电机MIT速度D
#define MIT_DELTA_R2_SLOW_TORQ 0.0f                       // R2慢速动作delta电机MIT前馈力矩
#define MIT_PITCH_R2_HIT_KP 250.0f                        // R2击球动作pitch电机MIT位置P
#define MIT_PITCH_R2_HIT_KD 2.0f                          // R2击球动作pitch电机MIT速度D
#define MIT_PITCH_R2_HIT_TORQ 8.0f                        // R2击球动作pitch电机MIT前馈力矩
#define MIT_PITCH_R2_GET_KP 100.0f                        // R2接球动作pitch电机MIT位置P
#define MIT_PITCH_R2_GET_KD 1.0f                          // R2接球动作pitch电机MIT速度D
#define MIT_PITCH_R2_GET_TORQ 2.0f                        // R2接球动作pitch电机MIT前馈力矩
#define PITCH_R2_POSITION_THRESHOLD 0.03f                 // Jeffrey070318增加：R2 pitch位置速度模式到位误差阈值。
#define PITCH_R2_FRONT_LIMIT_POS 0.46f                    // Jeffrey070318增加：R2 pitch前向实测机械限位，调试目标不要直接取到这里。
#define PITCH_R2_BACK_LIMIT_POS -0.74f                    // Jeffrey070318增加：R2 pitch背向实测机械限位，按零点反向取负并留余量调试。
#define PITCH_R2_TEST_FRONT_POS 0.46f                     // Jeffrey070318修改：R2 pitch前向测试位置临时给到实测正向限位。
#define PITCH_R2_TEST_BACK_POS -0.74f                     // Jeffrey070318修改：R2 pitch背向测试位置临时给到实测负向限位。
#define PITCH_R2_TEST_SPEED 2.0f                          // Jeffrey070318增加：R2 pitch位置速度模式测试速度。
#define PITCH_R2_REMOTE_ZERO_POS 0.0f                     // Jeffrey070318增加：R2遥控器控制pitch时的机械零点。
#define PITCH_R2_REMOTE_FRONT_POS PITCH_R2_TEST_FRONT_POS // Jeffrey070318增加：R2左摇杆上推对应的pitch前向目标。
#define PITCH_R2_REMOTE_BACK_POS PITCH_R2_TEST_BACK_POS   // Jeffrey070318增加：R2左摇杆下拉对应的pitch背向目标。
#define PITCH_R2_REMOTE_SPEED PITCH_R2_TEST_SPEED         // Jeffrey070318增加：R2遥控器控制pitch的位置速度模式速度。
#define PITCH_R2_REMOTE_STICK_MAX 660.0f                  // Jeffrey070318增加：R2遥控器pitch摇杆满量程，用于比例映射。
#define PITCH_R2_STICK_DIRECTION -1                       // Jeffrey070318增加：R2 pitch摇杆方向, 1=上推前倾 -1=上推后仰
#define PITCH_R2_REMOTE_MODE 1                            // Jeffrey070318增加：R2 pitch摇杆映射模式, 0=原逻辑(中心=零点) 1=中心=最前点
#define DELTA_R2_ORIGINAL_POS 0.0f                        // R2击球机构初始目标位置
#define DELTA_R2_HIT_1_POS 0.70f                          // Jeffrey070318修改：R2右开关机械臂发出目标改为实测最高点位置。
#define DELTA_R2_BACK_POS 0.0f                            // R2击球机构回收目标位置
#define DELTA_R2_TEST_DOWN_POS 0.9f                       // Jeffrey070318修改：R2直测目标改为最高点位置，按实测从零点伸张约0.9rad。
#define DELTA_R2_TEST_BACK_POS 0.0f                       // Jeffrey070318修改：R2直测回收位置保持零点，便于从零点到最高点观察动作。
#define DELTA_R2_TEST_TRIGGER_POS 0.8f                    // Jeffrey070318修改：R2直测触发阈值同步改到接近最高点，用于判断是否到达大行程位置。

/* ============================== 当前车种统一宏 ============================== */
// Jeffrey070318修改：改回显式条件编译映射，未选车种分支会被IDE虚化，便于合并检查。
#if defined(ROBOT_R1)
#define ROBOT_HAS_SERVE 1                                                              // 当前车是否带发球拨杆
#define WHEEL_BASE CHASSIS_R1_WHEEL_BASE                                               // 业务代码使用的前后轮中心距
#define TRACK_WIDTH CHASSIS_R1_TRACK_WIDTH                                             // 业务代码使用的左右轮中心距
#define CENTER_GIMBAL_OFFSET_X CHASSIS_R1_CENTER_OFFSET_X                              // 业务代码使用的云台X偏移
#define CENTER_GIMBAL_OFFSET_Y CHASSIS_R1_CENTER_OFFSET_Y                              // 业务代码使用的云台Y偏移
#define RADIUS_WHEEL CHASSIS_R1_RADIUS_WHEEL                                           // 业务代码使用的轮子半径
#define REDUCTION_RATIO_WHEEL CHASSIS_R1_REDUCTION_RATIO_WHEEL                         // 业务代码使用的轮组减速比
#define CHASSIS_MOTOR_LF_ID CHASSIS_R1_MOTOR_LF_ID                                     // 业务代码使用的左前轮ID
#define CHASSIS_MOTOR_RF_ID CHASSIS_R1_MOTOR_RF_ID                                     // 业务代码使用的右前轮ID
#define CHASSIS_MOTOR_LB_ID CHASSIS_R1_MOTOR_LB_ID                                     // 业务代码使用的左后轮ID
#define CHASSIS_MOTOR_RB_ID CHASSIS_R1_MOTOR_RB_ID                                     // 业务代码使用的右后轮ID
#define CHASSIS_MOTOR_TYPE CHASSIS_R1_MOTOR_TYPE                                       // 业务代码使用的底盘电机型号
#define CHASSIS_MOTOR_LF_REVERSE CHASSIS_R1_MOTOR_LF_REVERSE                           // 业务代码使用的左前轮方向
#define CHASSIS_MOTOR_RF_REVERSE CHASSIS_R1_MOTOR_RF_REVERSE                           // 业务代码使用的右前轮方向
#define CHASSIS_MOTOR_LB_REVERSE CHASSIS_R1_MOTOR_LB_REVERSE                           // 业务代码使用的左后轮方向
#define CHASSIS_MOTOR_RB_REVERSE CHASSIS_R1_MOTOR_RB_REVERSE                           // 业务代码使用的右后轮方向
#define CHASSIS_SPEED_PID_KP CHASSIS_R1_SPEED_PID_KP                                   // 业务代码使用的速度环P
#define CHASSIS_SPEED_PID_KI CHASSIS_R1_SPEED_PID_KI                                   // 业务代码使用的速度环I
#define CHASSIS_SPEED_PID_KD CHASSIS_R1_SPEED_PID_KD                                   // 业务代码使用的速度环D
#define CHASSIS_SPEED_PID_INTEGRAL_LIMIT CHASSIS_R1_SPEED_PID_INTEGRAL_LIMIT           // 业务代码使用的速度环积分限幅
#define CHASSIS_SPEED_PID_MAX_OUT CHASSIS_R1_SPEED_PID_MAX_OUT                         // 业务代码使用的速度环输出限幅
#define CHASSIS_CURRENT_PID_KP CHASSIS_R1_CURRENT_PID_KP                               // 业务代码使用的电流环P
#define CHASSIS_CURRENT_PID_KI CHASSIS_R1_CURRENT_PID_KI                               // 业务代码使用的电流环I
#define CHASSIS_CURRENT_PID_KD CHASSIS_R1_CURRENT_PID_KD                               // 业务代码使用的电流环D
#define CHASSIS_CURRENT_PID_INTEGRAL_LIMIT CHASSIS_R1_CURRENT_PID_INTEGRAL_LIMIT       // 业务代码使用的电流环积分限幅
#define CHASSIS_CURRENT_PID_MAX_OUT CHASSIS_R1_CURRENT_PID_MAX_OUT                     // 业务代码使用的电流环输出限幅
#define CHASSIS_HEADING_PID_KP CHASSIS_R1_HEADING_PID_KP                               // 业务代码使用的车头角度环P
#define CHASSIS_HEADING_PID_KI CHASSIS_R1_HEADING_PID_KI                               // 业务代码使用的车头角度环I
#define CHASSIS_HEADING_PID_KD CHASSIS_R1_HEADING_PID_KD                               // 业务代码使用的车头角度环D
#define CHASSIS_HEADING_PID_MAX_OUT CHASSIS_R1_HEADING_PID_MAX_OUT                     // 业务代码使用的车头角度环输出限幅
#define CHASSIS_HEADING_PID_DEADBAND CHASSIS_R1_HEADING_PID_DEADBAND                   // 业务代码使用的车头角度死区
#define CHASSIS_HEADING_PID_INTEGRAL_LIMIT CHASSIS_R1_HEADING_PID_INTEGRAL_LIMIT       // 业务代码使用的车头角度环积分限幅
#define CHASSIS_KEEP_FRONT_STATIC_WZ_DEADBAND CHASSIS_R1_KEEP_FRONT_STATIC_WZ_DEADBAND // 业务代码使用的静止保持wz防抖阈值
#define CHASSIS_ROTATE_WZ CHASSIS_R1_ROTATE_WZ                                         // 业务代码使用的小陀螺角速度
#define CHASSIS_VX_DIRECTION CHASSIS_R1_VX_DIRECTION                                   // 业务代码使用的底盘X方向符号
#define CHASSIS_VY_DIRECTION CHASSIS_R1_VY_DIRECTION                                   // 业务代码使用的底盘Y方向符号
#define NAV_MAX_SPEED NAV_R1_MAX_SPEED                                                 // 业务代码使用的导航速度上限
#define NAV_SPEED_GAIN NAV_R1_SPEED_GAIN                                               // 业务代码使用的导航速度比例
#define NAV_ARRIVAL_DIST NAV_R1_ARRIVAL_DIST                                           // 业务代码使用的导航到点距离
#define CMD_REMOTE_MOVE_SCALE CMD_R1_REMOTE_MOVE_SCALE                                 // 业务代码使用的遥控平移比例
#define CMD_REMOTE_YAW_SCALE CMD_R1_REMOTE_YAW_SCALE                                   // 业务代码使用的遥控旋转比例
#define CMD_REMOTE_YAW_MAX_WZ CMD_R1_REMOTE_YAW_MAX_WZ                                 // 业务代码使用的遥控旋转满杆输出上限
#define CMD_REMOTE_YAW_STICK_MAX CMD_R1_REMOTE_YAW_STICK_MAX                           // 业务代码使用的遥控旋转摇杆满量程
#define CMD_REMOTE_DEADBAND CMD_R1_REMOTE_DEADBAND                                     // 业务代码使用的遥控摇杆死区
#define CMD_REMOTE_STOP_DIAL_THRESHOLD CMD_R1_REMOTE_STOP_DIAL_THRESHOLD               // 业务代码使用的拨轮急停阈值
#define VISION_MODE VISION_R1_MODE                                                     // Jeffrey070318增加：业务代码使用的视觉回传模式。
#define VISION_PID_X_KP VISION_R1_PID_X_KP                                             // Jeffrey070318增加：业务代码使用的视觉X轴P。
#define VISION_PID_X_KI VISION_R1_PID_X_KI                                             // Jeffrey070318增加：业务代码使用的视觉X轴I。
#define VISION_PID_X_KD VISION_R1_PID_X_KD                                             // Jeffrey070318增加：业务代码使用的视觉X轴D。
#define VISION_PID_X_MAXOUT VISION_R1_PID_X_MAXOUT                                     // Jeffrey070318增加：业务代码使用的视觉X轴输出限幅。
#define VISION_PID_Y_KP VISION_R1_PID_Y_KP                                             // Jeffrey070318增加：业务代码使用的视觉Y轴P。
#define VISION_PID_Y_KI VISION_R1_PID_Y_KI                                             // Jeffrey070318增加：业务代码使用的视觉Y轴I。
#define VISION_PID_Y_KD VISION_R1_PID_Y_KD                                             // Jeffrey070318增加：业务代码使用的视觉Y轴D。
#define VISION_PID_Y_MAXOUT VISION_R1_PID_Y_MAXOUT                                     // Jeffrey070318增加：业务代码使用的视觉Y轴输出限幅。
#define VISION_PID_DEADBAND VISION_R1_PID_DEADBAND                                     // Jeffrey070318增加：业务代码使用的视觉PID死区。
#define VISION_PID_INTEGRAL_RATIO VISION_R1_PID_INTEGRAL_RATIO                         // Jeffrey070318增加：业务代码使用的视觉PID积分限幅比例。
#define OPTICAL_FLOW_PROTOCOL OPTICAL_FLOW_R1_PROTOCOL                                 // Jeffrey070318增加：业务代码使用的光流协议。
#define OPTICAL_FLOW_SCALE_X OPTICAL_FLOW_R1_SCALE_X                                   // Jeffrey070318增加：业务代码使用的光流X轴缩放。
#define OPTICAL_FLOW_SCALE_Y OPTICAL_FLOW_R1_SCALE_Y                                   // Jeffrey070318增加：业务代码使用的光流Y轴缩放。
#define OPTICAL_FLOW_SWAP_XY OPTICAL_FLOW_R1_SWAP_XY                                   // Jeffrey070318增加：业务代码使用的光流X/Y交换开关。
#define OPTICAL_FLOW_X_DIRECTION OPTICAL_FLOW_R1_X_DIRECTION                           // Jeffrey070318增加：业务代码使用的光流X方向符号。
#define OPTICAL_FLOW_Y_DIRECTION OPTICAL_FLOW_R1_Y_DIRECTION                           // Jeffrey070318增加：业务代码使用的光流Y方向符号。
#define OPTICAL_FLOW_ENABLE_GLOBAL_FRAME OPTICAL_FLOW_R1_ENABLE_GLOBAL_FRAME           // Jeffrey070318增加：业务代码使用的光流世界系开关。
#define DELTA_MOTOR_NUM DELTA_R1_MOTOR_NUM                                             // 业务代码使用的击球电机数量
#define DELTA_MOTOR1_ID DELTA_R1_MOTOR1_ID                                             // 业务代码使用的击球电机1 ID
#define DELTA_MOTOR2_ID DELTA_R1_MOTOR2_ID                                             // 业务代码使用的击球电机2 ID
#define DELTA_MOTOR3_ID DELTA_R1_MOTOR3_ID                                             // 业务代码使用的击球电机3 ID
#define PITCH_MOTOR_ID PITCH_R1_MOTOR_ID                                               // 业务代码使用的pitch电机ID
#define SERVE_MOTOR_ID SERVE_R1_MOTOR_ID                                               // 业务代码使用的发球拨杆电机ID
#define DELTA_SPEED DELTA_R1_SPEED                                                     // 业务代码使用的击球机构速度
#define DELTA_POSITION_THRESHOLD DELTA_R1_POSITION_THRESHOLD                           // 业务代码使用的击球机构到位阈值
#define SERVE_POSITION_THRESHOLD SERVE_R1_POSITION_THRESHOLD                           // 业务代码使用的发球拨杆到位阈值
#define MIT_DELTA_HIT_KP MIT_DELTA_R1_HIT_KP                                           // 业务代码使用的delta击球P
#define MIT_DELTA_HIT_KD MIT_DELTA_R1_HIT_KD                                           // 业务代码使用的delta击球D
#define MIT_DELTA_HIT_TORQ MIT_DELTA_R1_HIT_TORQ                                       // 业务代码使用的delta击球前馈
#define MIT_DELTA_GET_KP MIT_DELTA_R1_GET_KP                                           // 业务代码使用的delta接球P
#define MIT_DELTA_GET_KD MIT_DELTA_R1_GET_KD                                           // 业务代码使用的delta接球D
#define MIT_DELTA_GET_TORQ MIT_DELTA_R1_GET_TORQ                                       // 业务代码使用的delta接球前馈
#define MIT_DELTA_SLOW_KP MIT_DELTA_R1_SLOW_KP                                         // 业务代码使用的delta慢速P
#define MIT_DELTA_SLOW_KD MIT_DELTA_R1_SLOW_KD                                         // 业务代码使用的delta慢速D
#define MIT_DELTA_SLOW_TORQ MIT_DELTA_R1_SLOW_TORQ                                     // 业务代码使用的delta慢速前馈
#define MIT_PITCH_HIT_KP MIT_PITCH_R1_HIT_KP                                           // 业务代码使用的pitch击球P
#define MIT_PITCH_HIT_KD MIT_PITCH_R1_HIT_KD                                           // 业务代码使用的pitch击球D
#define MIT_PITCH_HIT_TORQ MIT_PITCH_R1_HIT_TORQ                                       // 业务代码使用的pitch击球前馈
#define MIT_PITCH_GET_KP MIT_PITCH_R1_GET_KP                                           // 业务代码使用的pitch接球P
#define MIT_PITCH_GET_KD MIT_PITCH_R1_GET_KD                                           // 业务代码使用的pitch接球D
#define MIT_PITCH_GET_TORQ MIT_PITCH_R1_GET_TORQ                                       // 业务代码使用的pitch接球前馈
#define PITCH_POSITION_THRESHOLD PITCH_R1_POSITION_THRESHOLD                           // Jeffrey070318增加：业务代码使用的pitch到位阈值
#define PITCH_TEST_FRONT_POS PITCH_R1_TEST_FRONT_POS                                   // Jeffrey070318增加：业务代码使用的pitch前向测试位置
#define PITCH_TEST_BACK_POS PITCH_R1_TEST_BACK_POS                                     // Jeffrey070318增加：业务代码使用的pitch背向测试位置
#define PITCH_TEST_SPEED PITCH_R1_TEST_SPEED                                           // Jeffrey070318增加：业务代码使用的pitch测试速度
#define PITCH_REMOTE_ZERO_POS PITCH_R1_REMOTE_ZERO_POS                                 // Jeffrey070318增加：业务代码使用的pitch遥控零点目标
#define PITCH_REMOTE_FRONT_POS PITCH_R1_REMOTE_FRONT_POS                               // Jeffrey070318增加：业务代码使用的pitch遥控前向目标
#define PITCH_REMOTE_BACK_POS PITCH_R1_REMOTE_BACK_POS                                 // Jeffrey070318增加：业务代码使用的pitch遥控背向目标
#define PITCH_REMOTE_SPEED PITCH_R1_REMOTE_SPEED                                       // Jeffrey070318增加：业务代码使用的pitch遥控速度
#define PITCH_REMOTE_STICK_MAX PITCH_R1_REMOTE_STICK_MAX                               // Jeffrey070318增加：业务代码使用的pitch摇杆满量程
#define PITCH_STICK_DIRECTION PITCH_R1_STICK_DIRECTION                                 // Jeffrey070318增加：业务代码使用的pitch摇杆方向
#define PITCH_REMOTE_MODE PITCH_R1_REMOTE_MODE                                         // Jeffrey070318增加：业务代码使用的pitch摇杆映射模式
#define DELTA_ORIGINAL_TARGET_POS DELTA_R1_ORIGINAL_POS                                // 业务代码使用的击球机构初始位置
#define DELTA_HIT_1_TARGET_POS DELTA_R1_HIT_1_POS                                      // 业务代码使用的击球目标位置
#define DELTA_BACK_TARGET_POS DELTA_R1_BACK_POS                                        // 业务代码使用的回收目标位置
#define DELTA_TEST_DOWN_POS DELTA_R1_TEST_DOWN_POS                                     // 业务代码使用的测试下压位置
#define DELTA_TEST_BACK_POS DELTA_R1_TEST_BACK_POS                                     // 业务代码使用的测试回收位置
#define DELTA_TEST_TRIGGER_POS DELTA_R1_TEST_TRIGGER_POS                               // 业务代码使用的测试触发位置
#elif defined(ROBOT_R2)
#define ROBOT_HAS_SERVE 0                                                              // 当前车是否带发球拨杆
#define WHEEL_BASE CHASSIS_R2_WHEEL_BASE                                               // 业务代码使用的前后轮中心距
#define TRACK_WIDTH CHASSIS_R2_TRACK_WIDTH                                             // 业务代码使用的左右轮中心距
#define CENTER_GIMBAL_OFFSET_X CHASSIS_R2_CENTER_OFFSET_X                              // 业务代码使用的云台X偏移
#define CENTER_GIMBAL_OFFSET_Y CHASSIS_R2_CENTER_OFFSET_Y                              // 业务代码使用的云台Y偏移
#define RADIUS_WHEEL CHASSIS_R2_RADIUS_WHEEL                                           // 业务代码使用的轮子半径
#define REDUCTION_RATIO_WHEEL CHASSIS_R2_REDUCTION_RATIO_WHEEL                         // 业务代码使用的轮组减速比
#define CHASSIS_MOTOR_LF_ID CHASSIS_R2_MOTOR_LF_ID                                     // 业务代码使用的左前轮ID
#define CHASSIS_MOTOR_RF_ID CHASSIS_R2_MOTOR_RF_ID                                     // 业务代码使用的右前轮ID
#define CHASSIS_MOTOR_LB_ID CHASSIS_R2_MOTOR_LB_ID                                     // 业务代码使用的左后轮ID
#define CHASSIS_MOTOR_RB_ID CHASSIS_R2_MOTOR_RB_ID                                     // 业务代码使用的右后轮ID
#define CHASSIS_MOTOR_TYPE CHASSIS_R2_MOTOR_TYPE                                       // 业务代码使用的底盘电机型号
#define CHASSIS_MOTOR_LF_REVERSE CHASSIS_R2_MOTOR_LF_REVERSE                           // 业务代码使用的左前轮方向
#define CHASSIS_MOTOR_RF_REVERSE CHASSIS_R2_MOTOR_RF_REVERSE                           // 业务代码使用的右前轮方向
#define CHASSIS_MOTOR_LB_REVERSE CHASSIS_R2_MOTOR_LB_REVERSE                           // 业务代码使用的左后轮方向
#define CHASSIS_MOTOR_RB_REVERSE CHASSIS_R2_MOTOR_RB_REVERSE                           // 业务代码使用的右后轮方向
#define CHASSIS_SPEED_PID_KP CHASSIS_R2_SPEED_PID_KP                                   // 业务代码使用的速度环P
#define CHASSIS_SPEED_PID_KI CHASSIS_R2_SPEED_PID_KI                                   // 业务代码使用的速度环I
#define CHASSIS_SPEED_PID_KD CHASSIS_R2_SPEED_PID_KD                                   // 业务代码使用的速度环D
#define CHASSIS_SPEED_PID_INTEGRAL_LIMIT CHASSIS_R2_SPEED_PID_INTEGRAL_LIMIT           // 业务代码使用的速度环积分限幅
#define CHASSIS_SPEED_PID_MAX_OUT CHASSIS_R2_SPEED_PID_MAX_OUT                         // 业务代码使用的速度环输出限幅
#define CHASSIS_CURRENT_PID_KP CHASSIS_R2_CURRENT_PID_KP                               // 业务代码使用的电流环P
#define CHASSIS_CURRENT_PID_KI CHASSIS_R2_CURRENT_PID_KI                               // 业务代码使用的电流环I
#define CHASSIS_CURRENT_PID_KD CHASSIS_R2_CURRENT_PID_KD                               // 业务代码使用的电流环D
#define CHASSIS_CURRENT_PID_INTEGRAL_LIMIT CHASSIS_R2_CURRENT_PID_INTEGRAL_LIMIT       // 业务代码使用的电流环积分限幅
#define CHASSIS_CURRENT_PID_MAX_OUT CHASSIS_R2_CURRENT_PID_MAX_OUT                     // 业务代码使用的电流环输出限幅
#define CHASSIS_HEADING_PID_KP CHASSIS_R2_HEADING_PID_KP                               // 业务代码使用的车头角度环P
#define CHASSIS_HEADING_PID_KI CHASSIS_R2_HEADING_PID_KI                               // 业务代码使用的车头角度环I
#define CHASSIS_HEADING_PID_KD CHASSIS_R2_HEADING_PID_KD                               // 业务代码使用的车头角度环D
#define CHASSIS_HEADING_PID_MAX_OUT CHASSIS_R2_HEADING_PID_MAX_OUT                     // 业务代码使用的车头角度环输出限幅
#define CHASSIS_HEADING_PID_DEADBAND CHASSIS_R2_HEADING_PID_DEADBAND                   // 业务代码使用的车头角度死区
#define CHASSIS_HEADING_PID_INTEGRAL_LIMIT CHASSIS_R2_HEADING_PID_INTEGRAL_LIMIT       // 业务代码使用的车头角度环积分限幅
#define CHASSIS_KEEP_FRONT_STATIC_WZ_DEADBAND CHASSIS_R2_KEEP_FRONT_STATIC_WZ_DEADBAND // 业务代码使用的静止保持wz防抖阈值
#define CHASSIS_ROTATE_WZ CHASSIS_R2_ROTATE_WZ                                         // 业务代码使用的小陀螺角速度
#define CHASSIS_VX_DIRECTION CHASSIS_R2_VX_DIRECTION                                   // 业务代码使用的底盘X方向符号
#define CHASSIS_VY_DIRECTION CHASSIS_R2_VY_DIRECTION                                   // 业务代码使用的底盘Y方向符号
#define NAV_MAX_SPEED NAV_R2_MAX_SPEED                                                 // 业务代码使用的导航速度上限
#define NAV_SPEED_GAIN NAV_R2_SPEED_GAIN                                               // 业务代码使用的导航速度比例
#define NAV_ARRIVAL_DIST NAV_R2_ARRIVAL_DIST                                           // 业务代码使用的导航到点距离
#define RIGHT2_FIXED_MOVE_TEST_Y R2_RIGHT2_FIXED_MOVE_TEST_Y                           // 业务代码使用的右二固定距离测试Y位移
#define RIGHT2_FIXED_MOVE_TEST_DEADBAND R2_RIGHT2_FIXED_MOVE_TEST_DEADBAND             // 业务代码使用的右二固定距离测试米制PID死区
#define CMD_REMOTE_MOVE_SCALE CMD_R2_REMOTE_MOVE_SCALE                                 // 业务代码使用的遥控平移比例
#define CMD_REMOTE_YAW_SCALE CMD_R2_REMOTE_YAW_SCALE                                   // 业务代码使用的遥控旋转比例
#define CMD_REMOTE_YAW_MAX_WZ CMD_R2_REMOTE_YAW_MAX_WZ                                 // 业务代码使用的遥控旋转满杆输出上限
#define CMD_REMOTE_YAW_STICK_MAX CMD_R2_REMOTE_YAW_STICK_MAX                           // 业务代码使用的遥控旋转摇杆满量程
#define CMD_REMOTE_DEADBAND CMD_R2_REMOTE_DEADBAND                                     // 业务代码使用的遥控摇杆死区
#define CMD_REMOTE_STOP_DIAL_THRESHOLD CMD_R2_REMOTE_STOP_DIAL_THRESHOLD               // 业务代码使用的拨轮急停阈值
#define VISION_MODE VISION_R2_MODE                                                     // Jeffrey070318增加：业务代码使用的视觉回传模式。
#define VISION_PID_X_KP VISION_R2_PID_X_KP                                             // Jeffrey070318增加：业务代码使用的视觉X轴P。
#define VISION_PID_X_KI VISION_R2_PID_X_KI                                             // Jeffrey070318增加：业务代码使用的视觉X轴I。
#define VISION_PID_X_KD VISION_R2_PID_X_KD                                             // Jeffrey070318增加：业务代码使用的视觉X轴D。
#define VISION_PID_X_MAXOUT VISION_R2_PID_X_MAXOUT                                     // Jeffrey070318增加：业务代码使用的视觉X轴输出限幅。
#define VISION_PID_Y_KP VISION_R2_PID_Y_KP                                             // Jeffrey070318增加：业务代码使用的视觉Y轴P。
#define VISION_PID_Y_KI VISION_R2_PID_Y_KI                                             // Jeffrey070318增加：业务代码使用的视觉Y轴I。
#define VISION_PID_Y_KD VISION_R2_PID_Y_KD                                             // Jeffrey070318增加：业务代码使用的视觉Y轴D。
#define VISION_PID_Y_MAXOUT VISION_R2_PID_Y_MAXOUT                                     // Jeffrey070318增加：业务代码使用的视觉Y轴输出限幅。
#define VISION_PID_DEADBAND VISION_R2_PID_DEADBAND                                     // Jeffrey070318增加：业务代码使用的视觉PID死区。
#define VISION_PID_INTEGRAL_RATIO VISION_R2_PID_INTEGRAL_RATIO                         // Jeffrey070318增加：业务代码使用的视觉PID积分限幅比例。
#define VISION_SERVE_TRIGGER_DELAY_MS VISION_R2_SERVE_TRIGGER_DELAY_MS                 // Jeffrey070318增加：业务代码使用的视觉触发机械臂延迟。
#define OPTICAL_FLOW_PROTOCOL OPTICAL_FLOW_R2_PROTOCOL                                 // Jeffrey070318增加：业务代码使用的光流协议。
#define OPTICAL_FLOW_SCALE_X OPTICAL_FLOW_R2_SCALE_X                                   // Jeffrey070318增加：业务代码使用的光流X轴缩放。
#define OPTICAL_FLOW_SCALE_Y OPTICAL_FLOW_R2_SCALE_Y                                   // Jeffrey070318增加：业务代码使用的光流Y轴缩放。
#define OPTICAL_FLOW_SWAP_XY OPTICAL_FLOW_R2_SWAP_XY                                   // Jeffrey070318增加：业务代码使用的光流X/Y交换开关。
#define OPTICAL_FLOW_X_DIRECTION OPTICAL_FLOW_R2_X_DIRECTION                           // Jeffrey070318增加：业务代码使用的光流X方向符号。
#define OPTICAL_FLOW_Y_DIRECTION OPTICAL_FLOW_R2_Y_DIRECTION                           // Jeffrey070318增加：业务代码使用的光流Y方向符号。
#define OPTICAL_FLOW_ENABLE_GLOBAL_FRAME OPTICAL_FLOW_R2_ENABLE_GLOBAL_FRAME           // Jeffrey070318增加：业务代码使用的光流世界系开关。
#define DELTA_MOTOR_NUM DELTA_R2_MOTOR_NUM                                             // 业务代码使用的击球电机数量
#define DELTA_MOTOR1_ID DELTA_R2_MOTOR1_ID                                             // 业务代码使用的击球电机1 ID
#define DELTA_MOTOR2_ID DELTA_R2_MOTOR2_ID                                             // 业务代码使用的击球电机2 ID
#define DELTA_MOTOR3_ID DELTA_R2_MOTOR3_ID                                             // 业务代码预留的击球电机3 ID
#define PITCH_MOTOR_ID PITCH_R2_MOTOR_ID                                               // 业务代码使用的pitch电机ID
#define DELTA_SPEED DELTA_R2_SPEED                                                     // 业务代码使用的击球机构速度
#define DELTA_POSITION_THRESHOLD DELTA_R2_POSITION_THRESHOLD                           // 业务代码使用的击球机构到位阈值
#define MIT_DELTA_HIT_KP MIT_DELTA_R2_HIT_KP                                           // 业务代码使用的delta击球P
#define MIT_DELTA_HIT_KD MIT_DELTA_R2_HIT_KD                                           // 业务代码使用的delta击球D
#define MIT_DELTA_HIT_TORQ MIT_DELTA_R2_HIT_TORQ                                       // 业务代码使用的delta击球前馈
#define MIT_DELTA_GET_KP MIT_DELTA_R2_GET_KP                                           // 业务代码使用的delta接球P
#define MIT_DELTA_GET_KD MIT_DELTA_R2_GET_KD                                           // 业务代码使用的delta接球D
#define MIT_DELTA_GET_TORQ MIT_DELTA_R2_GET_TORQ                                       // 业务代码使用的delta接球前馈
#define MIT_DELTA_SLOW_KP MIT_DELTA_R2_SLOW_KP                                         // 业务代码使用的delta慢速P
#define MIT_DELTA_SLOW_KD MIT_DELTA_R2_SLOW_KD                                         // 业务代码使用的delta慢速D
#define MIT_DELTA_SLOW_TORQ MIT_DELTA_R2_SLOW_TORQ                                     // 业务代码使用的delta慢速前馈
#define MIT_PITCH_HIT_KP MIT_PITCH_R2_HIT_KP                                           // 业务代码使用的pitch击球P
#define MIT_PITCH_HIT_KD MIT_PITCH_R2_HIT_KD                                           // 业务代码使用的pitch击球D
#define MIT_PITCH_HIT_TORQ MIT_PITCH_R2_HIT_TORQ                                       // 业务代码使用的pitch击球前馈
#define MIT_PITCH_GET_KP MIT_PITCH_R2_GET_KP                                           // 业务代码使用的pitch接球P
#define MIT_PITCH_GET_KD MIT_PITCH_R2_GET_KD                                           // 业务代码使用的pitch接球D
#define MIT_PITCH_GET_TORQ MIT_PITCH_R2_GET_TORQ                                       // 业务代码使用的pitch接球前馈
#define PITCH_POSITION_THRESHOLD PITCH_R2_POSITION_THRESHOLD                           // Jeffrey070318增加：业务代码使用的pitch到位阈值
#define PITCH_TEST_FRONT_POS PITCH_R2_TEST_FRONT_POS                                   // Jeffrey070318增加：业务代码使用的pitch前向测试位置
#define PITCH_TEST_BACK_POS PITCH_R2_TEST_BACK_POS                                     // Jeffrey070318增加：业务代码使用的pitch背向测试位置
#define PITCH_TEST_SPEED PITCH_R2_TEST_SPEED                                           // Jeffrey070318增加：业务代码使用的pitch测试速度
#define PITCH_REMOTE_ZERO_POS PITCH_R2_REMOTE_ZERO_POS                                 // Jeffrey070318增加：业务代码使用的pitch遥控零点目标
#define PITCH_REMOTE_FRONT_POS PITCH_R2_REMOTE_FRONT_POS                               // Jeffrey070318增加：业务代码使用的pitch遥控前向目标
#define PITCH_REMOTE_BACK_POS PITCH_R2_REMOTE_BACK_POS                                 // Jeffrey070318增加：业务代码使用的pitch遥控背向目标
#define PITCH_REMOTE_SPEED PITCH_R2_REMOTE_SPEED                                       // Jeffrey070318增加：业务代码使用的pitch遥控速度
#define PITCH_REMOTE_STICK_MAX PITCH_R2_REMOTE_STICK_MAX                               // Jeffrey070318增加：业务代码使用的pitch摇杆满量程
#define PITCH_STICK_DIRECTION PITCH_R2_STICK_DIRECTION                                 // Jeffrey070318增加：业务代码使用的pitch摇杆方向
#define PITCH_REMOTE_MODE PITCH_R2_REMOTE_MODE                                         // Jeffrey070318增加：业务代码使用的pitch摇杆映射模式
#define DELTA_ORIGINAL_TARGET_POS DELTA_R2_ORIGINAL_POS                                // 业务代码使用的击球机构初始位置
#define DELTA_HIT_1_TARGET_POS DELTA_R2_HIT_1_POS                                      // 业务代码使用的击球目标位置
#define DELTA_BACK_TARGET_POS DELTA_R2_BACK_POS                                        // 业务代码使用的回收目标位置
#define DELTA_TEST_DOWN_POS DELTA_R2_TEST_DOWN_POS                                     // 业务代码使用的测试下压位置
#define DELTA_TEST_BACK_POS DELTA_R2_TEST_BACK_POS                                     // 业务代码使用的测试回收位置
#define DELTA_TEST_TRIGGER_POS DELTA_R2_TEST_TRIGGER_POS                               // 业务代码使用的测试触发位置
#else
#error Robot type undefined! Define ROBOT_R1 or ROBOT_R2 in robot_def.h.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */

/*YYP0417新增*/
// 发球杆状态枚举定义
typedef enum
{
    LAUNCHER_ORIGIN = 0, // 发球杆零位
    LAUNCHER_HIT = 1,    // 发球杆移动至指定位置
    LAUNCHER_STOP = 2    // 发球杆电机急停
} LauncherStatus_TypeDef;
#if ROBOT_HAS_SERVE
// Jeffrey070318修改：发球杆全局状态只在R1存在，R2不暴露launcher语义。
extern LauncherStatus_TypeDef g_launcher_status; // 发球杆状态(全局变量)
#endif

// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// Jeffrey070318增加：从general_def.h迁入Delta/Serve状态和消息定义，统一由robot_def.h管理app层语义。
typedef enum
{
    DELTA_INIT,
    DELTA_ORIGINAL_POS,
    DELTA_SLOW_TO_TARGET, // 缓慢从0运动到-1.2
    DELTA_SERVE_HIT_1,
    DELTA_SERVE_BACK_1,
    DELTA_SERVE_HIT_2,
    DELTA_SERVE_BACK_2,
    DELTA_STOP,
    GET_BALL,
} Delta_State_t;

typedef enum
{
    SERVE_INIT,
    SERVE_ORIGINAL_POS,
    SERVE_READY,
    SERVE_HIT,
    SERVE_BACK,
    SERVE_STOP,
} Serve_State_t;

typedef enum
{
    DELTA_STOP_ACT,
    DELTA_SERVE,
    DELTA_HIT,
    DELTA_READY,
} Delta_Action_e;

typedef struct
{
    Delta_Action_e delta_action;
    float pitch_target_pos; // Jeffrey070318增加：CMD下发给pitch的位置目标，左摇杆上下控制。
    float pitch_speed;      // Jeffrey070318增加：CMD下发给pitch的位置速度模式速度。
    uint8_t test_seq;       // [测试] cmd 自增计数器, 验证 cmd->delta 链路
} Delta_Ctrl_Cmd_s;

typedef struct
{
    uint8_t delta_feedback;
    uint8_t test_seq; // [测试] delta 自增计数器, 验证 delta->cmd 链路
} Delta_Upload_Data_s;

typedef enum
{
    SERVE_HIT_ACT,
    SERVE_BACK_ACT,
    SERVE_READY_ACT,
} Serve_Action_e;

typedef struct
{
    Serve_State_t serve_state;
    uint8_t test_seq; // [测试] delta 自增计数器(透传), 验证 delta->serve 链路
} Serve_Ctrl_Cmd_s;

typedef struct
{
    uint8_t serve_feedback;
    uint8_t test_seq; // [测试] serve 自增计数器, 验证 serve->delta 链路
} Serve_Upload_Data_s;

typedef enum
{
    ACTION_ORIGINAL, // 发球杆零位
    ACTION_GO,       // 发球杆移动至指定位置
    MOTOR_DISABLE,   // 发球杆电机急停
} RemoteStatus_TypeDef;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * YYP0417修改
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 允许全向平移及底盘自由旋转（手动接发球时使用）
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随云台yaw方向(废弃，仅保留作兼容)
    CHASSIS_KEEP_FRONT,        // 底盘叠加角度环控制，使底盘始终保持当前角度，仅允许全向平移（自动接球时使用）
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat; // 剩余枪口热量

} Chassis_Upload_Data_s;

typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
