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
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

//Jeffrey070318增加：整车类型条件编译开关，R1为三击球电机+发球拨杆，R2为双击球电机且无发球拨杆。
#define ROBOT_R1
// #define ROBOT_R2

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
//#define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
//Jeffrey070318修改：底盘应用共用，但机械/物理参数按R1/R2完整隔离，后续两车独立实测调参。
#define CHASSIS_R1_WHEEL_BASE 400.0f
#define CHASSIS_R1_TRACK_WIDTH 400.0f
#define CHASSIS_R1_CENTER_OFFSET_X 0.0f
#define CHASSIS_R1_CENTER_OFFSET_Y 0.0f
#define CHASSIS_R1_RADIUS_WHEEL 60.0f
#define CHASSIS_R1_REDUCTION_RATIO_WHEEL 19.0f
#define CHASSIS_R1_MOTOR_LF_ID 1u
#define CHASSIS_R1_MOTOR_RF_ID 2u
#define CHASSIS_R1_MOTOR_LB_ID 4u
#define CHASSIS_R1_MOTOR_RB_ID 3u
#define CHASSIS_R1_MOTOR_TYPE M3508
#define CHASSIS_R1_MOTOR_LF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R1_MOTOR_RF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R1_MOTOR_LB_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R1_MOTOR_RB_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R1_SPEED_PID_KP 10.0f
#define CHASSIS_R1_SPEED_PID_KI 0.0f
#define CHASSIS_R1_SPEED_PID_KD 0.0f
#define CHASSIS_R1_SPEED_PID_INTEGRAL_LIMIT 3000.0f
#define CHASSIS_R1_SPEED_PID_MAX_OUT 12000.0f
#define CHASSIS_R1_CURRENT_PID_KP 0.5f
#define CHASSIS_R1_CURRENT_PID_KI 0.0f
#define CHASSIS_R1_CURRENT_PID_KD 0.0f
#define CHASSIS_R1_CURRENT_PID_INTEGRAL_LIMIT 3000.0f
#define CHASSIS_R1_CURRENT_PID_MAX_OUT 15000.0f
#define CHASSIS_R1_HEADING_PID_KP 70.0f
#define CHASSIS_R1_HEADING_PID_KI 1.5f
#define CHASSIS_R1_HEADING_PID_KD 120.0f
#define CHASSIS_R1_HEADING_PID_MAX_OUT 2500.0f
#define CHASSIS_R1_HEADING_PID_DEADBAND 0.3f
#define CHASSIS_R1_HEADING_PID_INTEGRAL_LIMIT 400.0f
#define CHASSIS_R1_ROTATE_WZ 2000.0f
#define NAV_R1_MAX_SPEED 10000.0f
#define NAV_R1_SPEED_GAIN 6000.0f
#define NAV_R1_ARRIVAL_DIST 0.15f

#define CHASSIS_R2_WHEEL_BASE 400.0f
#define CHASSIS_R2_TRACK_WIDTH 400.0f
#define CHASSIS_R2_CENTER_OFFSET_X 0.0f
#define CHASSIS_R2_CENTER_OFFSET_Y 0.0f
#define CHASSIS_R2_RADIUS_WHEEL 60.0f
#define CHASSIS_R2_REDUCTION_RATIO_WHEEL 19.0f
#define CHASSIS_R2_MOTOR_LF_ID 1u
#define CHASSIS_R2_MOTOR_RF_ID 2u
#define CHASSIS_R2_MOTOR_LB_ID 4u
#define CHASSIS_R2_MOTOR_RB_ID 3u
#define CHASSIS_R2_MOTOR_TYPE M3508
#define CHASSIS_R2_MOTOR_LF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R2_MOTOR_RF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R2_MOTOR_LB_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R2_MOTOR_RB_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_R2_SPEED_PID_KP 10.0f
#define CHASSIS_R2_SPEED_PID_KI 0.0f
#define CHASSIS_R2_SPEED_PID_KD 0.0f
#define CHASSIS_R2_SPEED_PID_INTEGRAL_LIMIT 3000.0f
#define CHASSIS_R2_SPEED_PID_MAX_OUT 12000.0f
#define CHASSIS_R2_CURRENT_PID_KP 0.5f
#define CHASSIS_R2_CURRENT_PID_KI 0.0f
#define CHASSIS_R2_CURRENT_PID_KD 0.0f
#define CHASSIS_R2_CURRENT_PID_INTEGRAL_LIMIT 3000.0f
#define CHASSIS_R2_CURRENT_PID_MAX_OUT 15000.0f
#define CHASSIS_R2_HEADING_PID_KP 70.0f
#define CHASSIS_R2_HEADING_PID_KI 1.5f
#define CHASSIS_R2_HEADING_PID_KD 120.0f
#define CHASSIS_R2_HEADING_PID_MAX_OUT 2500.0f
#define CHASSIS_R2_HEADING_PID_DEADBAND 0.3f
#define CHASSIS_R2_HEADING_PID_INTEGRAL_LIMIT 400.0f
#define CHASSIS_R2_ROTATE_WZ 2000.0f
#define NAV_R2_MAX_SPEED 10000.0f
#define NAV_R2_SPEED_GAIN 6000.0f
#define NAV_R2_ARRIVAL_DIST 0.15f

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

//Jeffrey070318增加：R1/R2机械臂电机ID、速度和阈值完整隔离，后续按实车独立调参。
#define DELTA_R1_MOTOR_NUM 3u
#define DELTA_R1_MOTOR1_ID 1u
#define DELTA_R1_MOTOR2_ID 2u
#define DELTA_R1_MOTOR3_ID 3u
#define PITCH_R1_MOTOR_ID 4u
#define SERVE_R1_MOTOR_ID 6u
#define DELTA_R1_SPEED 16.0f
#define DELTA_R1_POSITION_THRESHOLD 0.15f
#define SERVE_R1_POSITION_THRESHOLD 0.15f

#define DELTA_R2_MOTOR_NUM 2u
#define DELTA_R2_MOTOR1_ID 1u
#define DELTA_R2_MOTOR2_ID 2u
#define DELTA_R2_MOTOR3_ID 3u
#define PITCH_R2_MOTOR_ID 4u
#define DELTA_R2_SPEED 16.0f
#define DELTA_R2_POSITION_THRESHOLD 0.15f

//Jeffrey070318修改：当前编译使用的机械臂参数仅从R1/R2参数区映射，不再保留跨车共用值。
#if defined(ROBOT_R1)
#define WHEEL_BASE CHASSIS_R1_WHEEL_BASE
#define TRACK_WIDTH CHASSIS_R1_TRACK_WIDTH
#define CENTER_GIMBAL_OFFSET_X CHASSIS_R1_CENTER_OFFSET_X
#define CENTER_GIMBAL_OFFSET_Y CHASSIS_R1_CENTER_OFFSET_Y
#define RADIUS_WHEEL CHASSIS_R1_RADIUS_WHEEL
#define REDUCTION_RATIO_WHEEL CHASSIS_R1_REDUCTION_RATIO_WHEEL
#define CHASSIS_MOTOR_LF_ID CHASSIS_R1_MOTOR_LF_ID
#define CHASSIS_MOTOR_RF_ID CHASSIS_R1_MOTOR_RF_ID
#define CHASSIS_MOTOR_LB_ID CHASSIS_R1_MOTOR_LB_ID
#define CHASSIS_MOTOR_RB_ID CHASSIS_R1_MOTOR_RB_ID
#define CHASSIS_MOTOR_TYPE CHASSIS_R1_MOTOR_TYPE
#define CHASSIS_MOTOR_LF_REVERSE CHASSIS_R1_MOTOR_LF_REVERSE
#define CHASSIS_MOTOR_RF_REVERSE CHASSIS_R1_MOTOR_RF_REVERSE
#define CHASSIS_MOTOR_LB_REVERSE CHASSIS_R1_MOTOR_LB_REVERSE
#define CHASSIS_MOTOR_RB_REVERSE CHASSIS_R1_MOTOR_RB_REVERSE
#define CHASSIS_SPEED_PID_KP CHASSIS_R1_SPEED_PID_KP
#define CHASSIS_SPEED_PID_KI CHASSIS_R1_SPEED_PID_KI
#define CHASSIS_SPEED_PID_KD CHASSIS_R1_SPEED_PID_KD
#define CHASSIS_SPEED_PID_INTEGRAL_LIMIT CHASSIS_R1_SPEED_PID_INTEGRAL_LIMIT
#define CHASSIS_SPEED_PID_MAX_OUT CHASSIS_R1_SPEED_PID_MAX_OUT
#define CHASSIS_CURRENT_PID_KP CHASSIS_R1_CURRENT_PID_KP
#define CHASSIS_CURRENT_PID_KI CHASSIS_R1_CURRENT_PID_KI
#define CHASSIS_CURRENT_PID_KD CHASSIS_R1_CURRENT_PID_KD
#define CHASSIS_CURRENT_PID_INTEGRAL_LIMIT CHASSIS_R1_CURRENT_PID_INTEGRAL_LIMIT
#define CHASSIS_CURRENT_PID_MAX_OUT CHASSIS_R1_CURRENT_PID_MAX_OUT
#define CHASSIS_HEADING_PID_KP CHASSIS_R1_HEADING_PID_KP
#define CHASSIS_HEADING_PID_KI CHASSIS_R1_HEADING_PID_KI
#define CHASSIS_HEADING_PID_KD CHASSIS_R1_HEADING_PID_KD
#define CHASSIS_HEADING_PID_MAX_OUT CHASSIS_R1_HEADING_PID_MAX_OUT
#define CHASSIS_HEADING_PID_DEADBAND CHASSIS_R1_HEADING_PID_DEADBAND
#define CHASSIS_HEADING_PID_INTEGRAL_LIMIT CHASSIS_R1_HEADING_PID_INTEGRAL_LIMIT
#define CHASSIS_ROTATE_WZ CHASSIS_R1_ROTATE_WZ
#define NAV_MAX_SPEED NAV_R1_MAX_SPEED
#define NAV_SPEED_GAIN NAV_R1_SPEED_GAIN
#define NAV_ARRIVAL_DIST NAV_R1_ARRIVAL_DIST
#define DELTA_MOTOR_NUM DELTA_R1_MOTOR_NUM
#define DELTA_MOTOR1_ID DELTA_R1_MOTOR1_ID
#define DELTA_MOTOR2_ID DELTA_R1_MOTOR2_ID
#define DELTA_MOTOR3_ID DELTA_R1_MOTOR3_ID
#define PITCH_MOTOR_ID PITCH_R1_MOTOR_ID
#define SERVE_MOTOR_ID SERVE_R1_MOTOR_ID
#define ROBOT_HAS_SERVE 1
#define DELTA_SPEED DELTA_R1_SPEED
#define DELTA_POSITION_THRESHOLD DELTA_R1_POSITION_THRESHOLD
#define SERVE_POSITION_THRESHOLD SERVE_R1_POSITION_THRESHOLD
#elif defined(ROBOT_R2)
#define WHEEL_BASE CHASSIS_R2_WHEEL_BASE
#define TRACK_WIDTH CHASSIS_R2_TRACK_WIDTH
#define CENTER_GIMBAL_OFFSET_X CHASSIS_R2_CENTER_OFFSET_X
#define CENTER_GIMBAL_OFFSET_Y CHASSIS_R2_CENTER_OFFSET_Y
#define RADIUS_WHEEL CHASSIS_R2_RADIUS_WHEEL
#define REDUCTION_RATIO_WHEEL CHASSIS_R2_REDUCTION_RATIO_WHEEL
#define CHASSIS_MOTOR_LF_ID CHASSIS_R2_MOTOR_LF_ID
#define CHASSIS_MOTOR_RF_ID CHASSIS_R2_MOTOR_RF_ID
#define CHASSIS_MOTOR_LB_ID CHASSIS_R2_MOTOR_LB_ID
#define CHASSIS_MOTOR_RB_ID CHASSIS_R2_MOTOR_RB_ID
#define CHASSIS_MOTOR_TYPE CHASSIS_R2_MOTOR_TYPE
#define CHASSIS_MOTOR_LF_REVERSE CHASSIS_R2_MOTOR_LF_REVERSE
#define CHASSIS_MOTOR_RF_REVERSE CHASSIS_R2_MOTOR_RF_REVERSE
#define CHASSIS_MOTOR_LB_REVERSE CHASSIS_R2_MOTOR_LB_REVERSE
#define CHASSIS_MOTOR_RB_REVERSE CHASSIS_R2_MOTOR_RB_REVERSE
#define CHASSIS_SPEED_PID_KP CHASSIS_R2_SPEED_PID_KP
#define CHASSIS_SPEED_PID_KI CHASSIS_R2_SPEED_PID_KI
#define CHASSIS_SPEED_PID_KD CHASSIS_R2_SPEED_PID_KD
#define CHASSIS_SPEED_PID_INTEGRAL_LIMIT CHASSIS_R2_SPEED_PID_INTEGRAL_LIMIT
#define CHASSIS_SPEED_PID_MAX_OUT CHASSIS_R2_SPEED_PID_MAX_OUT
#define CHASSIS_CURRENT_PID_KP CHASSIS_R2_CURRENT_PID_KP
#define CHASSIS_CURRENT_PID_KI CHASSIS_R2_CURRENT_PID_KI
#define CHASSIS_CURRENT_PID_KD CHASSIS_R2_CURRENT_PID_KD
#define CHASSIS_CURRENT_PID_INTEGRAL_LIMIT CHASSIS_R2_CURRENT_PID_INTEGRAL_LIMIT
#define CHASSIS_CURRENT_PID_MAX_OUT CHASSIS_R2_CURRENT_PID_MAX_OUT
#define CHASSIS_HEADING_PID_KP CHASSIS_R2_HEADING_PID_KP
#define CHASSIS_HEADING_PID_KI CHASSIS_R2_HEADING_PID_KI
#define CHASSIS_HEADING_PID_KD CHASSIS_R2_HEADING_PID_KD
#define CHASSIS_HEADING_PID_MAX_OUT CHASSIS_R2_HEADING_PID_MAX_OUT
#define CHASSIS_HEADING_PID_DEADBAND CHASSIS_R2_HEADING_PID_DEADBAND
#define CHASSIS_HEADING_PID_INTEGRAL_LIMIT CHASSIS_R2_HEADING_PID_INTEGRAL_LIMIT
#define CHASSIS_ROTATE_WZ CHASSIS_R2_ROTATE_WZ
#define NAV_MAX_SPEED NAV_R2_MAX_SPEED
#define NAV_SPEED_GAIN NAV_R2_SPEED_GAIN
#define NAV_ARRIVAL_DIST NAV_R2_ARRIVAL_DIST
#define DELTA_MOTOR_NUM DELTA_R2_MOTOR_NUM
#define DELTA_MOTOR1_ID DELTA_R2_MOTOR1_ID
#define DELTA_MOTOR2_ID DELTA_R2_MOTOR2_ID
#define DELTA_MOTOR3_ID DELTA_R2_MOTOR3_ID
#define PITCH_MOTOR_ID PITCH_R2_MOTOR_ID
//Jeffrey070318修改：R2不运行Serve封装，因此不定义Serve电机参数，避免无用配置干扰阅读。
#define ROBOT_HAS_SERVE 0
#define DELTA_SPEED DELTA_R2_SPEED
#define DELTA_POSITION_THRESHOLD DELTA_R2_POSITION_THRESHOLD
#else
#error Robot type undefined! Define ROBOT_R1 or ROBOT_R2 in robot_def.h.
#endif

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

//Jeffrey070318增加：检查R1/R2定义冲突，确保同一次编译只生成一种整车固件。
#if defined(ROBOT_R1) && defined(ROBOT_R2)
#error Conflict robot type definition! You can only define ROBOT_R1 or ROBOT_R2.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */

/*YYP0417新增*/
//发球杆状态枚举定义
typedef enum
{
    LAUNCHER_ORIGIN   = 0,  // 发球杆零位
    LAUNCHER_HIT  = 1,   // 发球杆移动至指定位置
    LAUNCHER_STOP   = 2    // 发球杆电机急停
} LauncherStatus_TypeDef;
extern LauncherStatus_TypeDef g_launcher_status; // 发球杆状态(全局变量)

// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

//Jeffrey070318增加：从general_def.h迁入Delta/Serve状态和消息定义，统一由robot_def.h管理app层语义。
typedef enum
{
    DELTA_INIT,
    DELTA_ORIGINAL_POS,
    DELTA_SLOW_TO_TARGET,   // 缓慢从0运动到-1.2
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
    uint8_t test_seq;       // [测试] cmd 自增计数器, 验证 cmd->delta 链路
} Delta_Ctrl_Cmd_s;

typedef struct
{
    uint8_t delta_feedback;
    uint8_t test_seq;       // [测试] delta 自增计数器, 验证 delta->cmd 链路
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
    uint8_t test_seq;       // [测试] delta 自增计数器(透传), 验证 delta->serve 链路
} Serve_Ctrl_Cmd_s;

typedef struct
{
    uint8_t serve_feedback;
    uint8_t test_seq;       // [测试] serve 自增计数器, 验证 serve->delta 链路
} Serve_Upload_Data_s;

typedef enum
{
    ACTION_ORIGINAL,  // 发球杆零位
    ACTION_GO,        // 发球杆移动至指定位置
    MOTOR_DISABLE,    // 发球杆电机急停
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
    CHASSIS_KEEP_FRONT, // 底盘叠加角度环控制，使底盘始终保持当前角度，仅允许全向平移（自动接球时使用）
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

    uint8_t rest_heat;           // 剩余枪口热量


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
