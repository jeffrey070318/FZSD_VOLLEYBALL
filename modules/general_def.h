#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

// 一些module的通用数值型定义,注意条件macro兼容,一些宏可能在math.h中已经定义过了

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 (PI * 2.0f) // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

/**
 * @brief Delta状态机
 */
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

/**
 * @brief Serve状态机
 */
typedef enum
{
    SERVE_INIT,
    SERVE_ORIGINAL_POS,
    SERVE_READY,
    SERVE_HIT,
    SERVE_BACK,
    SERVE_STOP,
} Serve_State_t;

/**
 * @brief MIT模式控制数据结构体定义
 */
typedef struct
{
    float pos;
    float vel;
    float kp;
    float kd;
    float torq;
} MIT_CTRL_DATA;

#include "dmmotor.h"

/****************************** 未测试 */

/*delta的消息中心相关结构体*/
/**
 * @brief Delta动作枚举
 */
typedef enum
{
    DELTA_STOP_ACT,
    DELTA_SERVE,
    DELTA_HIT,
    DELTA_READY,
} Delta_Action_e;

/**
 * @brief cmd发布的delta控制数据，由delta订阅
 */
typedef struct
{
    Delta_Action_e delta_action;
    uint8_t test_seq;       // [测试] cmd 自增计数器, 验证 cmd→delta 链路
} Delta_Ctrl_Cmd_s;

/**
 * @brief delta发布的delta反馈数据，由cmd订阅
 */
typedef struct
{
    uint8_t delta_feedback;
    uint8_t test_seq;       // [测试] delta 自增计数器, 验证 delta→cmd 链路
} Delta_Upload_Data_s;

/*serve的消息中心结构体*/
/**
 * @brief Serve动作枚举
 */
typedef enum
{
    SERVE_HIT_ACT,
    SERVE_BACK_ACT,
    SERVE_READY_ACT,
} Serve_Action_e;

/**
 * @brief cmd发布的serve控制数据，由serve订阅
 */
typedef struct
{
    Serve_State_t serve_state;
    uint8_t test_seq;       // [测试] delta 自增计数器(透传), 验证 delta→serve 链路
} Serve_Ctrl_Cmd_s;

/**
 * @brief serve发布的serve反馈数据，由cmd订阅
 */
typedef struct
{
    uint8_t serve_feedback;
    uint8_t test_seq;       // [测试] serve 自增计数器, 验证 serve→delta 链路
} Serve_Upload_Data_s;

/**
 *  @brief 发球杆状态枚举定义 
 */
typedef enum
{
    ACTION_ORIGINAL,  // 发球杆零位
    ACTION_GO,   // 发球杆移动至指定位置
    MOTOR_DISABLE,    // 发球杆电机急停
} RemoteStatus_TypeDef;

/**
 * @brief 运行状态枚举定义
*/
typedef enum
{
    ROBOT_STOP,
    ROBOT_READY,
} Robot_Status_e;

#define DELTA_SPEED 16.0f
#define SERVE_SPEED 39.0f
#define DELTA_POSITION_THRESHOLD 0.15f
#define SERVE_POSITION_THRESHOLD 0.15f

#endif // !GENERAL_DEF_H