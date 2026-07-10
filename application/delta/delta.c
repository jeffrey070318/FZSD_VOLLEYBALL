#include <stdint.h>
#include <math.h>
#include "delta.h"

#include "cmsis_os.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
// Jeffrey070318修改：Delta状态机和消息结构迁移到robot_def.h，避免general_def.h承载app层语义。
#include "robot_def.h"
#include "remote_control.h"

// Jeffrey070318修改：Delta击球电机数量由ROBOT_R1/ROBOT_R2条件编译决定，R1=3个，R2=2个。
Joint_Motor_t Delta_motor[DELTA_MOTOR_NUM];
Joint_Motor_t Pitch_motor;

Delta_State_t Delta_State = DELTA_INIT;
Serve_State_t Serve_State = SERVE_INIT;
uint8_t Count_1 = 0;

// Jeffrey070318临时调试：R2机械臂调参Watch变量，用于判断任务是否运行、是否卡在等待电机反馈。
volatile uint32_t dbg_delta_task_loop_cnt = 0;
volatile uint32_t dbg_delta_state = 0;
volatile uint32_t dbg_delta_init_loop_cnt = 0;
volatile uint32_t dbg_delta_enable_send_cnt = 0;
volatile uint32_t dbg_delta_pitch_enable_send_cnt = 0;
volatile uint32_t dbg_delta_motors_enabled = 0;
volatile uint32_t dbg_delta_motor_state[DELTA_MOTOR_NUM] = {0};
volatile float dbg_delta_motor_pos[DELTA_MOTOR_NUM] = {0.0f};
volatile uint32_t dbg_pitch_motor_state = 0;
volatile float dbg_pitch_motor_pos = 0.0f;
volatile uint32_t dbg_pitch_motor_enabled = 0;

/* [测试] delta 自增计数器, 随每次任务循环 +1 */
static uint8_t delta_test_seq = 0;
// Jeffrey070318修改：恢复cmd→delta LiveWatch变量，验证遥控器开关命令是否进入Delta状态机。
static uint8_t dbg_cmd_action = DELTA_READY;
static uint8_t dbg_cmd_seq = 0;
// Jeffrey070318增加：Delta接收CMD链路LiveWatch变量，用于判断右开关动作是否到达Delta任务。
volatile uint32_t dbg_delta_cmd_msg_cnt = 0;
volatile uint8_t dbg_delta_recv_cmd_action = DELTA_READY;
volatile uint8_t dbg_delta_recv_cmd_seq = 0;
volatile float dbg_delta_recv_pitch_target_pos = 0.0f; // Jeffrey070318增加：记录Delta收到的pitch目标位置，确认CMD到Delta链路。
volatile float dbg_delta_recv_pitch_speed = 0.0f;      // Jeffrey070318增加：记录Delta收到的pitch速度，确认位置速度模式参数。
#if ROBOT_HAS_SERVE
// Jeffrey070318修改：serve调试变量只在R1存在，R2无发球拨杆时不编译这些未使用变量。
static uint8_t dbg_serve_state = 0;
static uint8_t dbg_serve_seq = 0;
#endif

RemoteStatus_TypeDef Remote_status = ACTION_ORIGINAL;

// Jeffrey070318修改：Delta/Pitch的R1/R2参数与统一宏映射已集中到robot_def.h，delta.c只消费当前车种宏。

static Publisher_t *delta_pub;                  // 用于发布底盘的数据
static Subscriber_t *delta_sub;                 // 用于订阅底盘的控制命令
static Delta_Ctrl_Cmd_s delta_cmd_recv;         // 底盘接收到的控制命令
static Delta_Upload_Data_s delta_feedback_data; // 底盘回传的反馈数据

#if ROBOT_HAS_SERVE
// Jeffrey070318修改：serve话题对象只在R1编译，避免R2无发球机构时产生无用链路。
static Publisher_t *serve_cmd_pub;
static Subscriber_t *serve_feed_sub;
static Serve_Ctrl_Cmd_s serve_cmd_send;
static Serve_Upload_Data_s serve_fetch_data;
#endif

MIT_CTRL_DATA DELTA_ORIGINAL_DATA = {
    .pos = DELTA_ORIGINAL_TARGET_POS,
    .vel = 0.0f,
    .kp = MIT_DELTA_SLOW_KP,
    .kd = MIT_DELTA_SLOW_KD,
    .torq = MIT_DELTA_SLOW_TORQ};

MIT_CTRL_DATA HIT_1_DATA = {
    .pos = DELTA_HIT_1_TARGET_POS,
    .vel = 0.0f,
    .kp = MIT_DELTA_HIT_KP,
    .kd = MIT_DELTA_HIT_KD,
    .torq = MIT_DELTA_HIT_TORQ};

MIT_CTRL_DATA BACK_DATA = {
    .pos = DELTA_BACK_TARGET_POS,
    .vel = 0.0f,
    .kp = MIT_DELTA_SLOW_KP,
    .kd = MIT_DELTA_SLOW_KD,
    .torq = MIT_DELTA_SLOW_TORQ};

// Jeffrey070318增加：集中初始化单个Delta电机，避免R1/R2数量变化时复制结构体初始化代码。
static void DeltaMotorInitOne(Joint_Motor_t *motor, uint16_t id)
{
    *motor = (Joint_Motor_t){
        .mode = MIT_MODE,
        .para = {
            .id = id,
            .state = DISABLE_STATE,
            .p_int = 0,
            .v_int = 0,
            .t_int = 0,
            .kp_int = 0,
            .kd_int = 0,
            .pos = 0.0f,
            .vel = 0.0f,
            .tor = 0.0f,
            .Kp = 0.0f,
            .Kd = 0.0f,
        },
    };
}

// Jeffrey070318增加：按当前车种的Delta电机数量批量使能。
static void DeltaEnableMotors(void)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        Enable_Motor_Mode(&hcan2, &Delta_motor[i]);
        dbg_delta_enable_send_cnt++;
        osDelay(100);
    }
}

// Jeffrey070318增加：DeltaApplyCmdAction提前使用使能状态判断，这里先声明后面实现的检查函数。
static uint8_t DeltaMotorsEnabled(void);

// Jeffrey070318增加：全车急停时批量失能Delta和pitch电机，避免继续执行上一次动作命令。
static void DeltaDisableMotors(void)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        Disable_Motor_Mode(&hcan2, Delta_motor[i].para.id, Delta_motor[i].mode);
    }
    Disable_Motor_Mode(&hcan2, Pitch_motor.para.id, Pitch_motor.mode);
}

// Jeffrey070318增加：把cmd层的DELTA_HIT/READY/STOP动作转换成Delta状态机目标状态。
static void DeltaApplyCmdAction(void)
{
    static Delta_Action_e last_action = DELTA_READY;
    Delta_Action_e action = delta_cmd_recv.delta_action;

    if (action == DELTA_STOP_ACT)
    {
        if (last_action != DELTA_STOP_ACT)
            DeltaDisableMotors();
        Delta_State = DELTA_STOP;
        last_action = action;
        return;
    }

    if (last_action == DELTA_STOP_ACT)
    {
        Delta_State = DELTA_INIT;
        last_action = action;
        return;
    }

    // Jeffrey070318增加：CMD刚启动或急停恢复后，电机未确认使能前必须先走DELTA_INIT，不能直接跳到动作状态。
    if (!DeltaMotorsEnabled() || Pitch_motor.para.state != ENABLE_STATE)
    {
        Delta_State = DELTA_INIT;
        last_action = action;
        return;
    }

    switch (action)
    {
    case DELTA_HIT:
    case DELTA_SERVE:
        Delta_State = DELTA_SERVE_HIT_1;
        break;
    case DELTA_READY:
    default:
        Delta_State = DELTA_ORIGINAL_POS;
        break;
    }

    last_action = action;
}

static void DeltaUpdateDebugSnapshot(void)
{
    dbg_delta_state = (uint32_t)Delta_State;
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        dbg_delta_motor_state[i] = (uint32_t)Delta_motor[i].para.state;
        dbg_delta_motor_pos[i] = Delta_motor[i].para.pos;
    }
    dbg_pitch_motor_state = (uint32_t)Pitch_motor.para.state;
    dbg_pitch_motor_pos = Pitch_motor.para.pos;
}

// Jeffrey070318增加：按当前车种判断所有Delta电机是否已使能。
static uint8_t DeltaMotorsEnabled(void)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        if (Delta_motor[i].para.state != ENABLE_STATE)
        {
            return 0;
        }
    }
    return 1;
}

// Jeffrey070318增加：按当前车种批量发送MIT控制，R2不会访问第三个电机。
static void DeltaMitCtrlAll(MIT_CTRL_DATA ctrl_data)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        Mit_Ctrl(&hcan2, &Delta_motor[i], ctrl_data);
    }
}

// Jeffrey070318增加：按当前车种判断所有Delta电机是否到达同一目标位置。
static uint8_t DeltaMotorsReached(float target_pos)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        if (fabsf(Delta_motor[i].para.pos - target_pos) >= DELTA_POSITION_THRESHOLD)
        {
            return 0;
        }
    }
    return 1;
}

// Jeffrey070318增加：Delta侧再次限制pitch目标，防止CMD异常值超过当前车种安全范围。
static float PitchConstrainTarget(float target_pos)
{
    float min_pos = PITCH_REMOTE_BACK_POS < PITCH_REMOTE_FRONT_POS ? PITCH_REMOTE_BACK_POS : PITCH_REMOTE_FRONT_POS;
    float max_pos = PITCH_REMOTE_BACK_POS > PITCH_REMOTE_FRONT_POS ? PITCH_REMOTE_BACK_POS : PITCH_REMOTE_FRONT_POS;

    if (target_pos < min_pos)
        return min_pos;
    if (target_pos > max_pos)
        return max_pos;
    return target_pos;
}

// Jeffrey070318增加：执行CMD下发的pitch目标，未收到速度时使用robot_def.h中的默认速度。
static void PitchApplyCmdTarget(void)
{
    float target_pos = PitchConstrainTarget(delta_cmd_recv.pitch_target_pos);
    float speed = (delta_cmd_recv.pitch_speed > 0.0f) ? delta_cmd_recv.pitch_speed : PITCH_REMOTE_SPEED;

    Pos_Speed_Ctrl(&hcan2, &Pitch_motor, target_pos, speed);
}

// Jeffrey070318增加：保留原Delta_Motion测试入口，同时兼容R1/R2不同电机数量。
static void DeltaPosSpeedAll(float pos, float vel)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        Pos_Speed_Ctrl(&hcan2, &Delta_motor[i], pos, vel);
    }
}

// Jeffrey070318增加：测试动作中需要判断所有Delta电机是否越过同一位置阈值。
static uint8_t DeltaMotorsBelow(float pos)
{
    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        if (Delta_motor[i].para.pos >= pos)
        {
            return 0;
        }
    }
    return 1;
}

// Jeffrey070318增加：按下标映射当前车种的Delta电机ID，R1/R2可独立改ID。
static uint16_t DeltaMotorIdByIndex(uint8_t index)
{
    switch (index)
    {
    case 0:
        return DELTA_MOTOR1_ID;
    case 1:
        return DELTA_MOTOR2_ID;
#if DELTA_MOTOR_NUM >= 3u
    case 2:
        return DELTA_MOTOR3_ID;
#endif
    default:
        return 0u;
    }
}

void DeltaInit(void)
{
#if ROBOT_HAS_SERVE
    serve_cmd_pub = PubRegister("serve_cmd", sizeof(Serve_Ctrl_Cmd_s));
    serve_feed_sub = SubRegister("serve_feed", sizeof(Serve_Upload_Data_s));
#endif

    for (uint8_t i = 0; i < DELTA_MOTOR_NUM; i++)
    {
        // Jeffrey070318修改：Delta电机ID来自R1/R2独立参数区，不再默认使用i+1。
        DeltaMotorInitOne(&Delta_motor[i], DeltaMotorIdByIndex(i));
    }

    Pitch_motor = (Joint_Motor_t){
        .mode = POS_MODE,
        .para = {
            // Jeffrey070318修改：Pitch电机ID来自R1/R2独立参数区。
            .id = PITCH_MOTOR_ID,
            .state = DISABLE_STATE,
            .p_int = 0,
            .v_int = 0,
            .t_int = 0,
            .kp_int = 0,
            .kd_int = 0,

            .pos = 0.0f,
            .vel = 0.0f,
            .tor = 0.0f,
            .Kp = 0.0f,
            .Kd = 0.0f,
        },
    };

    delta_sub = SubRegister("delta_cmd", sizeof(Delta_Ctrl_Cmd_s));
    delta_pub = PubRegister("delta_feed", sizeof(Delta_Upload_Data_s));

    // Delta_State = DELTA_INIT;
}

void DeltaTask()
{
    dbg_delta_task_loop_cnt++;
    DeltaUpdateDebugSnapshot();

    /* 接收 cmd 下发的 delta 控制命令 */
    if (SubGetMessage(delta_sub, (void *)&delta_cmd_recv))
    {
        // Jeffrey070318修改：收到cmd命令后立即更新Delta状态机，右开关下=向上，上/中=回零。
        dbg_cmd_action = (uint8_t)delta_cmd_recv.delta_action;
        dbg_cmd_seq = delta_cmd_recv.test_seq;
        // Jeffrey070318增加：记录Delta收到的CMD消息，排查CMD到Delta消息链路。
        dbg_delta_cmd_msg_cnt++;
        dbg_delta_recv_cmd_action = (uint8_t)delta_cmd_recv.delta_action;
        dbg_delta_recv_cmd_seq = delta_cmd_recv.test_seq;
        dbg_delta_recv_pitch_target_pos = delta_cmd_recv.pitch_target_pos;
        dbg_delta_recv_pitch_speed = delta_cmd_recv.pitch_speed;
        DeltaApplyCmdAction();
    }
#if ROBOT_HAS_SERVE
    // Jeffrey070318修改：R2没有发球拨杆，不订阅serve反馈。
    if (SubGetMessage(serve_feed_sub, (void *)&serve_fetch_data))
    {
        /* [测试] 存入 LiveWatch 变量, 验证 serve→delta 链路 */
        dbg_serve_state = serve_fetch_data.serve_feedback;
        dbg_serve_seq = serve_fetch_data.test_seq;
    }
#endif

    switch (Delta_State)
    {
    case DELTA_INIT:
        dbg_delta_init_loop_cnt++;
        // Jeffrey070318修改：Delta电机按R1/R2数量批量使能，避免R2访问第三个电机。
        DeltaEnableMotors();
        Enable_Motor_Mode(&hcan2, &Pitch_motor);
        dbg_delta_pitch_enable_send_cnt++;
        osDelay(100);

        dbg_delta_motors_enabled = DeltaMotorsEnabled();
        dbg_pitch_motor_enabled = (Pitch_motor.para.state == ENABLE_STATE);
        if (dbg_delta_motors_enabled && dbg_pitch_motor_enabled)
        {
            Delta_State = DELTA_ORIGINAL_POS;
            // Serve_State = SERVE_INIT;
        }
        break;

    case DELTA_ORIGINAL_POS:
        // 回归零位置
        // Jeffrey070318修改：原点控制按当前车种的Delta电机数量批量下发。
        DeltaMitCtrlAll(DELTA_ORIGINAL_DATA);
        // Jeffrey070318修改：整车遥控模式下pitch由CMD目标控制，左摇杆回中时目标就是机械零点。
        PitchApplyCmdTarget();
        // Pos_Speed_Ctrl(&hcan2, &Pitch_motor, -0.4f, 2.0f);

        // if(fabs(Delta_motor[0].para.pos) < DELTA_POSITION_THRESHOLD &&
        //    fabs(Delta_motor[1].para.pos) < DELTA_POSITION_THRESHOLD &&
        //    fabs(Delta_motor[2].para.pos) < DELTA_POSITION_THRESHOLD)
        // {
        //     //fabs(Pitch_motor.para.pos - (-0.4f)) < DELTA_POSITION_THRESHOLD
        //     osDelay(1000);
        //     Delta_State = DELTA_SERVE_HIT_1;//运行测试
        //     //Serve_State = SERVE_READY;
        // }
        break;

    case GET_BALL:
        // MIT 模式: 位置 + 高 Kp/Kd + 前馈力矩, 增大击球力度
        // Mit_Ctrl(&hcan2, Pitch_motor.para.id,      -0.4f, 0.0f, MIT_PITCH_GET_KP, MIT_PITCH_GET_KD, MIT_PITCH_GET_TORQ);
        // Mit_Ctrl(&hcan2, Delta_motor[0].para.id, Hit_1_Pos, 0.0f, MIT_DELTA_GET_KP, MIT_DELTA_GET_KD, MIT_DELTA_GET_TORQ);
        // Mit_Ctrl(&hcan2, Delta_motor[1].para.id, Hit_1_Pos, 0.0f, MIT_DELTA_GET_KP, MIT_DELTA_GET_KD, MIT_DELTA_GET_TORQ);
        // Mit_Ctrl(&hcan2, Delta_motor[2].para.id, Hit_1_Pos, 0.0f, MIT_DELTA_GET_KP, MIT_DELTA_GET_KD, MIT_DELTA_GET_TORQ);

        // Jeffrey070318修改：到位判断按当前车种的Delta电机数量执行，R2只检查两个电机。
        if (DeltaMotorsReached(DELTA_ORIGINAL_DATA.pos))
        {
            Delta_State = DELTA_SERVE_BACK_1;
        }

        break;

    case DELTA_SERVE_HIT_1:
        // MIT 模式: 击球阶段用最大 Kp 与额外前馈力矩
        // Jeffrey070318修改：击球控制按R1/R2数量批量下发，参数由车种宏选择。
        DeltaMitCtrlAll(HIT_1_DATA);
        // Jeffrey070318增加：机械臂发出时pitch仍响应左摇杆目标，便于接球姿态联调。
        PitchApplyCmdTarget();

        if (DeltaMotorsReached(HIT_1_DATA.pos))
        {
            // Delta_State = DELTA_SERVE_BACK_1;
            // 击球杆下压: 高 Kp + 大前馈力矩
            // Mit_Ctrl(&hcan2, Pitch_motor.para.id, -0.8f, 0.0f, MIT_PITCH_HIT_KP, MIT_PITCH_HIT_KD, MIT_PITCH_HIT_TORQ);
            // Serve_State = SERVE_HIT;
        }

        break;

    case DELTA_SERVE_BACK_1:

        // Jeffrey070318修改：回收控制按R1/R2数量批量下发。
        DeltaMitCtrlAll(BACK_DATA);

        if (DeltaMotorsReached(BACK_DATA.pos))
        {
            Delta_State = DELTA_STOP;
            // Pos_Speed_Ctrl(&hcan2, &Pitch_motor, 0.0f, 6.0f);
        }
        break;

        // case DELTA_SERVE_HIT_2:
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[0], Hit_2_Pos, DELTA_SPEED);
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[1], Hit_2_Pos, DELTA_SPEED);
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[2], Hit_2_Pos, DELTA_SPEED);

        //     if(fabs(Delta_motor[0].para.pos - Hit_2_Pos) < DELTA_POSITION_THRESHOLD &&
        //        fabs(Delta_motor[1].para.pos - Hit_2_Pos) < DELTA_POSITION_THRESHOLD &&
        //        fabs(Delta_motor[2].para.pos - Hit_2_Pos) < DELTA_POSITION_THRESHOLD)
        //     {
        //         Delta_State = DELTA_SERVE_BACK_2;
        //     }
        //     break;

        // case DELTA_SERVE_BACK_2:
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[0], Back_2_Pos, DELTA_SPEED);
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[1], Back_2_Pos, DELTA_SPEED);
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[2], Back_2_Pos, DELTA_SPEED);

        //     if(fabs(Delta_motor[0].para.pos - Back_2_Pos) < DELTA_POSITION_THRESHOLD &&
        //        fabs(Delta_motor[1].para.pos - Back_2_Pos) < DELTA_POSITION_THRESHOLD &&
        //        fabs(Delta_motor[2].para.pos - Back_2_Pos) < DELTA_POSITION_THRESHOLD)
        //     {
        //         Delta_State = DELTA_STOP;
        //     }
        //     break;

    case DELTA_STOP:
        // Jeffrey070318修改：急停状态保持失能，不再自动跳到GET_BALL，等待cmd恢复到READY/HIT后重新初始化。
        break;

    default:
        break;
    }
    DeltaUpdateDebugSnapshot();

    // switch (Serve_State)
    // {
    // case SERVE_INIT:
    //     if(Serve_motor.para.state == ENABLE_STATE)
    //     {
    //         Serve_State = SERVE_ORIGINAL_POS;
    //     }
    //     break;

    // case SERVE_ORIGINAL_POS:
    //     Pos_Speed_Ctrl(&hcan2, &Serve_motor, 0.0f, SERVE_SPEED);
    //     break;

    // case SERVE_READY:
    //     Pos_Speed_Ctrl(&hcan2, &Serve_motor, -0.4f, SERVE_SPEED);

    //     if(fabs(Pitch_motor.para.pos - (-0.3f)) < DELTA_POSITION_THRESHOLD)
    //     {
    //         Serve_State = SERVE_HIT;
    //     }
    //     break;

    // case SERVE_HIT:
    //     Pos_Speed_Ctrl(&hcan2, &Serve_motor, 2.0f, SERVE_SPEED);

    //     if(fabs(Serve_motor.para.pos - 1.9f) < DELTA_POSITION_THRESHOLD)
    //     {
    //         Serve_State = SERVE_BACK;
    //     }
    //     break;

    // case SERVE_BACK:
    //     Pos_Speed_Ctrl(&hcan2, &Serve_motor, 0.0f, SERVE_SPEED);

    //     if(fabs(Serve_motor.para.pos - 0.0f) < DELTA_POSITION_THRESHOLD)
    //     {
    //         Serve_State = SERVE_STOP;
    //     }
    //     break;

    // case SERVE_STOP:
    //     osDelay(2000);
    //     //Serve_State = SERVE_ORIGINAL_POS;
    //     break;

    // default:
    //     break;
    // }

#if ROBOT_HAS_SERVE
    // Jeffrey070318修改：只有R1存在serve封装，R2不发布serve控制话题。
    serve_cmd_send.serve_state = Serve_State;
    /* [测试] 填入 delta 自增计数器, serve 可验证 delta→serve 链路 */
    serve_cmd_send.test_seq = ++delta_test_seq;
    PubPushMessage(serve_cmd_pub, (void *)&serve_cmd_send);
#else
    // Jeffrey070318增加：R2无serve链路时仍保持delta测试序号递增，保证delta_feed节奏一致。
    delta_test_seq++;
#endif

    /* 填充 delta 反馈数据：上报 delta 当前状态给 cmd */
    delta_feedback_data.delta_feedback = (uint8_t)Delta_State;
    delta_feedback_data.pitch_angle = Pitch_motor.para.pos;
    /* [测试] 填入自增计数器(与上条消息同值, 表明同一周期) */
    delta_feedback_data.test_seq = delta_test_seq;
    PubPushMessage(delta_pub, (void *)&delta_feedback_data);
}

void Delta_Motion()
{
    // Disable_Motor_Mode(&hcan2, Delta_motor[0].para.id, Delta_motor[0].mode);
    // Disable_Motor_Mode(&hcan2, Delta_motor[1].para.id, Delta_motor[1].mode);
    // Disable_Motor_Mode(&hcan2, Delta_motor[2].para.id, Delta_motor[2].mode);
    // Jeffrey070318修改：Delta_Motion测试入口按R1/R2电机数量批量下发位置速度控制。
    DeltaPosSpeedAll(DELTA_TEST_DOWN_POS, DELTA_SPEED);
    // if(flag1)
    // {
    //     TargetAngle -= 0.1f;
    //     if(TargetAngle < -0.8f)
    //     {
    //         TargetAngle = -0.8f;
    //         flag1 = 0;
    //     }
    // }
    // else
    // {
    //     TargetAngle += 0.1f;
    //     if(TargetAngle > 0.0f)
    //     {
    //         TargetAngle = 0.0f;
    //         flag1 = 1;
    //     }
    // }
    // osDelay(1000);
    if (DeltaMotorsBelow(DELTA_TEST_TRIGGER_POS))
    {
        osDelay(100);
        // Jeffrey070318修改：测试回程同样按当前车种的Delta电机数量执行。
        DeltaPosSpeedAll(DELTA_TEST_BACK_POS, DELTA_SPEED);

        // if (Delta_motor[0].para.pos < -0.39 * 0.25f &&
        //     Delta_motor[1].para.pos < -0.39 * 0.25f &&
        //     Delta_motor[2].para.pos < -0.39 * 0.25f)
        // {
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[0], -0.8f, DELTA_SPEED);
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[1], -0.8f, DELTA_SPEED);
        //     Pos_Speed_Ctrl(&hcan2, &Delta_motor[2], -0.8f, DELTA_SPEED);
        //     osDelay(100);

        //     if (Delta_motor[0].para.pos < -0.39 * 0.25f &&
        //         Delta_motor[1].para.pos < -0.39 * 0.25f &&
        //         Delta_motor[2].para.pos < -0.39 * 0.25f)
        //     {
        //         osDelay(100);
        //         Pos_Speed_Ctrl(&hcan2, &Delta_motor[0], 0.0f, DELTA_SPEED);
        //         Pos_Speed_Ctrl(&hcan2, &Delta_motor[1], 0.0f, DELTA_SPEED);
        //         Pos_Speed_Ctrl(&hcan2, &Delta_motor[2], 0.0f, DELTA_SPEED);
        //     }
        // }
    }
}
