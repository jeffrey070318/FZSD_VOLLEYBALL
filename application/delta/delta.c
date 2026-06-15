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

/* [测试] delta 自增计数器, 随每次任务循环 +1 */
static uint8_t delta_test_seq = 0;
/* [测试] LiveWatch: 收到 cmd / serve 的最新值 */
static uint8_t dbg_cmd_action = 0;
static uint8_t dbg_cmd_seq = 0;
#if ROBOT_HAS_SERVE
// Jeffrey070318修改：serve调试变量只在R1存在，R2无发球拨杆时不编译这些未使用变量。
static uint8_t dbg_serve_state = 0;
static uint8_t dbg_serve_seq = 0;
#endif

RemoteStatus_TypeDef Remote_status = ACTION_ORIGINAL;

// Jeffrey070318修改：R1/R2机械结构和电机数量不同，MIT参数拆成两套，当前R2先给独立占位值方便后续实车调参。
#define MIT_DELTA_R1_HIT_KP 300.0f
#define MIT_DELTA_R1_HIT_KD 3.0f
#define MIT_DELTA_R1_HIT_TORQ 5.0f
#define MIT_DELTA_R1_GET_KP 200.0f
#define MIT_DELTA_R1_GET_KD 3.0f
#define MIT_DELTA_R1_GET_TORQ 3.0f
#define MIT_DELTA_R1_SLOW_KP 50.0f
#define MIT_DELTA_R1_SLOW_KD 3.0f
#define MIT_DELTA_R1_SLOW_TORQ 0.0f
// Jeffrey070318的R1R2华丽分割线
#define MIT_DELTA_R2_HIT_KP 300.0f
#define MIT_DELTA_R2_HIT_KD 3.0f
#define MIT_DELTA_R2_HIT_TORQ 5.0f
#define MIT_DELTA_R2_GET_KP 200.0f
#define MIT_DELTA_R2_GET_KD 3.0f
#define MIT_DELTA_R2_GET_TORQ 3.0f
#define MIT_DELTA_R2_SLOW_KP 50.0f
#define MIT_DELTA_R2_SLOW_KD 3.0f
#define MIT_DELTA_R2_SLOW_TORQ 0.0f

#define DELTA_R1_ORIGINAL_POS 0.0f
#define DELTA_R1_HIT_1_POS 0.0f
#define DELTA_R1_BACK_POS 0.0f
#define DELTA_R1_TEST_DOWN_POS -0.8f
#define DELTA_R1_TEST_BACK_POS -0.4f
#define DELTA_R1_TEST_TRIGGER_POS -0.18f
// Jeffrey070318的R1R2华丽分割线
#define DELTA_R2_ORIGINAL_POS 0.0f
#define DELTA_R2_HIT_1_POS 0.0f
#define DELTA_R2_BACK_POS 0.0f
#define DELTA_R2_TEST_DOWN_POS -0.8f
#define DELTA_R2_TEST_BACK_POS -0.4f
#define DELTA_R2_TEST_TRIGGER_POS -0.18f

#define MIT_PITCH_R1_HIT_KP 250.0f
#define MIT_PITCH_R1_HIT_KD 2.0f
#define MIT_PITCH_R1_HIT_TORQ 8.0f
#define MIT_PITCH_R1_GET_KP 100.0f
#define MIT_PITCH_R1_GET_KD 1.0f
#define MIT_PITCH_R1_GET_TORQ 2.0f
// Jeffrey070318的R1R2华丽分割线
#define MIT_PITCH_R2_HIT_KP 250.0f
#define MIT_PITCH_R2_HIT_KD 2.0f
#define MIT_PITCH_R2_HIT_TORQ 8.0f
#define MIT_PITCH_R2_GET_KP 100.0f
#define MIT_PITCH_R2_GET_KD 1.0f
#define MIT_PITCH_R2_GET_TORQ 2.0f

// Jeffrey070318增加：按整车类型选择当前编译使用的Delta参数集合。
#if defined(ROBOT_R1)
#define MIT_DELTA_HIT_KP MIT_DELTA_R1_HIT_KP
#define MIT_DELTA_HIT_KD MIT_DELTA_R1_HIT_KD
#define MIT_DELTA_HIT_TORQ MIT_DELTA_R1_HIT_TORQ
#define MIT_DELTA_GET_KP MIT_DELTA_R1_GET_KP
#define MIT_DELTA_GET_KD MIT_DELTA_R1_GET_KD
#define MIT_DELTA_GET_TORQ MIT_DELTA_R1_GET_TORQ
#define MIT_DELTA_SLOW_KP MIT_DELTA_R1_SLOW_KP
#define MIT_DELTA_SLOW_KD MIT_DELTA_R1_SLOW_KD
#define MIT_DELTA_SLOW_TORQ MIT_DELTA_R1_SLOW_TORQ
// Jeffrey070318修改：目标位置宏避免与Delta_State_t枚举同名。
#define DELTA_ORIGINAL_TARGET_POS DELTA_R1_ORIGINAL_POS
#define DELTA_HIT_1_TARGET_POS DELTA_R1_HIT_1_POS
#define DELTA_BACK_TARGET_POS DELTA_R1_BACK_POS
#define DELTA_TEST_DOWN_POS DELTA_R1_TEST_DOWN_POS
#define DELTA_TEST_BACK_POS DELTA_R1_TEST_BACK_POS
#define DELTA_TEST_TRIGGER_POS DELTA_R1_TEST_TRIGGER_POS
#define MIT_PITCH_HIT_KP MIT_PITCH_R1_HIT_KP
#define MIT_PITCH_HIT_KD MIT_PITCH_R1_HIT_KD
#define MIT_PITCH_HIT_TORQ MIT_PITCH_R1_HIT_TORQ
#define MIT_PITCH_GET_KP MIT_PITCH_R1_GET_KP
#define MIT_PITCH_GET_KD MIT_PITCH_R1_GET_KD
#define MIT_PITCH_GET_TORQ MIT_PITCH_R1_GET_TORQ
#elif defined(ROBOT_R2)
#define MIT_DELTA_HIT_KP MIT_DELTA_R2_HIT_KP
#define MIT_DELTA_HIT_KD MIT_DELTA_R2_HIT_KD
#define MIT_DELTA_HIT_TORQ MIT_DELTA_R2_HIT_TORQ
#define MIT_DELTA_GET_KP MIT_DELTA_R2_GET_KP
#define MIT_DELTA_GET_KD MIT_DELTA_R2_GET_KD
#define MIT_DELTA_GET_TORQ MIT_DELTA_R2_GET_TORQ
#define MIT_DELTA_SLOW_KP MIT_DELTA_R2_SLOW_KP
#define MIT_DELTA_SLOW_KD MIT_DELTA_R2_SLOW_KD
#define MIT_DELTA_SLOW_TORQ MIT_DELTA_R2_SLOW_TORQ
// Jeffrey070318修改：目标位置宏避免与Delta_State_t枚举同名。
#define DELTA_ORIGINAL_TARGET_POS DELTA_R2_ORIGINAL_POS
#define DELTA_HIT_1_TARGET_POS DELTA_R2_HIT_1_POS
#define DELTA_BACK_TARGET_POS DELTA_R2_BACK_POS
#define DELTA_TEST_DOWN_POS DELTA_R2_TEST_DOWN_POS
#define DELTA_TEST_BACK_POS DELTA_R2_TEST_BACK_POS
#define DELTA_TEST_TRIGGER_POS DELTA_R2_TEST_TRIGGER_POS
#define MIT_PITCH_HIT_KP MIT_PITCH_R2_HIT_KP
#define MIT_PITCH_HIT_KD MIT_PITCH_R2_HIT_KD
#define MIT_PITCH_HIT_TORQ MIT_PITCH_R2_HIT_TORQ
#define MIT_PITCH_GET_KP MIT_PITCH_R2_GET_KP
#define MIT_PITCH_GET_KD MIT_PITCH_R2_GET_KD
#define MIT_PITCH_GET_TORQ MIT_PITCH_R2_GET_TORQ
#endif

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
        osDelay(100);
    }
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
    /* 接收 cmd 下发的 delta 控制命令 */
    if (SubGetMessage(delta_sub, (void *)&delta_cmd_recv))
    {
        // /* [测试] 存入 LiveWatch 变量, 验证 cmd→delta 链路 */
        // dbg_cmd_action = (uint8_t)delta_cmd_recv.delta_action;
        // dbg_cmd_seq    = delta_cmd_recv.test_seq;
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
        // Jeffrey070318修改：Delta电机按R1/R2数量批量使能，避免R2访问第三个电机。
        DeltaEnableMotors();
        Enable_Motor_Mode(&hcan2, &Pitch_motor);
        osDelay(100);

        if (DeltaMotorsEnabled()
            // Pitch_motor.para.state == ENABLE_STATE
        )
        {
            Delta_State = DELTA_ORIGINAL_POS;
            // Serve_State = SERVE_INIT;
        }
        break;

    case DELTA_ORIGINAL_POS:
        // 回归零位置
        // Jeffrey070318修改：原点控制按当前车种的Delta电机数量批量下发。
        DeltaMitCtrlAll(DELTA_ORIGINAL_DATA);
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
        osDelay(1000);
        Count_1++;
        if (Count_1 > 5)
        {
            Count_1 = 0;
            Delta_State = GET_BALL;
        }
        break;

    default:
        break;
    }

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
