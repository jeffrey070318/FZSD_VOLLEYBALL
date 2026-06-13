#include <stdint.h>
#include "delta.h"

#include "cmsis_os.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"

Joint_Motor_t Delta_motor[3];
Joint_Motor_t Pitch_motor;

Delta_State_t Delta_State = DELTA_INIT;
Serve_State_t Serve_State = SERVE_INIT;
uint8_t Count_1 = 0;

/* [测试] delta 自增计数器, 随每次任务循环 +1 */
static uint8_t delta_test_seq = 0;
/* [测试] LiveWatch: 收到 cmd / serve 的最新值 */
static uint8_t dbg_cmd_action = 0;
static uint8_t dbg_cmd_seq    = 0;
static uint8_t dbg_serve_state = 0;
static uint8_t dbg_serve_seq  = 0;

RemoteStatus_TypeDef Remote_status = ACTION_ORIGINAL;

/* Delta 击球Kp/Kd */
#define MIT_DELTA_HIT_KP     300.0f   // 位置刚度, 越大扭矩越大 (0~500)
#define MIT_DELTA_HIT_KD      3.0f    // 速度阻尼, 抑制震荡 (0~5)
#define MIT_DELTA_HIT_TORQ    5.0f    // 前馈力矩, 叠加到输出 (±30)

/* Delta 中 KP/KD 实现平滑运动 */
#define MIT_DELTA_GET_KP     200.0f
#define MIT_DELTA_GET_KD      3.0f
#define MIT_DELTA_GET_TORQ    3.0f

/* Delta 小 KP/KD 实现较缓慢平滑运动 */
#define MIT_DELTA_SLOW_KP       50.0f
#define MIT_DELTA_SLOW_KD        3.0f
#define MIT_DELTA_SLOW_TORQ      0.0f

/* Pitch 击球杆：快速下压 */
#define MIT_PITCH_HIT_KP     250.0f
#define MIT_PITCH_HIT_KD      2.0f
#define MIT_PITCH_HIT_TORQ    8.0f

/* Pitch：取球/预备 */
#define MIT_PITCH_GET_KP     100.0f
#define MIT_PITCH_GET_KD      1.0f
#define MIT_PITCH_GET_TORQ    2.0f

static Publisher_t *delta_pub;                    // 用于发布底盘的数据
static Subscriber_t *delta_sub;                   // 用于订阅底盘的控制命令
static Delta_Ctrl_Cmd_s delta_cmd_recv;         // 底盘接收到的控制命令
static Delta_Upload_Data_s delta_feedback_data; // 底盘回传的反馈数据

static Publisher_t *serve_cmd_pub;
static Subscriber_t *serve_feed_sub;
static Serve_Ctrl_Cmd_s serve_cmd_send;
static Serve_Upload_Data_s serve_fetch_data;

MIT_CTRL_DATA DELTA_ORIGINAL_DATA = {
            .pos = 0.0f,
            .vel = 0.0f,
            .kp = MIT_DELTA_SLOW_KP,
            .kd = MIT_DELTA_SLOW_KD,
            .torq = MIT_DELTA_SLOW_TORQ
};

MIT_CTRL_DATA HIT_1_DATA = {
            .pos = 0.0f,
            .vel = 0.0f,
            .kp = MIT_DELTA_HIT_KP,
            .kd = MIT_DELTA_HIT_KD,
            .torq = MIT_DELTA_HIT_TORQ
};

MIT_CTRL_DATA BACK_DATA = {
            .pos = 0.0f,
            .vel = 0.0f,
            .kp = MIT_DELTA_SLOW_KP,
            .kd = MIT_DELTA_SLOW_KD,
            .torq = MIT_DELTA_SLOW_TORQ
};

void DeltaInit(void)
{
    serve_cmd_pub = PubRegister("serve_cmd", sizeof(Serve_Ctrl_Cmd_s));
    serve_feed_sub = SubRegister("serve_feed", sizeof(Serve_Upload_Data_s));

    for (int i = 0; i < 3; i++)
    {
        Delta_motor[i] = (Joint_Motor_t){
        .mode = MIT_MODE,
        .para = {
            .id = i + 1,
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

    Pitch_motor = (Joint_Motor_t){
        .mode = POS_MODE,
        .para = {
            .id = 4,
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

    //Delta_State = DELTA_INIT;
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
    /* 接收 serve 回传的反馈数据 */
    if (SubGetMessage(serve_feed_sub, (void *)&serve_fetch_data))
    {
        /* [测试] 存入 LiveWatch 变量, 验证 serve→delta 链路 */
        dbg_serve_state = serve_fetch_data.serve_feedback;
        dbg_serve_seq   = serve_fetch_data.test_seq;
    }

    switch(Delta_State)
    {
        case DELTA_INIT:
            Enable_Motor_Mode(&hcan2, &Delta_motor[0]);
            osDelay(100);
            Enable_Motor_Mode(&hcan2, &Delta_motor[1]);
            osDelay(100);
            Enable_Motor_Mode(&hcan2, &Delta_motor[2]);
            osDelay(100);
            Enable_Motor_Mode(&hcan2, &Pitch_motor);
            osDelay(100);

            if(Delta_motor[0].para.state == ENABLE_STATE &&
               Delta_motor[1].para.state == ENABLE_STATE &&
               Delta_motor[2].para.state == ENABLE_STATE
               //Pitch_motor.para.state == ENABLE_STATE
               )
            {
                Delta_State = DELTA_ORIGINAL_POS;
                //Serve_State = SERVE_INIT;
            }
            break;

        case DELTA_ORIGINAL_POS:
        //回归零位置    
            Mit_Ctrl(&hcan2, &Delta_motor[0], DELTA_ORIGINAL_DATA);
            Mit_Ctrl(&hcan2, &Delta_motor[1], DELTA_ORIGINAL_DATA);
            Mit_Ctrl(&hcan2, &Delta_motor[2], DELTA_ORIGINAL_DATA);
            //Pos_Speed_Ctrl(&hcan2, &Pitch_motor, -0.4f, 2.0f);

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
            //Mit_Ctrl(&hcan2, Pitch_motor.para.id,      -0.4f, 0.0f, MIT_PITCH_GET_KP, MIT_PITCH_GET_KD, MIT_PITCH_GET_TORQ);
            // Mit_Ctrl(&hcan2, Delta_motor[0].para.id, Hit_1_Pos, 0.0f, MIT_DELTA_GET_KP, MIT_DELTA_GET_KD, MIT_DELTA_GET_TORQ);
            // Mit_Ctrl(&hcan2, Delta_motor[1].para.id, Hit_1_Pos, 0.0f, MIT_DELTA_GET_KP, MIT_DELTA_GET_KD, MIT_DELTA_GET_TORQ);
            // Mit_Ctrl(&hcan2, Delta_motor[2].para.id, Hit_1_Pos, 0.0f, MIT_DELTA_GET_KP, MIT_DELTA_GET_KD, MIT_DELTA_GET_TORQ);

            if(fabs(Delta_motor[0].para.pos - DELTA_ORIGINAL_DATA.pos) < DELTA_POSITION_THRESHOLD &&
               fabs(Delta_motor[1].para.pos - DELTA_ORIGINAL_DATA.pos) < DELTA_POSITION_THRESHOLD &&
               fabs(Delta_motor[2].para.pos - DELTA_ORIGINAL_DATA.pos) < DELTA_POSITION_THRESHOLD)
            {
                Delta_State = DELTA_SERVE_BACK_1;
            }

            break;

        case DELTA_SERVE_HIT_1:
            // MIT 模式: 击球阶段用最大 Kp 与额外前馈力矩
            Mit_Ctrl(&hcan2, &Delta_motor[0], HIT_1_DATA);
            Mit_Ctrl(&hcan2, &Delta_motor[1], HIT_1_DATA);
            Mit_Ctrl(&hcan2, &Delta_motor[2], HIT_1_DATA);

            if(fabs(Delta_motor[0].para.pos - HIT_1_DATA.pos) < DELTA_POSITION_THRESHOLD &&
               fabs(Delta_motor[1].para.pos - HIT_1_DATA.pos) < DELTA_POSITION_THRESHOLD &&
               fabs(Delta_motor[2].para.pos - HIT_1_DATA.pos) < DELTA_POSITION_THRESHOLD)
            {
                // Delta_State = DELTA_SERVE_BACK_1;
                // 击球杆下压: 高 Kp + 大前馈力矩
                // Mit_Ctrl(&hcan2, Pitch_motor.para.id, -0.8f, 0.0f, MIT_PITCH_HIT_KP, MIT_PITCH_HIT_KD, MIT_PITCH_HIT_TORQ);
                // Serve_State = SERVE_HIT;
            }

            break;

        case DELTA_SERVE_BACK_1:

            Mit_Ctrl(&hcan2, &Delta_motor[0], BACK_DATA);
            Mit_Ctrl(&hcan2, &Delta_motor[1], BACK_DATA);
            Mit_Ctrl(&hcan2, &Delta_motor[2], BACK_DATA);

            if(fabs(Delta_motor[0].para.pos - BACK_DATA.pos) < DELTA_POSITION_THRESHOLD &&
               fabs(Delta_motor[1].para.pos - BACK_DATA.pos) < DELTA_POSITION_THRESHOLD &&
               fabs(Delta_motor[2].para.pos - BACK_DATA.pos) < DELTA_POSITION_THRESHOLD)
            {
                Delta_State = DELTA_STOP;
                //Pos_Speed_Ctrl(&hcan2, &Pitch_motor, 0.0f, 6.0f);
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
            Count_1 ++;
            if(Count_1 > 5)
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

    /* 填充 serve 控制命令：delta 通过此话题向 serve 下发目标状态 */
    serve_cmd_send.serve_state = Serve_State;
    /* [测试] 填入 delta 自增计数器, serve 可验证 delta→serve 链路 */
    serve_cmd_send.test_seq = ++delta_test_seq;
    PubPushMessage(serve_cmd_pub, (void *)&serve_cmd_send);

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
    Pos_Speed_Ctrl(&hcan2, &Delta_motor[0], -0.8f, DELTA_SPEED);
    Pos_Speed_Ctrl(&hcan2, &Delta_motor[1], -0.8f, DELTA_SPEED);
    Pos_Speed_Ctrl(&hcan2, &Delta_motor[2], -0.8f, DELTA_SPEED);
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
    if (Delta_motor[0].para.pos < -0.18f &&
        Delta_motor[1].para.pos < -0.18f &&
        Delta_motor[2].para.pos < -0.18f)
    {
        osDelay(100);
        Pos_Speed_Ctrl(&hcan2, &Delta_motor[0], -0.4f, DELTA_SPEED);
        Pos_Speed_Ctrl(&hcan2, &Delta_motor[1], -0.4f, DELTA_SPEED);
        Pos_Speed_Ctrl(&hcan2, &Delta_motor[2], -0.4f, DELTA_SPEED);

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