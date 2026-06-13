#include <stdint.h>
#include "serve.h"

#include "cmsis_os.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

#define SERVE_POSITION_THRESHOLD 0.15f

Joint_Motor_t Serve_motor;

static Publisher_t *serve_pub;                    // 用于发布底盘的数据
static Subscriber_t *serve_sub;                   // 用于订阅底盘的控制命令
static Serve_Ctrl_Cmd_s serve_cmd_recv;         // 底盘接收到的控制命令
static Serve_Upload_Data_s serve_feedback_data; 

static Serve_State_t serve_state = SERVE_INIT;  // 本地状态变量，不被新消息覆盖

/* [测试] serve 自增计数器, 随每次任务循环 +1 */
static uint8_t serve_test_seq = 0;
/* [测试] LiveWatch: 收到 delta 命令的最新值 */
static uint8_t dbg_cmd_state = 0;
static uint8_t dbg_cmd_seq   = 0;

//SERVE MIT 模式参数
MIT_CTRL_DATA SERVE_ORIGINAL_DATA={
    .pos = 0.0f,
    .vel = 0.0f,
    .kp = 100.0f,
    .kd = 3.0f,
    .torq = 1.0f
};

MIT_CTRL_DATA READY_DATA={
    .pos = -1.2f,
    .vel = 0.0f,
    .kp = 100.0f,
    .kd = 3.0f,
    .torq = 1.0f
};

MIT_CTRL_DATA HIT_DATA={
    .pos = 1.0f,
    .vel = 0.0f,
    .kp = 400.0f,
    .kd = 3.0f,
    .torq = 5.0f
};

void ServeInit(void)
{
    serve_sub = SubRegister("serve_cmd", sizeof(Serve_Ctrl_Cmd_s));
    serve_pub = PubRegister("serve_feed", sizeof(Serve_Upload_Data_s));

    Serve_motor = (Joint_Motor_t){
        .mode = MIT_MODE,
        .para = {
            .id = 6,
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

        Enable_Motor_Mode(&hcan2, &Serve_motor);

        serve_state = SERVE_INIT;
}

void ServeTask()
{
    /* 接收 delta 下发的 serve 控制命令 */
    if (SubGetMessage(serve_sub, (void *)&serve_cmd_recv))
    {
        // serve_state = serve_cmd_recv.serve_state;  // 仅在新消息到达时更新本地状态
        // /* [测试] 存入 LiveWatch 变量, 验证 delta→serve 链路 */
        // dbg_cmd_state = (uint8_t)serve_cmd_recv.serve_state;
        // dbg_cmd_seq   = serve_cmd_recv.test_seq;
    }

    switch (serve_state)
    {
    case SERVE_INIT:
        Enable_Motor_Mode(&hcan2, &Serve_motor);

        if(Serve_motor.para.state == ENABLE_STATE)
        {
            serve_state = SERVE_ORIGINAL_POS;
        }
        break;

    case SERVE_ORIGINAL_POS:
        Mit_Ctrl(&hcan2, &Serve_motor, SERVE_ORIGINAL_DATA);

        if(fabs(Serve_motor.para.pos) < SERVE_POSITION_THRESHOLD)
        {
            serve_state = SERVE_READY;
        }
        break;

    case SERVE_READY:
        Mit_Ctrl(&hcan2, &Serve_motor, READY_DATA);

        if(fabs(Serve_motor.para.pos - (-1.2f)) < SERVE_POSITION_THRESHOLD)
        {
            serve_state = SERVE_HIT;
        }
        break;

    case SERVE_HIT:
        osDelay(1000);
        Mit_Ctrl(&hcan2, &Serve_motor, HIT_DATA);

        if(fabs(Serve_motor.para.pos - (1.0f)) < SERVE_POSITION_THRESHOLD)
        {
            serve_state = SERVE_BACK;
        }
        break;

    case SERVE_BACK:
        Mit_Ctrl(&hcan2, &Serve_motor, SERVE_ORIGINAL_DATA);

        if(fabs(Serve_motor.para.pos) < SERVE_POSITION_THRESHOLD)
        {
            serve_state = SERVE_STOP;
        }
        break;

    case SERVE_STOP:
        osDelay(2000);
        break;

    default:
        break;
    }

    /* 填充反馈数据后发布 */
    serve_feedback_data.serve_feedback = (uint8_t)serve_state;
    /* [测试] 填入自增计数器 */
    serve_feedback_data.test_seq = ++serve_test_seq;
    PubPushMessage(serve_pub, (void *)&serve_feedback_data);
}