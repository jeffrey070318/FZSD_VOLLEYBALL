#include "general_def.h"
#include "robot_cmd.h"

#include "et_remote.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

static Publisher_t *delta_cmd_pub;
static Subscriber_t *delta_feed_sub;
static Delta_Ctrl_Cmd_s delta_cmd_send;
static Delta_Upload_Data_s delta_fetch_data;

static Robot_Status_e robot_state;

/* [测试] cmd 自增计数器, 随每次任务循环 +1 */
static uint8_t cmd_test_seq = 0;
/* [测试] LiveWatch: 收到 delta 反馈的最新值 */
static uint8_t dbg_delta_state = 0;
static uint8_t dbg_delta_seq = 0;

void RobotCMDInit()
{
    //遥控

    //话题通信
    delta_cmd_pub = PubRegister("delta_cmd", sizeof(Delta_Ctrl_Cmd_s));
    delta_feed_sub = SubRegister("delta_feed", sizeof(Delta_Upload_Data_s));


    //robot_state
}

void RemoteControlSet()
{

}

static void EmergencyHandler()
{

}

void RobotCMDTask()
{
    /* 接收 delta 回传的反馈数据 */
    if (SubGetMessage(delta_feed_sub, (void *)&delta_fetch_data))
    {
        /* [测试] 存入 LiveWatch 变量, 验证 delta→cmd 链路 */
        dbg_delta_state = delta_fetch_data.delta_feedback;
        dbg_delta_seq   = delta_fetch_data.test_seq;
    }

    /* 根据 robot_state 填充 delta 控制命令 */
    delta_cmd_send.delta_action = (Delta_Action_e)robot_state;
    /* [测试] 填入自增计数器 */
    delta_cmd_send.test_seq = ++cmd_test_seq;

    //推送消息
    PubPushMessage(delta_cmd_pub, (void *)&delta_cmd_send);
    //视觉消息
}