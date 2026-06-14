//Jeffrey070318修改：融合YYP整车cmd骨架，保留现有delta/serve调试链路并接入底盘、遥控器、IMU、光流和视觉接口。
#include "robot_def.h"
#include "robot_cmd.h"

#include "dm_imu.h"
#include "et_remote.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "optical_flow.h"

#include "bsp_log.h"

//Jeffrey070318增加：cmd层本地参数先集中在此处，后续R1/R2或遥控器确认后再迁移到robot_def.h条件编译。
#define CMD_YAW_SOURCE_DM_IMU      0
#define CMD_YAW_SOURCE_BMI088_INS  1
#define CMD_CHASSIS_YAW_SOURCE     CMD_YAW_SOURCE_BMI088_INS
#define CMD_RC_DEADBAND            50
#define CMD_CHASSIS_MOVE_SCALE     30.0f
#define CMD_CHASSIS_YAW_SCALE      4.0f
#define CMD_VISION_SEND_DIVIDER    5u

//Jeffrey070318增加：R1发球杆状态由cmd统一维护，后续serve/launcher封装可直接读取。
LauncherStatus_TypeDef g_launcher_status = LAUNCHER_ORIGIN;

static Publisher_t *delta_cmd_pub;
static Subscriber_t *delta_feed_sub;
static Delta_Ctrl_Cmd_s delta_cmd_send;
static Delta_Upload_Data_s delta_fetch_data;

//Jeffrey070318增加：融合YYP底盘控制话题，当前ONE_BOARD下通过消息中心直接给chassis应用发控制量。
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;
static Subscriber_t *chassis_feed_sub;
static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;
#endif

//Jeffrey070318增加：融合YYP传感器和视觉入口，按当前工程模块接口保存实例指针。
static ETRC_Ctrl_s *rc_data;
static OpticalFlowInstance *optical_flow;
static attitude_t *ins_imu_data;
static Vision_Recv_s *vision_recv_data;

static Robot_Status_e robot_state = ROBOT_STOP;

/* [测试] cmd 自增计数器, 随每次任务循环 +1 */
static uint8_t cmd_test_seq = 0;
/* [测试] LiveWatch: 收到 delta / 光流反馈的最新值 */
static uint8_t dbg_delta_state = 0;
static uint8_t dbg_delta_seq = 0;
static float dbg_flow_position_x = 0.0f;
static float dbg_flow_position_y = 0.0f;

//Jeffrey070318增加：遥控器死区处理，避免摇杆零点附近抖动直接进入底盘速度解算。
static int16_t ApplyDeadband(int16_t value)
{
    if (value > CMD_RC_DEADBAND || value < -CMD_RC_DEADBAND)
    {
        return value;
    }
    return 0;
}

//Jeffrey070318增加：统一清零底盘命令，急停、离线和模式未定义时都调用同一出口。
static void ClearChassisCmd(void)
{
#ifdef ONE_BOARD
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.offset_angle = 0.0f;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.chassis_speed_buff = 0;
#endif
}

//Jeffrey070318增加：按YYP思路保留KEEP_FRONT目标角锁定，yaw来源可在本文件宏中切换。
static void CalcOffsetAngle(void)
{
#ifdef ONE_BOARD
    static float keep_angle = 0.0f;
    static chassis_mode_e last_mode = CHASSIS_ZERO_FORCE;

    float yaw = 0.0f;
#if CMD_CHASSIS_YAW_SOURCE == CMD_YAW_SOURCE_DM_IMU
    const DM_IMU_Data_s *imu = DM_IMU_GetData();
    if (imu != NULL)
    {
        yaw = imu->yaw;
    }
#elif CMD_CHASSIS_YAW_SOURCE == CMD_YAW_SOURCE_BMI088_INS
    if (ins_imu_data != NULL)
    {
        yaw = ins_imu_data->Yaw;
    }
#endif

    if (chassis_cmd_send.chassis_mode == CHASSIS_KEEP_FRONT &&
        last_mode != CHASSIS_KEEP_FRONT)
    {
        keep_angle = yaw;
    }
    last_mode = chassis_cmd_send.chassis_mode;

    float offset = yaw - keep_angle;
    if (offset > 180.0f)
    {
        offset -= 360.0f;
    }
    if (offset < -180.0f)
    {
        offset += 360.0f;
    }
    chassis_cmd_send.offset_angle = offset;
#endif
}

//Jeffrey070318增加：将发球杆状态显式映射为delta动作，替换原先robot_state到delta_action的直接强转。
static Delta_Action_e GetDeltaAction(void)
{
    if (robot_state == ROBOT_STOP || g_launcher_status == LAUNCHER_STOP)
    {
        return DELTA_STOP_ACT;
    }
    if (g_launcher_status == LAUNCHER_HIT)
    {
        return DELTA_HIT;
    }
    return DELTA_READY;
}

//Jeffrey070318增加：融合YYP遥控器控制逻辑，但使用当前工程已有ETRemote数据结构。
static void RemoteControlSet(void)
{
    ClearChassisCmd();

    if (rc_data == NULL || !ETRemoteIsOnline())
    {
        g_launcher_status = LAUNCHER_STOP;
        return;
    }

#ifdef ONE_BOARD
    if (switch_is_up(rc_data->switch_left_3))
    {
        chassis_cmd_send.vx = CMD_CHASSIS_MOVE_SCALE * (float)ApplyDeadband(rc_data->rocker_r_);
        chassis_cmd_send.vy = CMD_CHASSIS_MOVE_SCALE * (float)ApplyDeadband(rc_data->rocker_r1);

        if (switch_is_down(rc_data->switch_right_3))
        {
            chassis_cmd_send.chassis_mode = CHASSIS_KEEP_FRONT;
            g_launcher_status = LAUNCHER_HIT;
        }
        else if (switch_is_up(rc_data->switch_right_3))
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            chassis_cmd_send.wz = CMD_CHASSIS_YAW_SCALE * (float)ApplyDeadband(rc_data->rocker_l_);
            g_launcher_status = LAUNCHER_ORIGIN;
        }
        else
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            g_launcher_status = LAUNCHER_ORIGIN;
        }
    }
    else if (switch_is_down(rc_data->switch_left_3))
    {
        //Jeffrey070318增加：视觉模式暂不闭环接管底盘，只保持前向并等待视觉控制量后续接入。
        chassis_cmd_send.chassis_mode = CHASSIS_KEEP_FRONT;
        g_launcher_status = LAUNCHER_ORIGIN;
    }
    else
    {
        g_launcher_status = LAUNCHER_ORIGIN;
    }
#endif
}

//Jeffrey070318增加：当前ET遥控器没有拨轮急停，先用遥控器离线作为整车急停条件。
static void EmergencyHandler(void)
{
    static Robot_Status_e last_robot_state = ROBOT_STOP;

    if (rc_data == NULL || !ETRemoteIsOnline())
    {
        robot_state = ROBOT_STOP;
        ClearChassisCmd();
        g_launcher_status = LAUNCHER_STOP;
    }
    else if (robot_state == ROBOT_STOP)
    {
        robot_state = ROBOT_READY;
    }

    if (last_robot_state != robot_state)
    {
        if (robot_state == ROBOT_STOP)
        {
            LOGERROR("[CMD] emergency stop!");
        }
        else
        {
            LOGINFO("[CMD] robot ready");
        }
        last_robot_state = robot_state;
    }
}

//Jeffrey070318增加：保留YYP光流调试观测量，后续定位闭环接入时可直接使用累计位移。
static void UpdateOpticalFlowDebug(void)
{
    const OpticalFlow_Data_s *flow_data = OpticalFlowGetData(optical_flow);
    if (flow_data != NULL && flow_data->updated)
    {
        dbg_flow_position_x = flow_data->position_x;
        dbg_flow_position_y = flow_data->position_y;
        OpticalFlowClearUpdated(optical_flow);
    }
}

//Jeffrey070318增加：按当前视觉模块API周期发送姿态，不沿用YYP旧版VisionSend参数形式。
static void SendVisionData(void)
{
    static uint8_t vision_send_count = 0;

    if (vision_recv_data == NULL)
    {
        return;
    }

    vision_send_count++;
    if (vision_send_count < CMD_VISION_SEND_DIVIDER)
    {
        return;
    }
    vision_send_count = 0;

    if (ins_imu_data != NULL)
    {
        VisionSetAltitude(ins_imu_data->Yaw, ins_imu_data->Pitch, ins_imu_data->Roll);
    }
    VisionSetFlag(COLOR_NONE, VISION_MODE_AIM, BULLET_SPEED_NONE);
    VisionSend();
}

void RobotCMDInit(void)
{
    //Jeffrey070318增加：使用当前工程ETRemote遥控器接口接收SBUS，避免YYP版remote_control中FLAG1外部符号冲突。
    rc_data = ETRemoteInit(&huart5);

    //Jeffrey070318增加：融合YYP视觉、DM-IMU、光流和BMI088 INS初始化入口。
    vision_recv_data = VisionInit(&huart9);
    DM_IMU_Init_Config_s imu_conf = {
        .tx_id = 0x11,
        .rx_id = 0x01,
        .can_handle = &hfdcan3,
    };
    DM_IMU_Init(&imu_conf);

    OpticalFlow_Init_Config_s flow_conf = {
        .usart_handle = &huart7,
        .protocol = OPTICAL_FLOW_UPIXELS,
        .flow_scale = OPTICAL_FLOW_DEFAULT_SCALE,
    };
    optical_flow = OpticalFlowInit(&flow_conf);
    ins_imu_data = INS_Init();

    //Jeffrey070318修改：cmd同时发布delta和chassis命令，保留现有delta调试回传订阅。
    delta_cmd_pub = PubRegister("delta_cmd", sizeof(Delta_Ctrl_Cmd_s));
    delta_feed_sub = SubRegister("delta_feed", sizeof(Delta_Upload_Data_s));

#ifdef ONE_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif

    ClearChassisCmd();
    robot_state = ROBOT_READY;
}

void RobotCMDTask(void)
{
    /* 接收 delta 回传的反馈数据 */
    if (SubGetMessage(delta_feed_sub, (void *)&delta_fetch_data))
    {
        /* [测试] 存入 LiveWatch 变量, 验证 delta→cmd 链路 */
        dbg_delta_state = delta_fetch_data.delta_feedback;
        dbg_delta_seq = delta_fetch_data.test_seq;
    }

#ifdef ONE_BOARD
    //Jeffrey070318增加：接收底盘反馈，当前先保留数据入口，后续可补功率/速度闭环。
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif

    RemoteControlSet();
    CalcOffsetAngle();
    UpdateOpticalFlowDebug();
    EmergencyHandler();

    //Jeffrey070318修改：delta命令从显式动作映射生成，避免Robot_Status_e和Delta_Action_e枚举值偶然耦合。
    delta_cmd_send.delta_action = GetDeltaAction();
    delta_cmd_send.test_seq = ++cmd_test_seq;
    PubPushMessage(delta_cmd_pub, (void *)&delta_cmd_send);

#ifdef ONE_BOARD
    //Jeffrey070318增加：向底盘应用推送遥控器解析后的速度、模式和KEEP_FRONT角度误差。
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif

    SendVisionData();
}
