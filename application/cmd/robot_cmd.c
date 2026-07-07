// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "dm_imu.h"
#include "optical_flow.h"
#include "user_lib.h"
#include "screen_task.h" // Jeffrey070318增加：接入共通LCD导航显示任务的数据初始化接口。
#include "controller.h"  // Jeffrey070318增加：视觉偏移模式需要PIDInstance/PIDCalculate。
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// Yaw数据源选择: 0=DM_IMU(CAN), 1=BMI088+INS(SPI/EKF)
#define YAW_SOURCE_DM_IMU 0
#define YAW_SOURCE_BMI088_INS 1
#define CHASSIS_YAW_SOURCE YAW_SOURCE_BMI088_INS

float temp_float = 0;
float temp_float1 = 0;
float temp_float_x = 0;
float temp_float_y = 0;
// YYP0417添加：发球杆状态全局变量定义,初始状态设为零位,根据遥控器右侧开关的状态进行切换
#if ROBOT_HAS_SERVE
// Jeffrey070318修改：launcher状态只在R1编译，R2没有发球拨杆时不保留该全局语义。
LauncherStatus_TypeDef g_launcher_status = LAUNCHER_ORIGIN;
#endif

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *delta_cmd_pub;
static Subscriber_t *delta_feed_sub;
static Delta_Ctrl_Cmd_s delta_cmd_send;
static Delta_Upload_Data_s delta_fetch_data;

static OpticalFlowInstance *optical_flow; // 光流模块实例
static attitude_t *ins_imu_data;          // BMI088 INS 解算结果指针
static RC_ctrl_t *rc_data;                // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data;   // 视觉接收数据指针,初始化时返回
#if ROBOT_ENABLE_VISION
// Jeffrey070318增加：视觉相机启用时才保留发送缓存，未接相机时避免无用视觉链路。
static Vision_Send_s vision_send_data; // 视觉发送数据
static uint8_t vision_serve_pending = 0;
static uint32_t vision_serve_trigger_tick = 0;
#endif

// Jeffrey070318增加：cmd层使用中性的Delta动作缓存，R1/R2分别在小函数内映射输入来源。
static Delta_Action_e cmd_delta_action = DELTA_READY;
static float cmd_pitch_target_pos = PITCH_REMOTE_ZERO_POS; // Jeffrey070318增加：缓存CMD下发的pitch目标位置。
static float cmd_pitch_speed = PITCH_REMOTE_SPEED;         // Jeffrey070318增加：缓存pitch位置速度模式速度，随车种参数切换。
static uint8_t cmd_test_seq = 0;
static uint8_t dbg_delta_state = 0;
static uint8_t dbg_delta_seq = 0;

// Jeffrey070318增加：CMD遥控链路LiveWatch变量，用于判断遥控器输入是否进入整车控制。
volatile uint32_t dbg_cmd_init_done = 0;
volatile uint32_t dbg_cmd_rc_data_addr = 0;
volatile uint8_t dbg_cmd_rc_data_is_null = 1;
volatile uint32_t dbg_cmd_task_loop_cnt = 0;
volatile uint32_t dbg_cmd_delta_pub_cnt = 0;
volatile uint32_t dbg_cmd_chassis_pub_cnt = 0;
volatile uint32_t dbg_cmd_emergency_stop_cnt = 0;
volatile uint32_t dbg_cmd_ready_cnt = 0;
volatile uint32_t dbg_cmd_rc_offline_stop_cnt = 0;
volatile uint8_t dbg_cmd_rc_online = 0;
volatile uint8_t dbg_cmd_switch_left1 = 0;
volatile uint8_t dbg_cmd_switch_left2 = 0;
volatile uint8_t dbg_cmd_switch_right1 = 0;
volatile uint8_t dbg_cmd_switch_right2 = 0;
volatile uint8_t dbg_cmd_switch_left = 0;
volatile uint8_t dbg_cmd_switch_right = 0;
volatile uint8_t dbg_cmd_robot_state = 0;
volatile uint8_t dbg_cmd_delta_action = 0;
volatile int16_t dbg_cmd_rocker_l_ = 0;
volatile int16_t dbg_cmd_rocker_l1 = 0;
volatile int16_t dbg_cmd_rocker_r_ = 0;
volatile int16_t dbg_cmd_rocker_r1 = 0;
volatile float dbg_cmd_chassis_vx = 0.0f;
volatile float dbg_cmd_chassis_vy = 0.0f;
volatile float dbg_cmd_chassis_wz = 0.0f;
volatile uint8_t dbg_cmd_chassis_mode = 0;
volatile uint16_t dbg_cmd_keep_front_hold_ticks = 0;
volatile uint8_t dbg_cmd_keep_front_lock_active = 0;
volatile uint8_t dbg_cmd_right2_test_active = 0;
volatile float dbg_cmd_right2_test_target_x = 0.0f;
volatile float dbg_cmd_right2_test_target_y = 0.0f;
volatile float dbg_cmd_pitch_target_pos = 0.0f; // Jeffrey070318增加：LiveWatch查看CMD下发给pitch的目标位置。
volatile float dbg_cmd_pitch_speed = 0.0f;      // Jeffrey070318增加：LiveWatch查看CMD下发给pitch的位置速度模式速度。
volatile uint8_t dbg_cmd_vision_flag = 0;
volatile uint8_t dbg_cmd_vision_serve_pending = 0;
volatile uint32_t dbg_cmd_vision_serve_elapsed_ms = 0;

// static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
// static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
// static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
// static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

// static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
// static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
// static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
// static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

// Jeffrey070318修改：视觉模式由上位机cmd动态选择，偏移PID实例在视觉导航启用时常驻。
#if ROBOT_ENABLE_VISION && ROBOT_ENABLE_OPTICAL_FLOW
static PIDInstance pid_vision_x;
static PIDInstance pid_vision_y;
static uint8_t last_vision_cmd = CMD_MOVE_PLAN;
static uint16_t offset_invalid_cnt = 0;
#endif
static uint8_t right2_fixed_move_active = 0;
static float right2_fixed_move_target_x = 0.0f;
static float right2_fixed_move_target_y = 0.0f;

void RobotCMDInit()
{
    dbg_cmd_init_done = 0;                       // Jeffrey070318增加：CMD初始化开始，便于判断是否卡在初始化中途。
    rc_data = RemoteControlInit(&huart5);        // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    dbg_cmd_rc_data_addr = (uint32_t)rc_data;    // Jeffrey070318增加：记录遥控器数据指针地址，判断RemoteControlInit是否返回。
    dbg_cmd_rc_data_is_null = (rc_data == NULL); // Jeffrey070318增加：确认CMD是否拿到遥控器数据缓存。
#if ROBOT_ENABLE_VISION
    // Jeffrey070318修改：相机接好并打开ROBOT_ENABLE_VISION后才初始化视觉串口。
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口huart
#else
    // Jeffrey070318增加：相机未连接时不注册视觉串口，避免无外设调试阶段进入异常。
    vision_recv_data = NULL;
#endif

    // gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    // shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    delta_cmd_pub = PubRegister("delta_cmd", sizeof(Delta_Ctrl_Cmd_s));
    delta_feed_sub = SubRegister("delta_feed", sizeof(Delta_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    // gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入

    DM_IMU_Init_Config_s imu_conf = {
        .tx_id = 0x11, // MCU → IMU 指令
        .rx_id = 0x01, // IMU → MCU 欧拉角
        .can_handle = &hfdcan3,
    };
    DM_IMU_Init(&imu_conf);

#if ROBOT_ENABLE_OPTICAL_FLOW
    {
        OpticalFlow_Init_Config_s flow_conf = {
            .usart_handle = &huart10,
            // Jeffrey070318修改：光流协议和安装方向改走robot_def.h，R1/R2可分别调参。
            .protocol = OPTICAL_FLOW_PROTOCOL,
            .flow_scale = OPTICAL_FLOW_SCALE_X,
            .flow_scale_y = OPTICAL_FLOW_SCALE_Y,
            .swap_xy = OPTICAL_FLOW_SWAP_XY,
            .x_direction = OPTICAL_FLOW_X_DIRECTION,
            .y_direction = OPTICAL_FLOW_Y_DIRECTION,
            .enable_global_frame = OPTICAL_FLOW_ENABLE_GLOBAL_FRAME,
        };
        optical_flow = OpticalFlowInit(&flow_conf);
    }
#else
    // Jeffrey070318增加：光流计未连接时不注册huart10，避免OpticalFlowInit链路触发HardFault。
    optical_flow = NULL;
#endif

    ins_imu_data = INS_Init(); // 获取 BMI088 EKF 解算结果指针(幂等,可安全多次调用)

// Jeffrey070318修改：视觉cmd可在坐标导航和偏移PID间切换，因此PID在视觉导航启用时初始化。
#if ROBOT_ENABLE_VISION && ROBOT_ENABLE_OPTICAL_FLOW
    PID_Init_Config_s cfg_x = {
        .Kp = VISION_PID_X_KP,
        .Ki = VISION_PID_X_KI,
        .Kd = VISION_PID_X_KD,
        .MaxOut = VISION_PID_X_MAXOUT,
        .DeadBand = VISION_PID_DEADBAND,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = VISION_PID_X_MAXOUT * VISION_PID_INTEGRAL_RATIO,
    };
    PID_Init_Config_s cfg_y = {
        .Kp = VISION_PID_Y_KP,
        .Ki = VISION_PID_Y_KI,
        .Kd = VISION_PID_Y_KD,
        .MaxOut = VISION_PID_Y_MAXOUT,
        .DeadBand = VISION_PID_DEADBAND,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = VISION_PID_Y_MAXOUT * VISION_PID_INTEGRAL_RATIO,
    };
    PIDInit(&pid_vision_x, &cfg_x);
    PIDInit(&pid_vision_y, &cfg_y);
#endif

    // Jeffrey070318增加：把cmd里已初始化的数据指针传给屏幕任务，任务是否创建由ROBOT_ENABLE_SCREEN_TASK控制。
    ScreenNavInit(optical_flow, ins_imu_data, vision_recv_data, rc_data);
    dbg_cmd_init_done = 1; // Jeffrey070318增加：CMD初始化完成，后续若遥控仍无数据则优先查接收链路。
}

/**
 * YYP0418修改
 * @brief 根据IMU获取的当前底盘yaw角计算和目标保持角度的误差
 *        进入CHASSIS_KEEP_FRONT模式时捕获当前yaw作为目标角度,
 *        之后每周期计算 yaw - keep_angle 并归一化到[-180, 180]
 *
 *        yaw数据源由CHASSIS_YAW_SOURCE宏控制:
 *        YAW_SOURCE_DM_IMU → DM-IMU-L1 (CAN, Euler)
 *        YAW_SOURCE_BMI088_INS → BMI088 + EKF (SPI)
 */
static void CalcOffsetAngle()
{
    static float keep_angle = 0;
    static chassis_mode_e last_mode = CHASSIS_ZERO_FORCE;

    float yaw;
#if CHASSIS_YAW_SOURCE == YAW_SOURCE_DM_IMU
    const DM_IMU_Data_s *imu = DM_IMU_GetData();
    yaw = imu->yaw;
#elif CHASSIS_YAW_SOURCE == YAW_SOURCE_BMI088_INS
    yaw = ins_imu_data->Yaw;
#else
    yaw = 0;
#endif

    // 进入KEEP_FRONT模式时锁定当前yaw为目标角度
    if (chassis_cmd_send.chassis_mode == CHASSIS_KEEP_FRONT && last_mode != CHASSIS_KEEP_FRONT)
    {
        keep_angle = yaw;
    }
    last_mode = chassis_cmd_send.chassis_mode;

    // 计算偏移并归一化到[-180, 180]
    float offset = yaw - keep_angle;
    if (offset > 180.0f)
        offset -= 360.0f;
    if (offset < -180.0f)
        offset += 360.0f;
    chassis_cmd_send.offset_angle = offset;
}

/**
 * @brief 视觉导航: 替换摇杆 vx/vy 为导航计算值.
 *
 * 不动条件: 视觉离线 | 光流离线 | target=(0,0) | 已到达目标.\n
 * 右开关及朝向控制逻辑与手动模式一致, 本函数不干预.
 */
// Jeffrey070318修改：左一开关下位进入自动导航，vx/vy由视觉/光流链路给出。
static void AutoNavigation(void)
{
#if ROBOT_ENABLE_VISION && ROBOT_ENABLE_OPTICAL_FLOW
    if (!VisionIsOnline() || !OpticalFlowIsOnline(optical_flow))
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        return;
    }

    uint8_t cmd = vision_recv_data->cmd;

    if (cmd != last_vision_cmd)
    {
        if (cmd == CMD_OFFSET)
        {
            PID_Init_Config_s cfg_x = {
                .Kp = VISION_PID_X_KP,
                .Ki = VISION_PID_X_KI,
                .Kd = VISION_PID_X_KD,
                .MaxOut = VISION_PID_X_MAXOUT,
                .DeadBand = VISION_PID_DEADBAND,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = VISION_PID_X_MAXOUT * VISION_PID_INTEGRAL_RATIO,
            };
            PID_Init_Config_s cfg_y = {
                .Kp = VISION_PID_Y_KP,
                .Ki = VISION_PID_Y_KI,
                .Kd = VISION_PID_Y_KD,
                .MaxOut = VISION_PID_Y_MAXOUT,
                .DeadBand = VISION_PID_DEADBAND,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = VISION_PID_Y_MAXOUT * VISION_PID_INTEGRAL_RATIO,
            };
            PIDInit(&pid_vision_x, &cfg_x);
            PIDInit(&pid_vision_y, &cfg_y);
            offset_invalid_cnt = 0;
        }
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        last_vision_cmd = cmd;
        return;
    }

    if (cmd == CMD_MOVE_PLAN)
    {
        const OpticalFlow_Data_s *flow_data = OpticalFlowGetData(optical_flow);

        if (vision_recv_data->target_x == 0.0f && vision_recv_data->target_y == 0.0f)
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
            return;
        }

        float err_x = vision_recv_data->target_x - flow_data->position_x_global;
        float err_y = vision_recv_data->target_y - flow_data->position_y_global;
        float dist = Sqrt(err_x * err_x + err_y * err_y);

        if (dist > 3.0f || dist < NAV_ARRIVAL_DIST)
        {
            chassis_cmd_send.vx = 0.0f;
            chassis_cmd_send.vy = 0.0f;
        }
        else
        {
            float speed = dist * NAV_SPEED_GAIN;
            speed = float_constrain(speed, 0.0f, NAV_MAX_SPEED);
            chassis_cmd_send.vx = (err_x / dist) * speed;
            chassis_cmd_send.vy = (err_y / dist) * speed;
        }
    }
    else
    {
        if (vision_recv_data->target_x == 0.0f && vision_recv_data->target_y == 0.0f)
        {
            offset_invalid_cnt++;
            if (offset_invalid_cnt >= 200u)
            {
                chassis_cmd_send.vx = 0.0f;
                chassis_cmd_send.vy = 0.0f;
            }
            else
            {
                float decay = 1.0f - (float)offset_invalid_cnt / 200.0f;
                chassis_cmd_send.vx *= decay;
                chassis_cmd_send.vy *= decay;
            }
        }
        else
        {
            offset_invalid_cnt = 0;
            chassis_cmd_send.vx = PIDCalculate(&pid_vision_x, -vision_recv_data->target_x, 0.0f);
            chassis_cmd_send.vy = PIDCalculate(&pid_vision_y, -vision_recv_data->target_y, 0.0f);
        }
    }
#else
    // Jeffrey070318增加：相机或光流未连接时禁用自动导航，底盘速度由遥控器逻辑接管。
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
#endif
}

// Jeffrey070318增加：右二临时固定距离测试，用光流当前位置生成Y方向误差，但输出走视觉误差PID，便于无算法时调offset响应。
static void Right2FixedMoveTest(void)
{
#if ROBOT_ENABLE_OPTICAL_FLOW
    if (!OpticalFlowIsOnline(optical_flow))
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        right2_fixed_move_active = 0;
        dbg_cmd_right2_test_active = 0;
        return;
    }

    const OpticalFlow_Data_s *flow_data = OpticalFlowGetData(optical_flow);
    if (!right2_fixed_move_active)
    {
        right2_fixed_move_target_x = flow_data->position_x_global;
        right2_fixed_move_target_y = flow_data->position_y_global + RIGHT2_FIXED_MOVE_TEST_Y;
        right2_fixed_move_active = 1;

        PID_Init_Config_s cfg_y = {
            .Kp = VISION_PID_Y_KP,
            .Ki = VISION_PID_Y_KI,
            .Kd = VISION_PID_Y_KD,
            .MaxOut = VISION_PID_Y_MAXOUT,
            .DeadBand = RIGHT2_FIXED_MOVE_TEST_DEADBAND,
            .Improve = PID_Integral_Limit,
            .IntegralLimit = VISION_PID_Y_MAXOUT * VISION_PID_INTEGRAL_RATIO,
        };
        PIDInit(&pid_vision_y, &cfg_y);
    }

    float err_y = right2_fixed_move_target_y - flow_data->position_y_global;
    float dist = (err_y >= 0.0f) ? err_y : -err_y;

    dbg_cmd_right2_test_active = right2_fixed_move_active;
    dbg_cmd_right2_test_target_x = right2_fixed_move_target_x;
    dbg_cmd_right2_test_target_y = right2_fixed_move_target_y;

    if (dist > 3.0f || dist < NAV_ARRIVAL_DIST)
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
    }
    else
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = PIDCalculate(&pid_vision_y, -err_y, 0.0f);
    }
#else
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
#endif
}
// Jeffrey070318增加：遥控旋转使用二次曲线，小摇杆保留更细角度控制，满杆仍保留可用转速。
static float RemoteYawWzFromJoystick(int16_t rocker_yaw)
{
    int16_t abs_rocker = abs(rocker_yaw);
    if (abs_rocker <= CMD_REMOTE_DEADBAND)
        return 0.0f;

    float range = CMD_REMOTE_YAW_STICK_MAX - (float)CMD_REMOTE_DEADBAND;
    if (range <= 0.0f)
        return 0.0f;

    float ratio = ((float)abs_rocker - (float)CMD_REMOTE_DEADBAND) / range;
    if (ratio > 1.0f)
        ratio = 1.0f;

    float wz = ratio * ratio * CMD_REMOTE_YAW_MAX_WZ;
    return (rocker_yaw >= 0) ? wz : -wz;
}

/**
 * YYP0418修改
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    static uint8_t keep_front_lock_active = 0;
    static uint16_t keep_front_idle_ticks = 0;

    // Jeffrey070318增加：缓存CMD看到的遥控器原始值，判断rc_data指针是否正常更新。
    dbg_cmd_rocker_r_ = rc_data[TEMP].rc.rocker_r_;
    dbg_cmd_rocker_r1 = rc_data[TEMP].rc.rocker_r1;
    dbg_cmd_rocker_l1 = rc_data[TEMP].rc.rocker_l1;
    dbg_cmd_rocker_l_ = rc_data[TEMP].rc.rocker_l_;
    dbg_cmd_switch_left1 = rc_data[TEMP].rc.switch_left1;
    dbg_cmd_switch_left2 = rc_data[TEMP].rc.switch_left2;
    dbg_cmd_switch_right1 = rc_data[TEMP].rc.switch_right1;
    dbg_cmd_switch_right2 = rc_data[TEMP].rc.switch_right2;
    dbg_cmd_switch_left = rc_data[TEMP].rc.switch_left;
    dbg_cmd_switch_right = rc_data[TEMP].rc.switch_right;

    const int move_x_active = abs(rc_data[TEMP].rc.rocker_r_) > CMD_REMOTE_DEADBAND;
    const int move_y_active = abs(rc_data[TEMP].rc.rocker_r1) > CMD_REMOTE_DEADBAND;
    const int yaw_active = abs(rc_data[TEMP].rc.rocker_l_) > CMD_REMOTE_DEADBAND;
    const int auto_mode = switch_is_down(rc_data[TEMP].rc.switch_left1);

    if (auto_mode)
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_right2))
        {
            Right2FixedMoveTest();
        }
        else
        {
            right2_fixed_move_active = 0;
            dbg_cmd_right2_test_active = 0;
            AutoNavigation();
        }
    }
    else
    {
        if (move_x_active)
            chassis_cmd_send.vx = CMD_REMOTE_MOVE_SCALE * (float)rc_data[TEMP].rc.rocker_r_;
        else
            chassis_cmd_send.vx = 0.0f;

        if (move_y_active)
            chassis_cmd_send.vy = CMD_REMOTE_MOVE_SCALE * (float)rc_data[TEMP].rc.rocker_r1;
        else
            chassis_cmd_send.vy = 0.0f;
    }

    const int chassis_translate_active = auto_mode ? (chassis_cmd_send.vx != 0.0f || chassis_cmd_send.vy != 0.0f)
                                                   : (move_x_active || move_y_active);
    if (yaw_active)
    {
        keep_front_lock_active = 0;
        keep_front_idle_ticks = 0;
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.wz = RemoteYawWzFromJoystick(rc_data[TEMP].rc.rocker_l_);
    }
    else if (chassis_translate_active)
    {
        keep_front_lock_active = 1;
        keep_front_idle_ticks = 0;
        chassis_cmd_send.chassis_mode = CHASSIS_KEEP_FRONT;
        chassis_cmd_send.wz = 0.0f;
    }
    else if (keep_front_lock_active)
    {
        if (keep_front_idle_ticks < 0xFFFFu)
            keep_front_idle_ticks++;
        chassis_cmd_send.chassis_mode = CHASSIS_KEEP_FRONT;
        chassis_cmd_send.wz = 0.0f;
    }
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.wz = 0.0f;
    }
    // Jeffrey070318增加：缓存CMD下发底盘前的速度命令，判断死区/急停是否把输出清零。
    dbg_cmd_chassis_vx = chassis_cmd_send.vx;
    dbg_cmd_chassis_vy = chassis_cmd_send.vy;
    dbg_cmd_chassis_wz = chassis_cmd_send.wz;
    dbg_cmd_chassis_mode = (uint8_t)chassis_cmd_send.chassis_mode;
    dbg_cmd_keep_front_hold_ticks = keep_front_idle_ticks;
    dbg_cmd_keep_front_lock_active = keep_front_lock_active;
}

// Jeffrey070318增加：把左摇杆上下比例映射到pitch前/后目标，摇杆回中则回零点。
#if !ROBOT_LOCK_PITCH_TARGET
static float RemotePitchTargetFromJoystick(int16_t rocker_l1)
{
    // Jeffrey070318增加：pitch摇杆方向修正，PITCH_STICK_DIRECTION=-1时上下颠倒
    rocker_l1 = rocker_l1 * PITCH_STICK_DIRECTION;

    float ratio = (float)rocker_l1 / PITCH_REMOTE_STICK_MAX;
    if (ratio > 1.0f)
        ratio = 1.0f;
    if (ratio < -1.0f)
        ratio = -1.0f;

#if PITCH_REMOTE_MODE == 1
    // Mode 1: 摇杆中心=后方起始位(BACK_POS), 下拉前移从BACK到ZERO
    // ratio=0(中位)→BACK, ratio=+1(拉到底)→ZERO
    (void)CMD_REMOTE_DEADBAND; // 本模式不使用死区，回中即停在后方起始位
    if (ratio < 0.0f)
    {
        return PITCH_REMOTE_BACK_POS;
    }
    return PITCH_REMOTE_BACK_POS + ratio * (PITCH_REMOTE_ZERO_POS - PITCH_REMOTE_BACK_POS);
#else
    // Mode 0: 原有逻辑，摇杆中心=零点，上推前倾/下拉后仰
    if (abs(rocker_l1) <= CMD_REMOTE_DEADBAND)
    {
        return PITCH_REMOTE_ZERO_POS;
    }
    if (ratio > 0.0f)
    {
        return PITCH_REMOTE_ZERO_POS + ratio * (PITCH_REMOTE_FRONT_POS - PITCH_REMOTE_ZERO_POS);
    }
    return PITCH_REMOTE_ZERO_POS + (-ratio) * (PITCH_REMOTE_BACK_POS - PITCH_REMOTE_ZERO_POS);
#endif
}
#endif

// Jeffrey070318修改：手动模式左二控制机械臂；自动模式下机械臂只接受视觉flag，不再响应遥控左二。
static void RemoteControlSetArm(void)
{
#if ROBOT_LOCK_PITCH_TARGET
    cmd_pitch_target_pos = ROBOT_LOCK_PITCH_TARGET_POS;
#else
    cmd_pitch_target_pos = RemotePitchTargetFromJoystick(rc_data[TEMP].rc.rocker_l1);
#endif
    cmd_pitch_speed = PITCH_REMOTE_SPEED;
    // Jeffrey070318增加：缓存pitch目标，确认CMD下发位置是否符合当前调试策略。
    dbg_cmd_pitch_target_pos = cmd_pitch_target_pos;
    dbg_cmd_pitch_speed = cmd_pitch_speed;

    if (switch_is_down(rc_data[TEMP].rc.switch_left1))
    {
#if ROBOT_ENABLE_VISION
        uint8_t vision_flag = (VisionIsOnline() && vision_recv_data != NULL) ? vision_recv_data->flag : 0u;
        dbg_cmd_vision_flag = vision_flag;

        if (vision_flag == 1u)
        {
            uint32_t now = HAL_GetTick();
            if (!vision_serve_pending)
            {
                vision_serve_pending = 1;
                vision_serve_trigger_tick = now;
            }

            dbg_cmd_vision_serve_elapsed_ms = now - vision_serve_trigger_tick;
            if (dbg_cmd_vision_serve_elapsed_ms >= VISION_SERVE_TRIGGER_DELAY_MS)
                cmd_delta_action = DELTA_SERVE;
            else
                cmd_delta_action = DELTA_READY;
        }
        else
        {
            vision_serve_pending = 0;
            dbg_cmd_vision_serve_elapsed_ms = 0;
            cmd_delta_action = DELTA_READY;
        }
        dbg_cmd_vision_serve_pending = vision_serve_pending;
#else
        cmd_delta_action = DELTA_READY;
#endif
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left2))
    {
#if ROBOT_ENABLE_VISION
        vision_serve_pending = 0;
        dbg_cmd_vision_flag = 0;
        dbg_cmd_vision_serve_pending = 0;
        dbg_cmd_vision_serve_elapsed_ms = 0;
#endif
        cmd_delta_action = DELTA_HIT;
    }
    else
    {
#if ROBOT_ENABLE_VISION
        vision_serve_pending = 0;
        dbg_cmd_vision_flag = 0;
        dbg_cmd_vision_serve_pending = 0;
        dbg_cmd_vision_serve_elapsed_ms = 0;
#endif
        cmd_delta_action = DELTA_READY;
    }
    // Jeffrey070318增加：缓存CMD转换出的机械臂动作，自动模式下应只随视觉flag变化。
    dbg_cmd_delta_action = (uint8_t)cmd_delta_action;
#if ROBOT_HAS_SERVE
    // Jeffrey070318修改：当前右开关不再控制launcher，避免R1调机械臂时误触发发球拨杆。
    if (g_launcher_status != LAUNCHER_STOP)
        g_launcher_status = LAUNCHER_ORIGIN;
#endif
}

// Jeffrey070318增加：遥控器离线时不使用rc_data中的摇杆/开关值，防止错帧或掉线造成底盘误动作。
static void RemoteOfflineStop(void)
{
    robot_state = ROBOT_STOP;
    cmd_delta_action = DELTA_STOP_ACT;
    cmd_pitch_target_pos = PITCH_REMOTE_ZERO_POS;
    cmd_pitch_speed = PITCH_REMOTE_SPEED;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    dbg_cmd_rc_offline_stop_cnt++;
    dbg_cmd_chassis_vx = chassis_cmd_send.vx;
    dbg_cmd_chassis_vy = chassis_cmd_send.vy;
    dbg_cmd_chassis_wz = chassis_cmd_send.wz;
    dbg_cmd_chassis_mode = (uint8_t)chassis_cmd_send.chassis_mode;
    dbg_cmd_delta_action = (uint8_t)cmd_delta_action;
    dbg_cmd_pitch_target_pos = cmd_pitch_target_pos;
    dbg_cmd_pitch_speed = cmd_pitch_speed;
#if ROBOT_ENABLE_VISION
    vision_serve_pending = 0;
    dbg_cmd_vision_flag = 0;
    dbg_cmd_vision_serve_pending = 0;
    dbg_cmd_vision_serve_elapsed_ms = 0;
#endif
}

static Delta_Action_e GetDeltaAction(void)
{
    if (robot_state == ROBOT_STOP)
    {
        return DELTA_STOP_ACT;
    }
    return cmd_delta_action;
}

/**
 *   YYP0418删除
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
// Jeffrey070318修改：键鼠入口当前仍为占位，标记unused避免编译告警干扰遥控器调试。
__attribute__((unused)) static void MouseKeySet()
{
    // 占位
}

/**
 * @brief  紧急停止,包括右一开关下位/重要模块离线/双板通信失效等
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // Jeffrey070318修改：右一开关下位急停，回拨恢复运行；右二暂不参与CMD控制。
    if (switch_is_down(rc_data[TEMP].rc.switch_right1)) // 后续再叠加重要应用/模块离线判断
    {
        robot_state = ROBOT_STOP;
        // gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        // Jeffrey070318增加：急停次数和最终底盘命令缓存，用于确认左开关是否持续触发急停。
        dbg_cmd_emergency_stop_cnt++;
        dbg_cmd_chassis_vx = chassis_cmd_send.vx;
        dbg_cmd_chassis_vy = chassis_cmd_send.vy;
        dbg_cmd_chassis_wz = chassis_cmd_send.wz;
        dbg_cmd_chassis_mode = (uint8_t)chassis_cmd_send.chassis_mode;
        // shoot_cmd_send.shoot_mode = SHOOT_OFF;
        // shoot_cmd_send.friction_mode = FRICTION_OFF;
        // shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // Jeffrey070318修改：右一开关离开下位即恢复整车运行，符合“回拨解除急停”的测试习惯。
    else
    {
        robot_state = ROBOT_READY;
        // Jeffrey070318增加：恢复运行次数，用于确认左开关上拨是否被CMD识别。
        dbg_cmd_ready_cnt++;
        // shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * YYP0418修改*/
void RobotCMDTask()
{
    // Jeffrey070318增加：CMD任务循环计数，确认RobotTask是否持续调用RobotCMDTask。
    dbg_cmd_task_loop_cnt++;
    dbg_cmd_rc_online = RemoteControlIsOnline();
    const OpticalFlow_Data_s *flow_data = NULL; // Jeffrey070318增加：光流关闭时保持NULL，避免后续误解引用。
#if !ROBOT_ENABLE_OPTICAL_FLOW && !ROBOT_ENABLE_VISION
    (void)flow_data; // Jeffrey070318增加：视觉和光流同时关闭时消除未使用变量告警。
#endif

    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    if (SubGetMessage(delta_feed_sub, (void *)&delta_fetch_data))
    {
        dbg_delta_state = delta_fetch_data.delta_feedback;
        dbg_delta_seq = delta_fetch_data.test_seq;
    }
    // SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    // SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

// 更新光流模块偏航角,用于世界坐标系旋转映射
#if ROBOT_ENABLE_OPTICAL_FLOW
#if CHASSIS_YAW_SOURCE == YAW_SOURCE_DM_IMU
    OpticalFlowSetYaw(optical_flow, DM_IMU_GetData()->yaw);
#elif CHASSIS_YAW_SOURCE == YAW_SOURCE_BMI088_INS
    OpticalFlowSetYaw(optical_flow, ins_imu_data->Yaw);
#endif

    // 读取光流模块累计位移数据,使用世界坐标系下的值。
    flow_data = OpticalFlowGetData(optical_flow);
    if (flow_data != NULL && flow_data->updated)
    {
        OpticalFlowClearUpdated(optical_flow);
        temp_float_x = flow_data->position_x_global;
        temp_float_y = flow_data->position_y_global;
    }
#else
    // Jeffrey070318增加：光流计未连接时不读取定位数据，临时把上报/显示坐标保持为0。
    temp_float_x = 0.0f;
    temp_float_y = 0.0f;
#endif

    // Jeffrey070318修改：只有遥控器在线时才读取摇杆/开关，离线时直接进入安全输出。
    if (dbg_cmd_rc_online)
    {
        RemoteControlSet();
        RemoteControlSetArm();
        EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
    }
    else
    {
        RemoteOfflineStop();
    }

    // 遥控/急停逻辑确定本周期底盘模式后再计算偏移角，保证重新平移时当周期锁当前yaw。
    CalcOffsetAngle();

    delta_cmd_send.delta_action = GetDeltaAction();
    // Jeffrey070318增加：CMD每周期随Delta动作一起下发pitch目标，急停/离线时保持回零。
    delta_cmd_send.pitch_target_pos = (robot_state == ROBOT_STOP) ? PITCH_REMOTE_ZERO_POS : cmd_pitch_target_pos;
    delta_cmd_send.pitch_speed = cmd_pitch_speed;
    delta_cmd_send.test_seq = ++cmd_test_seq;
    PubPushMessage(delta_cmd_pub, (void *)&delta_cmd_send);
    // Jeffrey070318增加：缓存发布给Delta的最终动作，判断急停是否把机械臂动作覆盖掉。
    dbg_cmd_delta_pub_cnt++;
    dbg_cmd_delta_action = (uint8_t)delta_cmd_send.delta_action;
    dbg_cmd_robot_state = (uint8_t)robot_state;

// 上报世界坐标系位置及偏航角到视觉上位机
#if ROBOT_ENABLE_VISION
    if (!dbg_cmd_rc_online)
        vision_send_data.mode = MODE_IDLE;
    else if (switch_is_down(rc_data[TEMP].rc.switch_left1))
        vision_send_data.mode = MODE_SELF;
    else
        vision_send_data.mode = MODE_REMOTE;

#if CHASSIS_YAW_SOURCE == YAW_SOURCE_DM_IMU
    vision_send_data.robot_yaw = DM_IMU_GetData()->yaw;
#elif CHASSIS_YAW_SOURCE == YAW_SOURCE_BMI088_INS
    vision_send_data.robot_yaw = ins_imu_data->Yaw;
#endif
    // Jeffrey070318修改：光流未连接时视觉上报坐标置0，避免访问空指针。
    vision_send_data.robot_x = (flow_data != NULL) ? flow_data->position_x_global : 0.0f;
    vision_send_data.robot_y = (flow_data != NULL) ? flow_data->position_y_global : 0.0f;
#endif

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    // Jeffrey070318增加：记录底盘命令发布次数，确认CMD到chassis消息链路是否有输出。
    dbg_cmd_chassis_pub_cnt++;
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    // PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    // PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
#if ROBOT_ENABLE_VISION
    // Jeffrey070318修改：相机未连接时跳过视觉发送，避免启动未连接串口链路。
    VisionSend(&vision_send_data); // 发送
#endif
}
