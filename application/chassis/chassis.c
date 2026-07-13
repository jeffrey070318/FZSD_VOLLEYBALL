/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
// #include "referee_UI.h"
#include "controller.h"
#include "user_lib.h"
#include "arm_math.h"
#if ROBOT_ENABLE_VOFA_CHASSIS_DEBUG
#include "vofa.h"
#include "usart.h"
extern volatile float dbg_cmd_vision_target_x;
extern volatile float dbg_cmd_vision_target_y;
#endif

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

// static referee_info_t* referee_data; // 用于获取裁判系统的数据
// static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static float vt_trans_lf, vt_trans_rf, vt_trans_lb, vt_trans_rb;
static float chassis_ff_current_lf, chassis_ff_current_rf, chassis_ff_current_lb, chassis_ff_current_rb;

static PIDInstance heading_pid;                               // 前向保持PID实例
static PID_Init_Config_s heading_pid_config;                  // PID配置(文件级存储,供模式切入时重新初始化)
static chassis_mode_e last_chassis_mode = CHASSIS_ZERO_FORCE; // 检测模式切换,用于PID复位

// Jeffrey070318增加：底盘接收链路LiveWatch变量，用于判断CMD发布的底盘命令是否被chassis收到。
volatile uint32_t dbg_chassis_task_loop_cnt = 0;
volatile uint32_t dbg_chassis_msg_cnt = 0;
volatile uint8_t dbg_chassis_mode = 0;
volatile float dbg_chassis_recv_vx = 0.0f;
volatile float dbg_chassis_recv_vy = 0.0f;
volatile float dbg_chassis_recv_wz = 0.0f;
volatile float dbg_chassis_recv_offset_angle = 0.0f;
volatile float dbg_chassis_final_wz = 0.0f;
volatile float dbg_chassis_vt_lf = 0.0f;
volatile float dbg_chassis_vt_rf = 0.0f;
volatile float dbg_chassis_vt_lb = 0.0f;
volatile float dbg_chassis_vt_rb = 0.0f;
volatile float dbg_chassis_fb_speed_lf = 0.0f;
volatile float dbg_chassis_fb_speed_rf = 0.0f;
volatile float dbg_chassis_fb_speed_lb = 0.0f;
volatile float dbg_chassis_fb_speed_rb = 0.0f;
volatile int16_t dbg_chassis_fb_current_lf = 0;
volatile int16_t dbg_chassis_fb_current_rf = 0;
volatile int16_t dbg_chassis_fb_current_lb = 0;
volatile int16_t dbg_chassis_fb_current_rb = 0;
volatile uint16_t dbg_chassis_fb_ecd_lf = 0;
volatile uint16_t dbg_chassis_fb_ecd_rf = 0;
volatile uint16_t dbg_chassis_fb_ecd_lb = 0;
volatile uint16_t dbg_chassis_fb_ecd_rb = 0;

static void ChassisUpdateMotorDebug()
{
    if (motor_lf != NULL)
    {
        dbg_chassis_fb_speed_lf = motor_lf->measure.speed_aps;
        dbg_chassis_fb_current_lf = motor_lf->measure.real_current;
        dbg_chassis_fb_ecd_lf = motor_lf->measure.ecd;
    }
    if (motor_rf != NULL)
    {
        dbg_chassis_fb_speed_rf = motor_rf->measure.speed_aps;
        dbg_chassis_fb_current_rf = motor_rf->measure.real_current;
        dbg_chassis_fb_ecd_rf = motor_rf->measure.ecd;
    }
    if (motor_lb != NULL)
    {
        dbg_chassis_fb_speed_lb = motor_lb->measure.speed_aps;
        dbg_chassis_fb_current_lb = motor_lb->measure.real_current;
        dbg_chassis_fb_ecd_lb = motor_lb->measure.ecd;
    }
    if (motor_rb != NULL)
    {
        dbg_chassis_fb_speed_rb = motor_rb->measure.speed_aps;
        dbg_chassis_fb_current_rb = motor_rb->measure.real_current;
        dbg_chassis_fb_ecd_rb = motor_rb->measure.ecd;
    }
}

#if ROBOT_ENABLE_VOFA_CHASSIS_DEBUG
static void ChassisVofaSendDebug()
{
    static uint16_t vofa_divider = 0;
    float vofa_data[17];

    vofa_divider++;
    if (vofa_divider < ROBOT_VOFA_CHASSIS_DEBUG_DIVIDER)
        return;
    vofa_divider = 0;

    vofa_data[0] = -chassis_cmd_recv.vx;
    vofa_data[1] = -chassis_cmd_recv.vy;
    vofa_data[2] = -chassis_cmd_recv.wz;
    vofa_data[3] = -vt_lf;
    vofa_data[4] = -vt_rf;
    vofa_data[5] = -vt_lb;
    vofa_data[6] = -vt_rb;
    vofa_data[7] = dbg_chassis_fb_speed_lf;
    vofa_data[8] = dbg_chassis_fb_speed_rf;
    vofa_data[9] = dbg_chassis_fb_speed_lb;
    vofa_data[10] = dbg_chassis_fb_speed_rb;
    vofa_data[11] = (float)dbg_chassis_fb_current_lf;
    vofa_data[12] = (float)dbg_chassis_fb_current_rf;
    vofa_data[13] = (float)dbg_chassis_fb_current_lb;
    vofa_data[14] = (float)dbg_chassis_fb_current_rb;
    vofa_data[15] = dbg_cmd_vision_target_x;
    vofa_data[16] = dbg_cmd_vision_target_y;

    (void)vofa_justfloat_output_dma(vofa_data, 17u, &huart7);
}
#endif

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            // Jeffrey070318修改：底盘速度环参数改为使用robot_def.h中R1/R2独立宏，调参只改对应车种参数。
            .speed_PID = {
                .Kp = CHASSIS_SPEED_PID_KP,
                .Ki = CHASSIS_SPEED_PID_KI,
                .Kd = CHASSIS_SPEED_PID_KD,
                .IntegralLimit = CHASSIS_SPEED_PID_INTEGRAL_LIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = CHASSIS_SPEED_PID_MAX_OUT,
            },
            // Jeffrey070318修改：底盘电流环参数改为使用robot_def.h中R1/R2独立宏。
            .current_PID = {
                .Kp = CHASSIS_CURRENT_PID_KP,
                .Ki = CHASSIS_CURRENT_PID_KI,
                .Kd = CHASSIS_CURRENT_PID_KD,
                .IntegralLimit = CHASSIS_CURRENT_PID_INTEGRAL_LIMIT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = CHASSIS_CURRENT_PID_MAX_OUT,
            },
            .current_feedforward_ptr = &chassis_ff_current_lf,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        // Jeffrey070318修改：底盘电机型号改走robot_def.h，R1/R2可独立配置。
        .motor_type = CHASSIS_MOTOR_TYPE,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    // Jeffrey070318修改：左前轮ID和方向改走robot_def.h，调R1/R2时不再改chassis.c。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LF_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_LF_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    // Jeffrey070318修改：右前轮ID和方向改走robot_def.h。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RF_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_RF_REVERSE;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &chassis_ff_current_rf;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    // Jeffrey070318修改：左后轮ID和方向改走robot_def.h。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_LB_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_LB_REVERSE;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &chassis_ff_current_lb;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    // Jeffrey070318修改：右后轮ID和方向改走robot_def.h。
    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RB_ID;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = CHASSIS_MOTOR_RB_REVERSE;
    chassis_motor_config.controller_param_init_config.current_feedforward_ptr = &chassis_ff_current_rb;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    //    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI

    // SuperCap_Init_Config_s cap_conf = {
    //     .can_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 0x302, // 超级电容默认接收id
    //         .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    //     }};
    // cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

    // 前向保持PID初始化
    // Jeffrey070318修改：车头保持PID改走robot_def.h，R1/R2底盘参数分开调。
    heading_pid_config = (PID_Init_Config_s){
        .Kp = CHASSIS_HEADING_PID_KP,
        .Ki = CHASSIS_HEADING_PID_KI,
        .Kd = CHASSIS_HEADING_PID_KD,
        .MaxOut = CHASSIS_HEADING_PID_MAX_OUT,
        .DeadBand = CHASSIS_HEADING_PID_DEADBAND,
        .IntegralLimit = CHASSIS_HEADING_PID_INTEGRAL_LIMIT,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_Trapezoid_Intergral,
    };
    PIDInit(&heading_pid, &heading_pid_config);
}

// #define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// /**
//  * @brief 计算每个轮毂电机的输出,正运动学解算
//  *        用宏进行预替换减小开销,运动解算具体过程参考教程
//  */
// static void MecanumCalculate()
// {
//     vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
//     vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
//     vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
//     vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
// }
/* 移除旧宏定义 */
// #define FRONT_ARM_X  ((+HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_X) * DEGREE_2_RAD)
// #define BACK_ARM_X   ((-HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_X) * DEGREE_2_RAD)
// #define LEFT_ARM_Y   ((-HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
// #define RIGHT_ARM_Y  ((+HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/* 新增X型布局的旋转力臂系数，由前后半轴距与左右半轮距之和乘角度转换 */
#define ROTATION_RADIUS ((HALF_WHEEL_BASE + HALF_TRACK_WIDTH) * DEGREE_2_RAD)

/**
 * @brief 四全向轮（X型布局，四角安装）正运动学解算
 * 坐标系 x：前为正；y：右为正；wz：逆时针为正
 *
 * 轮子编号：左前(lf)、右前(rf)、左后(lb)、右后(rb)
 * 每个全向轮的驱动力方向配置如下（与x轴夹角）：
 *   lf : 45°  -> 获得 vx+vy 分量
 *   rf : 135° -> 获得 -vx+vy 分量
 *   lb : 315° -> 获得 vx-vy 分量
 *   rb : 225° -> 获得 -vx-vy 分量
 * 所有轮子对旋转的贡献系数相等，为 (HALF_WHEEL_BASE + HALF_TRACK_WIDTH) * DEGREE_2_RAD
 */
static void MecanumCalculate()
{
    float vy = chassis_vx;
    float vx = chassis_vy;
    float wz = chassis_cmd_recv.wz;

    vt_trans_lf = vx + vy;
    vt_trans_rf = -vx + vy;
    vt_trans_lb = vx - vy;
    vt_trans_rb = -vx - vy;

    vt_lf = vt_trans_lf + ROTATION_RADIUS * wz;
    vt_rf = vt_trans_rf + ROTATION_RADIUS * wz;
    vt_lb = vt_trans_lb + ROTATION_RADIUS * wz;
    vt_rb = vt_trans_rb + ROTATION_RADIUS * wz;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    // 前馈只使用平移分量，避免车头锁定/ yaw 微调时 wz 分量被前馈放大。
    chassis_ff_current_lf = CHASSIS_SPEED_FEEDFORWARD_KV * vt_trans_lf * ((CHASSIS_MOTOR_LF_REVERSE == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f);
    chassis_ff_current_rf = CHASSIS_SPEED_FEEDFORWARD_KV * vt_trans_rf * ((CHASSIS_MOTOR_RF_REVERSE == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f);
    chassis_ff_current_lb = CHASSIS_SPEED_FEEDFORWARD_KV * vt_trans_lb * ((CHASSIS_MOTOR_LB_REVERSE == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f);
    chassis_ff_current_rb = CHASSIS_SPEED_FEEDFORWARD_KV * vt_trans_rb * ((CHASSIS_MOTOR_RB_REVERSE == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f);
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // Jeffrey070318增加：底盘任务循环计数，确认ChassisTask是否持续运行。
    dbg_chassis_task_loop_cnt++;
    ChassisUpdateMotorDebug();
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    if (SubGetMessage(chassis_sub, &chassis_cmd_recv))
    {
        // Jeffrey070318增加：记录底盘收到的CMD命令，判断消息中心链路是否正常。
        dbg_chassis_msg_cnt++;
        dbg_chassis_recv_vx = chassis_cmd_recv.vx;
        dbg_chassis_recv_vy = chassis_cmd_recv.vy;
        dbg_chassis_recv_wz = chassis_cmd_recv.wz;
        dbg_chassis_recv_offset_angle = chassis_cmd_recv.offset_angle;
        dbg_chassis_mode = (uint8_t)chassis_cmd_recv.chassis_mode;
    }
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        // Jeffrey070318修改：零力模式下清空四轮目标并直接返回，避免后续轮速解算再次写入速度参考。
        vt_lf = 0.0f;
        vt_rf = 0.0f;
        vt_lb = 0.0f;
        vt_rb = 0.0f;
        dbg_chassis_final_wz = 0.0f;
        dbg_chassis_vt_lf = vt_lf;
        dbg_chassis_vt_rf = vt_rf;
        dbg_chassis_vt_lb = vt_lb;
        dbg_chassis_vt_rb = vt_rb;
        EstimateSpeed();
#ifdef ONE_BOARD
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
        CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
#if ROBOT_ENABLE_VOFA_CHASSIS_DEBUG
        ChassisVofaSendDebug();
#endif
        return;
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘自由旋转全向机动，wz已在robot_cmd中设置好,不需要单独设置pid,以wz为输入直接输出速度
        // chassis_cmd_recv.wz = 0;

        break;
    case CHASSIS_KEEP_FRONT: // 保持前向,PID控制wz,限幅±2000(100%摇杆最大)
    {
        if (last_chassis_mode != CHASSIS_KEEP_FRONT)
        {
            PIDInit(&heading_pid, &heading_pid_config); // 重新切入时复位积分和历史误差
        }
        float angle_wz = PIDCalculate(&heading_pid, 0.0f, chassis_cmd_recv.offset_angle);
        float gyro_wz = -CHASSIS_HEADING_GYRO_DAMP * chassis_cmd_recv.yaw_rate;
        chassis_cmd_recv.wz = float_constrain(angle_wz + gyro_wz,
                                              -CHASSIS_HEADING_PID_MAX_OUT,
                                              CHASSIS_HEADING_PID_MAX_OUT);
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 2000;
        break;
    default:
        break;
    }

    // 记录本周期模式,供下周期检测模式切换(PID复位等)
    last_chassis_mode = chassis_cmd_recv.chassis_mode;
    dbg_chassis_final_wz = chassis_cmd_recv.wz;

    // 没有云台，底盘前方就是正方向，遥控器输入直接映射
    // Jeffrey070318增加：底盘方向修正，正负由robot_def.h中CHASSIS_VX/VY_DIRECTION宏控制
    chassis_vx = chassis_cmd_recv.vx * CHASSIS_VX_DIRECTION;
    chassis_vy = chassis_cmd_recv.vy * CHASSIS_VY_DIRECTION;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();
    // Jeffrey070318增加：记录底盘四轮解算输出，判断命令是否最终转成电机参考值。
    dbg_chassis_vt_lf = vt_lf;
    dbg_chassis_vt_rf = vt_rf;
    dbg_chassis_vt_lb = vt_lb;
    dbg_chassis_vt_rb = vt_rb;

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
#if ROBOT_ENABLE_VOFA_CHASSIS_DEBUG
    ChassisVofaSendDebug();
#endif
}
