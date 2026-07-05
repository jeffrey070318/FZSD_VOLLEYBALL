/**
 * @file remote_control.c
 * @brief  遥控器模块,富斯i6x遥控器数据解析模块,适配SBUS协议,美国手映射,无拨轮,SWB/SWC开关适配
 *         Jeffrey070318修改：保留CH1-CH16调试解析，CH5/CH6/CH7/CH8分别作为左一/左二/右一/右二拨杆。
 *
 */
#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

// #define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小

#define REMOTE_CONTROL_FRAME_SIZE 25u // SBUS帧大小25字节（富斯IA6B）
#define SBUS_CHANNEL_NUM 16u          // Jeffrey070318增加：SBUS标准最多16个模拟通道，调试拨码时全部解析出来。
#define SBUS_SWITCH_THRESHOLD 500     // 两态拨杆阈值，通道过零附近保持上一次状态，防止切换瞬间抖动。

// #define SBUS_STICK_RANGE 1024   // ch[i] = raw - offset 后的理论幅值
#define SBUS_STICK_RANGE 784 // ch[i] = raw - offset 后的实测值
#define RC_STICK_RANGE 660   // 原框架DT7遥控器摇杆幅值
// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

// Jeffrey070318增加：遥控器接收链路LiveWatch变量，用于判断DMA/IDLE回调是否进入。
volatile uint32_t dbg_rc_rx_callback_cnt = 0;
volatile uint32_t dbg_rc_parse_cnt = 0;
volatile uint32_t dbg_rc_lost_cnt = 0;
volatile uint16_t dbg_rc_recv_len = 0;
volatile uint8_t dbg_rc_frame_head = 0;
volatile uint8_t dbg_rc_frame_tail = 0;
volatile uint8_t dbg_rc_online = 0;
volatile int16_t dbg_rc_raw_ch[SBUS_CHANNEL_NUM] = {0}; // Jeffrey070318修改：原始通道监视扩展到CH1-CH16，确认拨码真实输出通道。
volatile int16_t dbg_rc_rocker_l_ = 0;
volatile int16_t dbg_rc_rocker_l1 = 0;
volatile int16_t dbg_rc_rocker_r_ = 0;
volatile int16_t dbg_rc_rocker_r1 = 0;
volatile uint8_t dbg_rc_switch_left1 = 0;
volatile uint8_t dbg_rc_switch_left2 = 0;
volatile uint8_t dbg_rc_switch_right1 = 0;
volatile uint8_t dbg_rc_switch_right2 = 0;
volatile uint8_t dbg_rc_switch_left = 0;
volatile uint8_t dbg_rc_switch_right = 0;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > RC_STICK_RANGE)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

// Jeffrey070318增加：通用SBUS 11bit通道解析，避免手写高通道位偏移时出错。
static int16_t SBUSReadChannel(const uint8_t *sbus_buf, uint8_t channel_index)
{
    uint16_t bit_index = (uint16_t)channel_index * 11u;
    uint8_t byte_index = 1u + (uint8_t)(bit_index / 8u); // 跳过SBUS帧头0x0F，从数据区开始。
    uint8_t bit_offset = (uint8_t)(bit_index % 8u);

    uint32_t raw = ((uint32_t)sbus_buf[byte_index] >> bit_offset) |
                   ((uint32_t)sbus_buf[byte_index + 1u] << (8u - bit_offset)) |
                   ((uint32_t)sbus_buf[byte_index + 2u] << (16u - bit_offset));

    return (int16_t)((raw & 0x07FFu) - RC_CH_VALUE_OFFSET);
}

static uint8_t SBUSSwitchFromChannel(int16_t channel, uint8_t last_state)
{
    if (channel > SBUS_SWITCH_THRESHOLD)
        return RC_SW_DOWN;
    if (channel < -SBUS_SWITCH_THRESHOLD)
        return RC_SW_UP;

    if (last_state == RC_SW_DOWN || last_state == RC_SW_UP)
        return last_state;

    return RC_SW_UP;
}

// /**
//  * @brief 遥控器数据解析
//  *
//  * @param sbus_buf 接收buffer
//  */
// static void sbus_to_rc(const uint8_t *sbus_buf)
// {
//     // 摇杆,直接解算时减去偏置
//     rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
//     rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
//     rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
//     rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
//     rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
//     RectifyRCjoystick();
//     // 开关,0左1右
//     rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
//     rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

//     // 鼠标解析
//     rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
//     rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
//     rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
//     rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

//     //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
//     *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
//     if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
//         rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
//     else
//         memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
//     if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
//         rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
//     else
//         memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

//     uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
//         key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
//         key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
//         key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
//         key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
//         key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

//     for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
//     {
//         if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
//             continue;
//         // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
//         if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
//             rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
//         // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
//         if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
//             rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
//         // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
//         if ((key_with_shift & j) && !(key_last_with_shift & j))
//             rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
//     }

//     memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
// }

/**
 * @brief 富斯i6x遥控器数据sbus解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // ========== 1. 校验SBUS起始字节（可选，增强鲁棒性） ==========
    // if (sbus_buf[0] != 0x0F) { // SBUS起始字节为0x0F
    //     memset(&rc_ctrl[TEMP], 0, sizeof(RC_ctrl_t));
    //     memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
    //     return;
    // }

    // ========== 2. SBUS通道解析（富斯i6x美国手映射） ==========
    // 通道映射（富斯i6x美国手）：
    // Ch1 → 右摇杆水平（rocker_r_） | Ch2 → 右摇杆竖直（rocker_r1）
    // Ch3 → 左摇杆竖直（rocker_l1） | Ch4 → 左摇杆水平（rocker_l_）
    // Jeffrey070318修改：统一解析CH1-CH16，CH5/CH6/CH7/CH8用于左一/左二/右一/右二拨码。
    int16_t ch[SBUS_CHANNEL_NUM] = {0};
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUM; i++) // Jeffrey070318修改：LiveWatch同步显示CH1-CH16。
    {
        ch[i] = SBUSReadChannel(sbus_buf, i);
        dbg_rc_raw_ch[i] = ch[i];
    }
    // Jeffrey070318修改：CH1-CH8按YYP验证过的公式显式解析，避免通用解析影响当前遥控器排查。
    ch[0] = ((sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[1] = (((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[2] = (((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) | (sbus_buf[5] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[3] = (((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[4] = (((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[5] = (((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[6] = (((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    ch[7] = (((sbus_buf[10] >> 5) | (sbus_buf[11] << 3) | (sbus_buf[12] << 11)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    for (uint8_t i = 0; i < 8; i++) // Jeffrey070318修改：前八通道调试值同步为YYP公式结果，便于和学长工程直接对比。
    {
        dbg_rc_raw_ch[i] = ch[i];
    }

    // ========== 3. 将SBUS中-1024~+1024映射回原框架中的-660~+660 ==========

    for (uint8_t i = 0; i < 4; i++)
    {
        // 这里把限幅关了是因为实测发现使用过程中某些通道会出现尖峰（原因不明）
        // 尖峰值往往大于实测最大幅值，如果直接限幅到实测最大幅值会导致尖峰值被剪切成正常值
        // 因此采用大于实测最大幅值直接裁剪的方式，见RectifyRCjoystick();
        //  1) 输入限幅到 [-1024, +1024]（防止接收机输出略超范围）
        //  if (ch[i] >  SBUS_STICK_RANGE) ch[i] =  SBUS_STICK_RANGE;
        //  if (ch[i] < -SBUS_STICK_RANGE) ch[i] = -SBUS_STICK_RANGE;

        // 2) 线性缩放：ch[i] * 660 / 1024，并做四舍五入
        int32_t tmp = (int32_t)ch[i] * RC_STICK_RANGE;

        // 四舍五入：正数 +512，负数 -512
        if (tmp >= 0)
            tmp += (SBUS_STICK_RANGE / 2);
        else
            tmp -= (SBUS_STICK_RANGE / 2);

        tmp /= SBUS_STICK_RANGE;

        // 3) 输出再限幅到 [-660, +660]
        // if (tmp >  RC_STICK_RANGE) tmp =  RC_STICK_RANGE;
        // if (tmp < -RC_STICK_RANGE) tmp = -RC_STICK_RANGE;

        ch[i] = (int16_t)tmp;

        // 4) 死区: 限幅后绝对值小于10则置零,消除摇杆中位抖动
        if (ch[i] < 10 && ch[i] > -10)
            ch[i] = 0;
    }
    // 赋值到rc_ctrl
    rc_ctrl[TEMP].rc.rocker_r_ = ch[0]; // 右水平
    rc_ctrl[TEMP].rc.rocker_r1 = ch[1]; // 右竖直
    rc_ctrl[TEMP].rc.rocker_l1 = ch[2]; // 左竖直
    rc_ctrl[TEMP].rc.rocker_l_ = ch[3]; // 左水平
    rc_ctrl[TEMP].rc.dial = 0;          // 富斯i6x无拨轮，置0
    // Jeffrey070318增加：保存缩放后的摇杆值，判断解析结果是否进入CMD。
    dbg_rc_rocker_r_ = rc_ctrl[TEMP].rc.rocker_r_;
    dbg_rc_rocker_r1 = rc_ctrl[TEMP].rc.rocker_r1;
    dbg_rc_rocker_l1 = rc_ctrl[TEMP].rc.rocker_l1;
    dbg_rc_rocker_l_ = rc_ctrl[TEMP].rc.rocker_l_;

    // ========== 4. 开关解析（CH5/CH6/CH7/CH8分别为左一/左二/右一/右二） ==========
    // 两态拨杆只输出UP/DOWN；通道切换过零附近保持上一次状态，不输出MID。
    rc_ctrl[TEMP].rc.switch_left1 = SBUSSwitchFromChannel(ch[4], rc_ctrl[LAST].rc.switch_left1);
    rc_ctrl[TEMP].rc.switch_left2 = SBUSSwitchFromChannel(ch[5], rc_ctrl[LAST].rc.switch_left2);
    rc_ctrl[TEMP].rc.switch_right1 = SBUSSwitchFromChannel(ch[6], rc_ctrl[LAST].rc.switch_right1);
    rc_ctrl[TEMP].rc.switch_right2 = SBUSSwitchFromChannel(ch[7], rc_ctrl[LAST].rc.switch_right2);
    rc_ctrl[TEMP].rc.switch_left = rc_ctrl[TEMP].rc.switch_left1;
    rc_ctrl[TEMP].rc.switch_right = rc_ctrl[TEMP].rc.switch_right1;

    // Jeffrey070318增加：保存开关解析结果，排查四个拨杆通道映射是否正确。
    dbg_rc_switch_left1 = rc_ctrl[TEMP].rc.switch_left1;
    dbg_rc_switch_left2 = rc_ctrl[TEMP].rc.switch_left2;
    dbg_rc_switch_right1 = rc_ctrl[TEMP].rc.switch_right1;
    dbg_rc_switch_right2 = rc_ctrl[TEMP].rc.switch_right2;
    dbg_rc_switch_left = rc_ctrl[TEMP].rc.switch_left;
    dbg_rc_switch_right = rc_ctrl[TEMP].rc.switch_right;

    // ========== 5. 鼠标/键盘置0（SBUS无该数据） ==========
    memset(&rc_ctrl[TEMP].mouse, 0, sizeof(rc_ctrl[TEMP].mouse));         // 鼠标置0
    memset(&rc_ctrl[TEMP].key, 0, sizeof(rc_ctrl[TEMP].key));             // 键盘置0
    memset(&rc_ctrl[TEMP].key_count, 0, sizeof(rc_ctrl[TEMP].key_count)); // 按键计数置0

    // ========== 5. 摇杆矫正（超过±660置0） ==========
    RectifyRCjoystick();

    // ========== 6. 保存上一次数据（保留原逻辑） ==========
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
    // Jeffrey070318增加：解析计数自增，用于确认sbus_to_rc是否持续运行。
    dbg_rc_parse_cnt++;
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    // Jeffrey070318增加：记录串口回调次数/帧头/长度，用于确认遥控器DMA接收是否正常。
    dbg_rc_rx_callback_cnt++;
    dbg_rc_recv_len = rc_usart_instance->recv_len;
    dbg_rc_frame_head = rc_usart_instance->recv_buff[0];
    dbg_rc_frame_tail = rc_usart_instance->recv_buff[REMOTE_CONTROL_FRAME_SIZE - 1];
    if (rc_daemon_instance != NULL)
        DaemonReload(rc_daemon_instance);     // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    uint8_t head = rc_usart_instance->recv_buff[0];
    (void)head; // Jeffrey070318增加：保留head局部观察位，当前逻辑不使用，避免编译告警。
    // Jeffrey070318增加：遥控器离线计数，判断是否被daemon反复清零。
    dbg_rc_lost_cnt++;
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
    {
        // Jeffrey070318增加：保存在线状态，便于LiveWatch直接观察遥控器daemon状态。
        dbg_rc_online = DaemonIsOnline(rc_daemon_instance);
        return dbg_rc_online;
    }
    dbg_rc_online = 0;
    return 0;
}
