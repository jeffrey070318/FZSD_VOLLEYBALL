#include "bsp_init.h"
#include "robot.h"
#include "robot_task.h"
#include "robot_def.h"
#include "chassis.h"
#include "delta.h"
#include "serve.h"
#include "robot_cmd.h"

// Jeffrey070318增加：Robot层初始化/任务运行LiveWatch变量，只在application层定位任务链路是否跑通。
volatile uint32_t dbg_robot_init_stage = 0;
volatile uint32_t dbg_robot_task_loop_cnt = 0;
volatile uint32_t dbg_robot_cmd_task_call_cnt = 0;

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    dbg_robot_init_stage = 1; // Jeffrey070318增加：进入RobotInit，已关闭中断准备初始化BSP。
    BSPInit();
    dbg_robot_init_stage = 2; // Jeffrey070318增加：BSPInit完成，后续开始初始化各app。

    // Jeffrey070318增加：底盘是R1/R2共用部分，ONE_BOARD/CHASSIS_BOARD下都初始化底盘应用。
#if R2_DEBUG_ENABLE_CHASSIS_APP && (defined(ONE_BOARD) || defined(CHASSIS_BOARD))
    ChassisInit();
    dbg_robot_init_stage = 3; // Jeffrey070318增加：底盘app初始化完成。
#endif

#if R2_DEBUG_ENABLE_DELTA_APP
    DeltaInit();
    dbg_robot_init_stage = 4; // Jeffrey070318增加：Delta机械臂app初始化完成。
#endif
#if ROBOT_HAS_SERVE
    // Jeffrey070318修改：只有R1包含发球拨杆Serve应用，R2只初始化接球机械臂Delta。
    ServeInit();
    dbg_robot_init_stage = 5; // Jeffrey070318增加：R1发球拨杆app初始化完成。
#endif
// Jeffrey070318修改：进入遥控器整车测试阶段，按宏开关恢复CMD初始化。
#if R2_DEBUG_ENABLE_CMD_APP
    RobotCMDInit();
    dbg_robot_init_stage = 6; // Jeffrey070318增加：CMD初始化完成，遥控器注册应已被调用。
#endif

    OSTaskInit(); // 创建基础任务
    dbg_robot_init_stage = 7; // Jeffrey070318增加：FreeRTOS应用任务创建完成。

    // 初始化完成,开启中断
    __enable_irq();
    dbg_robot_init_stage = 8; // Jeffrey070318增加：RobotInit完成并重新打开中断。
}

void RobotTask()
{
    dbg_robot_task_loop_cnt++; // Jeffrey070318增加：确认RobotTask是否被FreeRTOS周期调用。
// Jeffrey070318修改：CMD启用时由遥控器统一下发底盘和机械臂命令。
#if R2_DEBUG_ENABLE_CMD_APP
    dbg_robot_cmd_task_call_cnt++; // Jeffrey070318增加：确认RobotTask确实调用到CMD任务入口。
    RobotCMDTask();
#endif

#if R2_DEBUG_ENABLE_CHASSIS_APP && (defined(ONE_BOARD) || defined(CHASSIS_BOARD))
    ChassisTask();
#endif

#if R2_DEBUG_ENABLE_DELTA_APP
    DeltaTask();
#endif
#if ROBOT_HAS_SERVE
    // Jeffrey070318修改：只有R1运行Serve任务，避免R2访问不存在的发球机构。
    ServeTask();
#endif
}
