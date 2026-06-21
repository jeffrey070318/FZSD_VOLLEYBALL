#include "bsp_init.h"
#include "robot.h"
#include "robot_task.h"
#include "robot_def.h"
#include "chassis.h"
#include "delta.h"
#include "serve.h"
#include "robot_cmd.h"

void RobotInit()
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    BSPInit();

    // Jeffrey070318增加：底盘是R1/R2共用部分，ONE_BOARD/CHASSIS_BOARD下都初始化底盘应用。
#if R2_DEBUG_ENABLE_CHASSIS_APP && (defined(ONE_BOARD) || defined(CHASSIS_BOARD))
    ChassisInit();
#endif

#if R2_DEBUG_ENABLE_DELTA_APP
    DeltaInit();
#endif
#if ROBOT_HAS_SERVE
    // Jeffrey070318修改：只有R1包含发球拨杆Serve应用，R2只初始化接球机械臂Delta。
    ServeInit();
#endif
    // Jeffrey070318临时调整：R2机械臂调参阶段CMD和光流链路待大改，且光流硬件未接；先停用RobotCMD初始化，避免USART/DMA初始化HardFault。
    // RobotCMDInit();

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    // Jeffrey070318临时调整：CMD待重写，R2当前只保留底盘/Delta等模块任务运行，后续恢复RobotCMDTask。
    // RobotCMDTask();

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
