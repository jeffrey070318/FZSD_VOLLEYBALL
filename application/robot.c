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
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    DeltaInit();
#if ROBOT_HAS_SERVE
    //Jeffrey070318修改：只有R1包含发球拨杆Serve应用，R2只初始化接球机械臂Delta。
    ServeInit();
#endif
    RobotCMDInit();

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{

    RobotCMDTask();

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif

    DeltaTask();
#if ROBOT_HAS_SERVE
    //Jeffrey070318修改：只有R1运行Serve任务，避免R2访问不存在的发球机构。
    ServeTask();
#endif
}
