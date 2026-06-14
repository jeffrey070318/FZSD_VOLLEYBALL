#include "bsp_init.h"
#include "robot.h"
#include "robot_task.h"
#include "robot_def.h" //Jeffrey070318增加：使用板级条件编译决定当前整车需要初始化哪些app。
#include "chassis.h" //Jeffrey070318增加：接入YYP底盘app，使cmd发布的chassis_cmd被底盘消费。
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

    //Jeffrey070318增加：底盘是R1/R2共用部分，ONE_BOARD/CHASSIS_BOARD下都初始化底盘应用。
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    DeltaInit();
    ServeInit();
    RobotCMDInit();

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
    //Jeffrey070318修改：cmd先生成本周期控制量，再由底盘和机构应用消费，减少一周期控制滞后。
    RobotCMDTask();

    //Jeffrey070318增加：底盘任务接收cmd速度和模式命令，完成四轮运动解算。
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif

    DeltaTask();
    ServeTask();
}
