#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

/**
 * @brief 底盘单模块直测任务,用于绕过CMD直接给底盘固定速度.
 *
 */
// Jeffrey070318增加：提供底盘直测入口，打开CHASSIS_DIRECT_TEST后由RobotTask直接调用。
void ChassisDirectTestTask(void);

#endif // CHASSIS_H
