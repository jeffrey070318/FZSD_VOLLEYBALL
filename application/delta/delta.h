#ifndef DELTA_H
#define DELTA_H

/**
* @brief 初始化Delta机械臂
*
*/
void DeltaInit(void);

/**
 * @brief Delta机械臂任务
 * 
 */
void DeltaTask(void);

/**
 * @brief Delta机械臂动作
 * 
 */
void Delta_Motion();

/**
 * @brief Delta单模块直测任务,用于绕过CMD直接跑机械臂测试动作.
 *
 */
// Jeffrey070318增加：提供Delta直测入口，打开DELTA_DIRECT_TEST后由RobotTask直接调用。
void DeltaDirectTestTask(void);

#endif
