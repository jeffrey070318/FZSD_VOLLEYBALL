#ifndef SERVE_H
#define SERVE_H

/**
* @brief 初始化Serve机械臂
*
*/
void ServeInit(void);

/**
 * @brief Serve机械臂任务
 * 
 */
void ServeTask(void);

/**
 * @brief Serve单模块直测任务,用于R1绕过CMD/Delta直接测试发球拨杆.
 *
 */
// Jeffrey070318增加：提供Serve直测入口，打开SERVE_DIRECT_TEST后由RobotTask直接调用。
void ServeDirectTestTask(void);

#endif
