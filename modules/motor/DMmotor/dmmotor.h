#ifndef DMMOTOR_H
#define DMMOTOR_H

#include "fdcan.h"
#include "bsp_can.h"
#include "stdint.h"
#include "general_def.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		    0x200
#define POSI_MODE			0x300

#define DISABLE_STATE		        0x00
#define ENABLE_STATE		        0x01
#define OVERVOLTAGE_STATE	        0x08
#define UNDERVOLTAGE_STATE	        0x09
#define OVERCURRENT_STATE	        0x0A
#define MOS_OVER_TEMP_STATE	        0x0B
#define COIL_OVER_TEMPERATURE_STATE	0x0C
#define COMMUNICATION_LOSS_STATE	0x0D
#define OVERLOAD_STATE	            0x0E

#define P_MIN -12.56637f
#define P_MAX 12.56637f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -30.0f
#define T_MAX 30.0f

typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;

    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;

    float t_mos;        //驱动上mos的平均温度
    float t_rotor;      //电机内部线圈的平均温度

    uint32_t last_fdb_time;  //上次反馈时间
} motor_fbpara_t;

typedef struct 
{
    /* data */
    uint16_t mode;
    motor_fbpara_t para;
} Joint_Motor_t;

/********************function********************/

void Clear_Error(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void Enable_Motor_Mode(hcan_t* hcan, Joint_Motor_t* motor);
void Disable_Motor_Mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
void Save_Pos_Zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

void Mit_Ctrl(hcan_t* hcan, Joint_Motor_t* motor, MIT_CTRL_DATA ctrl_data);
void Pos_Speed_Ctrl(hcan_t* hcan, Joint_Motor_t* motor, float pos, float vel);
void Speed_Ctrl(hcan_t* hcan,uint16_t motor_id, float vel);

void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
void Dm8009_Fbdata(Joint_Motor_t *motor, uint8_t *rx_data);

#endif // !DM8009_H