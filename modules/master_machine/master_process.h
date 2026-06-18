#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"

/* ---------------- Frame lengths, matching the upper-computer protocol ---------------- */
#define VISION_RECV_SIZE 22u
#define VISION_SEND_SIZE 17u

/* ---------------- Volleyball navigation protocol enums ---------------- */
typedef enum {
    MODE_IDLE   = 0,
    MODE_REMOTE = 1,
    MODE_SELF   = 2
} Robot_Mode_e;

typedef enum {
    STATE_WAITING_PLAN  = 0,
    STATE_RECEIVED_PLAN = 1,
    STATE_CATCHING      = 2,
    STATE_OVER          = 3
} Robot_State_e;

typedef enum {
    CMD_MOVE_PLAN = 0
} Plan_Cmd_e;

#pragma pack(1)

typedef struct {
    float target_x;
    float target_y;
    float target_yaw;
    float target_time;
} Vision_Recv_s;

typedef struct {
    Robot_Mode_e mode;
    Robot_State_e state;
    float robot_x;
    float robot_y;
    float robot_yaw;
} Vision_Send_s;

#pragma pack()

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
void VisionSend(Vision_Send_s *send);
uint8_t VisionIsOnline(void);

#endif // MASTER_PROCESS_H
