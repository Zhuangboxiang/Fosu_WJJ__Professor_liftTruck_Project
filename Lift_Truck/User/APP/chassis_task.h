#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "Stepper_Motor.h"
#include "PC_Comm.h"

#define CHASSIS_WHEEL_R         0.0425f       /* 轮半径 [m] (直径85mm) */
#define CHASSIS_TRACK_WIDTH     0.24f       /* 轮距(左右轮间距) [m] (240mm) */

#define CHASSIS_MAX_V           0.4f        /* 最大线速度 [m/s] */
#define CHASSIS_MAX_W           3.1416f     /* 最大角速度 [rad/s] (一圈≈2s) */

#define WHEEL_RAD_TO_RPM        (60.0f / (2.0f * 3.1415926f * CHASSIS_WHEEL_R))  /* 线速度→转速 [m/s → rpm] */

#define WHEEL_L  0
#define WHEEL_R  1

typedef enum
{
    CHASSIS_MODE_RC  = 0,   /* 遥控器模式 */
    CHASSIS_MODE_NAV = 1,   /* 导航模式(上位机) */
} Chassis_Mode_e;

typedef struct
{
    Stepper_Motor_Info_Typedef Motor[2];
    Chassis_Mode_e mode;			/* 控制模式 */
    float vx_target;				/* 目标速度 X [m/s] (遥控/上位机输入) */
    float vy_target;				/* 目标速度 Y [m/s] */
    float wz_target;				/* 目标角速度 Z [rad/s] */
    float vx_actual;				/* 实际速度 X [m/s] (编码器反馈正解) */
    float vy_actual;				/* 实际速度 Y [m/s] */
    float wz_actual;				/* 实际角速度 Z [rad/s] (编码器反馈正解) */
    float rpm_L_target;				/* 左轮目标转速 [rpm] (电机端, 已乘减速比) */
    float rpm_R_target;				/* 右轮目标转速 [rpm] (电机端, 已乘减速比) */
    PC_Speed_Typedef pc_speed;		/* 上位机通信速度(收发统一) */
    uint8_t init_flag;
} Chassis_Info_Typedef;

extern Chassis_Info_Typedef Chassis;

void chassis_task(void);
void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);
void Chassis_Stop(Chassis_Info_Typedef *chassis);
void Chassis_Init(Chassis_Info_Typedef *chassis);

#endif
