#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "Stepper_Motor.h"
#include "PC_Comm.h"

#define CHASSIS_WHEEL_R         0.09f       /* 轮半径 [m] (直径18cm) */
#define CHASSIS_TRACK_WIDTH     0.37f       /* 轮距(左右轮间距) [m] (37cm) */

#define CHASSIS_MAX_V           0.4f        /* 最大线速度 [m/s] */
#define CHASSIS_MAX_W           3.1416f     /* 最大角速度 [rad/s] (一圈≈2s) */

#define WHEEL_RAD_TO_RPM        (60.0f / (2.0f * 3.1415926f * CHASSIS_WHEEL_R))  /* 线速度→转速 [m/s → rpm] */

#define WHEEL_L  0
#define WHEEL_R  1
#define LIFT     2     /* 丝杆抬升电机索引 */

/* ---- 丝杆抬升参数 (单位: mm) — TODO: 实测后修改 ---- */
#define LEAD_SCREW_PITCH        2.07f       /* 丝杆导程 [mm/rev] — 实测: 10圈=20.7mm */
#define LIFT_GEAR_RATIO         13.7f       /* 减速比 13.7:1 (电机侧:输出侧) */
#define LIFT_MIN_HEIGHT         0.0f        /* 最小高度 [mm] — 零点位置 — TODO: 实测 */
#define LIFT_MAX_HEIGHT         100.6f      /* 最大高度 [mm] — 实测: 总行程100.6mm */
#define LIFT_SPEED              2500         /* 抬升速度 [RPM] — TODO: 实测，范围0-3000rpm */
#define LIFT_ACCEL              150          /* 抬升加速度 — TODO: 实测 */
#define LIFT_MANUAL_MAX_SPEED   15.0f        /* 手动控制最大抬升速率 [mm/s] */
#define LIFT_MOTOR_ADDR         3           /* 抬升电机地址 */

/* 高度 ↔ 电机角度 换算 (含减速比) */
#define HEIGHT_TO_ANGLE(h)      ((h) / LEAD_SCREW_PITCH * 360.0f * LIFT_GEAR_RATIO)
#define ANGLE_TO_HEIGHT(a)      ((a) * LEAD_SCREW_PITCH / 360.0f / LIFT_GEAR_RATIO)

typedef enum
{
    CHASSIS_MODE_RC  = 0,   /* 遥控器模式 */
    CHASSIS_MODE_NAV = 1,   /* 导航模式(上位机) */
} Chassis_Mode_e;

typedef struct
{
    Stepper_Motor_Info_Typedef Motor[3];  /* 0=左轮, 1=右轮, 2=丝杆抬升 */
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
    /* ---- 丝杆抬升 ---- */
    float lift_target_height;     /* 目标高度 [mm] */
    float lift_cur_height;        /* 当前高度 [mm] (编码器反馈) */
    float lift_bus_voltage;       /* 丝杆电机总线电压 [mV] */
    uint8_t lift_homing;          /* 回零中标志 */
} Chassis_Info_Typedef;

extern Chassis_Info_Typedef Chassis;

void chassis_task(void);
void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);
void Chassis_Stop(Chassis_Info_Typedef *chassis);
void Chassis_Init(Chassis_Info_Typedef *chassis);
void Chassis_Lift_Home(Chassis_Info_Typedef *chassis);
void Chassis_Lift_Set_Height(Chassis_Info_Typedef *chassis, float height_mm);

#endif
