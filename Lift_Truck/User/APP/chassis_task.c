/**
 ******************************************************************************
 * @file    chassis_task.c
 * @brief   底盘控制任务 (差速轮模型)
 *
 *  架构说明:
 *  - 每种控制模式对应一个独立的 Mode_Handler 函数
 *  - 主循环通过 Chassis_Mode_Dispatch 根据当前模式分派到对应 handler
 *  - 速度来源: RC模式由PS2任务设置 | NAV模式由上位机设置
 *
 *  +------------------+      +-----------------------+
 *  |  chassis_task    | ---> | Chassis_Mode_Dispatch |
 *  +------------------+      +-----------------------+
 *                                     |
 *              +----------------------+----------------------+
 *              |                      |                      |
 *     CHASSIS_MODE_RC       CHASSIS_MODE_NAV          (default)
 *              |                      |                      |
 *     RC_Mode_Handler       NAV_Mode_Handler        Chassis_Stop
 *   (PS2任务直接设速)     (上位机速度→SetSpeed)   (安全停车)
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "PC_Comm.h"

/* Private define ------------------------------------------------------------*/
#define RPM_TO_MOTOR(r)     ((int16_t)((r) * Stepper_Ratio))
#define MOTOR_RPM_TO_MS(m)  ((m) / Stepper_Ratio / WHEEL_RAD_TO_RPM)  /* 电机端RPM → 轮端线速度 [m/s] */

/* Global variable -----------------------------------------------------------*/
Chassis_Info_Typedef Chassis = {
    .Motor[WHEEL_L] = {
        .Set.motor_Addr = 1,
        .Set.Firmware_v = Firmware_Emm,
    },
    .Motor[WHEEL_R] = {
        .Set.motor_Addr = 2,
        .Set.Firmware_v = Firmware_Emm,
    },
    .mode          = CHASSIS_MODE_RC,
    .vx_target     = 0,
    .vy_target     = 0,
    .wz_target     = 0,
    .vx_actual     = 0,
    .vy_actual     = 0,
    .wz_actual     = 0,
    .rpm_L_target  = 0,
    .rpm_R_target  = 0,
    .pc_speed      = {0},
    .init_flag     = 0,
};

/* Private function prototypes -----------------------------------------------*/
static void Chassis_RC_Mode_Handler(Chassis_Info_Typedef *chassis);
static void Chassis_NAV_Mode_Handler(Chassis_Info_Typedef *chassis);
static void Chassis_Mode_Dispatch(Chassis_Info_Typedef *chassis);
static void Chassis_Motor_Output(Chassis_Info_Typedef *chassis);
static void Diff_Wheel_Calc(Chassis_Info_Typedef *chassis, float vx, float vy, float wz);
static void Chassis_Actual_Calc(Chassis_Info_Typedef *chassis);

/* ---------------------------------------------------------------------------*/
/*                            Public Functions                                */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  底盘控制任务入口
 */
void chassis_task(void)
{
    uint32_t tick = 0;
    uint8_t query_idx = 0;  /* 轮询索引: 0=左轮, 1=右轮 */

    Chassis_Init(&Chassis);

    while (1)
    {
        Chassis_Mode_Dispatch(&Chassis);                         /* 按模式分发控制逻辑 */
        Chassis_Motor_Output(&Chassis);                          /* 统一输出到电机 */
        Chassis_Actual_Calc(&Chassis);                           /* 正解算: 电机反馈 → vx/wz */
       // PC_Info_Upload(Chassis.vx_actual, Chassis.vy_actual, Chassis.wz_actual);
        PC_Info_Upload(Chassis.vx_target, Chassis.vy_target, Chassis.wz_target);
        /* 每50ms轮询一个电机的反馈数据 (0x43查询) */
        if (++tick >= 50)
        {
            tick = 0;
            Stepper_Motor_Call_Info(&Chassis.Motor[query_idx], 5u);
            query_idx = (query_idx == 0) ? 1 : 0;
        }

        osDelay(1);
    }
}

/**
 * @brief  底盘初始化: 使能两路电机
 */
void Chassis_Init(Chassis_Info_Typedef *chassis)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        Stepper_Motor_Set_Cmd(&chassis->Motor[i], Stepper_Enable, 10u);
    }
    osDelay(500);
    chassis->init_flag = 1;
}

/**
 * @brief  设置底盘目标速度 (带限幅)
 */
void Chassis_Set_Velocity(Chassis_Info_Typedef *chassis, float vx, float vy, float wz)
{
    VAL_LIMIT(vx, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    VAL_LIMIT(vy, -CHASSIS_MAX_V, CHASSIS_MAX_V);
    VAL_LIMIT(wz, -CHASSIS_MAX_W, CHASSIS_MAX_W);

    chassis->vx_target = vx;
    chassis->vy_target = vy;
    chassis->wz_target = wz;
}

/**
 * @brief  底盘急停
 */
void Chassis_Stop(Chassis_Info_Typedef *chassis)
{
    Chassis_Set_Velocity(chassis, 0, 0, 0);
}

/* ---------------------------------------------------------------------------*/
/*                          Mode Dispatch Layer                               */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  根据当前模式分派到对应的模式处理函数
 * @note   新增控制模式时只需: 1)添加 case 分支  2)实现对应的 Handler
 */
static void Chassis_Mode_Dispatch(Chassis_Info_Typedef *chassis)
{
    switch (chassis->mode)
    {
        case CHASSIS_MODE_RC:
            Chassis_RC_Mode_Handler(chassis);
            break;

        case CHASSIS_MODE_NAV:
            Chassis_NAV_Mode_Handler(chassis);
            break;

        default:
            Chassis_Stop(chassis);
            break;
    }
}

/* ---------------------------------------------------------------------------*/
/*                         Mode Handler Functions                             */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  RC 遥控器模式 —— 速度由 PS2 任务通过 Chassis_Set_Velocity() 设置
 */
static void Chassis_RC_Mode_Handler(Chassis_Info_Typedef *chassis)
{
    (void)chassis;
    /* 无额外处理: PS2 任务已直接写入 vx_target/vy_target/wz_target */
}

/**
 * @brief  NAV 导航模式 —— 读取上位机下发速度并应用到底盘
 */
static void Chassis_NAV_Mode_Handler(Chassis_Info_Typedef *chassis)
{
    Chassis_Set_Velocity(chassis,
                         chassis->pc_speed.rx_vx,
                         chassis->pc_speed.rx_vy,
                         chassis->pc_speed.rx_vw);
}

/* ---------------------------------------------------------------------------*/
/*                          Motor Output Layer                                */
/* ---------------------------------------------------------------------------*/

/**
 * @brief  输出电机控制指令 (初始化完成前不输出)
 */
static void Chassis_Motor_Output(Chassis_Info_Typedef *chassis)
{
    if (chassis->init_flag == 0)
        return;

    Diff_Wheel_Calc(chassis, chassis->vx_target, chassis->vy_target, chassis->wz_target);
}

/**
 * @brief  差速轮逆运动学解算: vx [m/s], wz [rad/s] → 左右轮转速 [rpm]
 * @note   右手系: vx>0 前进,  wz>0 逆时针
 *         vy 保留但差速轮不参与横向运动计算
 *
 *   左轮线速度: vL = vx - wz * d/2
 *   右轮线速度: vR = vx + wz * d/2
 *   (d = CHASSIS_TRACK_WIDTH, 左右轮间距)
 */
static void Diff_Wheel_Calc(Chassis_Info_Typedef *chassis, float vx, float vy, float wz)
{
    (void)vy;  /* 差速轮无横向运动能力，vy 保留仅用于通信兼容 */
    float half_track = CHASSIS_TRACK_WIDTH * 0.5f;
    wz = -wz;                              /* 输入 wz 符号取反 */
    float vL = vx - wz * half_track;  /* 左轮线速度 [m/s] */
    float vR = vx + wz * half_track;  /* 右轮线速度 [m/s] */

    float rpm_L = vL * WHEEL_RAD_TO_RPM;
    float rpm_R = vR * WHEEL_RAD_TO_RPM;

    int16_t motor_L = RPM_TO_MOTOR(-rpm_L);/* 左轮反装，取反 */
    int16_t motor_R = RPM_TO_MOTOR(rpm_R);  

    chassis->rpm_L_target = (float)motor_L;  /* 记录目标值 (电机端, 含减速比) */
    chassis->rpm_R_target = (float)motor_R;

    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_L], motor_L, 0, 3);
    Stepper_Motor_Set_Speed(&chassis->Motor[WHEEL_R], motor_R, 0, 3);
}

/**
 * @brief  差速轮正运动学: 电机反馈转速 → vx/wz [m/s, rad/s]
 * @note   从编码器反馈的电机端RPM反算实际底盘线速度和角速度
 *
 *   vx = (vR + vL) / 2
 *   wz = (vR - vL) / d
 *   (d = CHASSIS_TRACK_WIDTH)
 *   右轮因反装，反馈转速取反
 */
static void Chassis_Actual_Calc(Chassis_Info_Typedef *chassis)
{
    float vL = -MOTOR_RPM_TO_MS( chassis->Motor[WHEEL_L].Data.speed);  /* 左轮线速度 [m/s] (反装取反) */
    float vR =  MOTOR_RPM_TO_MS( chassis->Motor[WHEEL_R].Data.speed);  /* 右轮线速度 [m/s] */

    chassis->vx_actual = (vR + vL) * 0.5f;
    chassis->wz_actual = (vR - vL) / CHASSIS_TRACK_WIDTH;
}
