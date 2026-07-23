#include "ps2_task.h"
#include "chassis_task.h"
#include "bsp_dwt.h"

#define PS2_TASK_PERIOD_MS  30

static void PS2_Ctrl_Chassis(PS2_Info_Typedef *PS2, Chassis_Info_Typedef *Chassis);
static void PS2_Mode_Switch(PS2_Info_Typedef *PS2, Chassis_Info_Typedef *Chassis);

void PS2_task(void)
{
    while (1)
    {
        uint64_t t0 = DWT_GetTimeline_us();

        PS2_ScanKey(&PS2_Info);
        PS2_Mode_Switch(&PS2_Info, &Chassis);
        PS2_Ctrl_Chassis(&PS2_Info, &Chassis);

        PS2_Info.loop_time_us = (float)(DWT_GetTimeline_us() - t0);

        osDelay(PS2_TASK_PERIOD_MS);
    }
}

/**
 * @brief  PS2 START键切换控制模式 (RC ↔ NAV)
 */
static void PS2_Mode_Switch(PS2_Info_Typedef *PS2, Chassis_Info_Typedef *Chassis)
{
    static uint8_t start_last = 0;

    if (PS2->PS2_buff[2] != 0x5A)
        return;

    if (PS2->Data.PS2_START && !start_last)  /* 上升沿 */
    {
        if (Chassis->mode == CHASSIS_MODE_RC)
            Chassis->mode = CHASSIS_MODE_NAV;
        else
            Chassis->mode = CHASSIS_MODE_RC;
    }
    start_last = PS2->Data.PS2_START;
}

/**
 * @brief  PS2 遥控器控制底盘 (仅 RC 模式生效)
 */
static void PS2_Ctrl_Chassis(PS2_Info_Typedef *PS2, Chassis_Info_Typedef *Chassis)
{
    if (Chassis->mode != CHASSIS_MODE_RC)
        return;

    if (PS2->PS2_buff[2] != 0x5A)
        return;

    /* 摇杆→速度 */
    int16_t jx = PS2_LRC_UD(PS2);
    int16_t jy = PS2_LRC_LR(PS2);
    int16_t jz = -PS2_RRC_LR(PS2);

    if (PS2->Data.mode == PS2_MODE_RED)
    {
        float vx = (float)jx / PS2_RC_MAX * CHASSIS_MAX_V;
        float vy = (float)jy / PS2_RC_MAX * CHASSIS_MAX_V;
        float wz = (float)jz / PS2_RC_MAX * CHASSIS_MAX_W;
        Chassis_Set_Velocity(Chassis, vx, vy, wz);

        /* 右摇杆上下: 累加控制丝杆高度 */
        int16_t r_ud = PS2_RRC_UD(PS2);
        if (r_ud > 20 || r_ud < -20)  /* 死区 ±20 */
        {
            float delta = (float)r_ud / PS2_RC_MAX * LIFT_MANUAL_MAX_SPEED
                          * (PS2_TASK_PERIOD_MS / 1000.0f);
            Chassis_Lift_Set_Height(Chassis, Chassis->lift_target_height + delta);
        }

        /* 三角键: 回零 (上升沿) */
        static uint8_t tri_last = 0;
        if (PS2->Data.PS2_TRI && !tri_last)
            Chassis_Lift_Home(Chassis);
        tri_last = PS2->Data.PS2_TRI;
    }
    else
    {
    }
}
