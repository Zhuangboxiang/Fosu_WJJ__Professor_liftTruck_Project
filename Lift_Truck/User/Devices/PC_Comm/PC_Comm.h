/**
 ******************************************************************************
 * @file    PC_Comm.h
 * @version V1.0.0
 * @date    2026.04.11
 * @brief   上位机通信驱动实现声明
 * @encoding UTF-8
 ******************************************************************************
 * @attention
 * * 无
 ******************************************************************************
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __PC_COMM_H
#define __PC_COMM_H

/* Includes ----------------------------------------------------------------- */
#include "cmsis_os.h"
#include "stdbool.h"

/* Defines ------------------------------------------------------------------ */
#define PC_TX_HEAD						0xAA		/* 帧头(小车→上位机) */
#define PC_TX_FRAME_LEN					34u			/* 发送帧: HEAD(1)+Vx/Vy/Vw(12)+Bat(4)+H(4)+Yaw/Pitch/Roll(12)+XOR(1) */

#define PC_RX_HEAD						0x5A		/* 包头(上位机→小车) */
#define PC_RX_TAIL						0xA5		/* 包尾(上位机→小车) */
#define PC_RX_FRAME_LEN					19u			/* 接收帧: HEAD(1)+Vx(4)+Vy(4)+Vw(4)+H(4)+校验(1)+TAIL(1) */

/* Enums -------------------------------------------------------------------- */

/* Structs ------------------------------------------------------------------ */
/**
 * @brief 发送帧结构体(小车→上位机)
 * @note  packed 防对齐, 帧头 0xAA, 固定34字节
 */
typedef struct __attribute__((packed)) {
    uint8_t HEAD;				/* 帧头: 0xAA */
    float Vx;		            /* X方向速度 [m/s] */
    float Vy;		            /* Y方向速度 [m/s] */
    float Vw;		            /* Z方向角速度 [rad/s] */
    float Bat_V;		        /* 电池电压 [V] */
    float Lift_H;	            /* 丝杆当前高度 [mm] */
    float Yaw;		            /* 偏航角 [rad] */
    float Pitch;		        /* 俯仰角 [rad] */
    float Roll;		            /* 横滚角 [rad] */
    uint8_t Checksum;			/* 校验和 (前33字节异或) */
} PC_TxFrame_Typedef;

/**
 * @brief 上位机通信速度结构体(收发统一)
 */
typedef struct {
    float tx_vx;		/* 发往上位机: X方向速度 [m/s] */
    float tx_vy;		/* 发往上位机: Y方向速度 [m/s] */
    float tx_vw;		/* 发往上位机: Z方向角速度 [rad/s] */
    float rx_vx;		/* 上位机接收: X方向速度 [m/s] */
    float rx_vy;		/* 上位机接收: Y方向速度 [m/s] */
    float rx_vw;		/* 上位机接收: Z方向角速度 [rad/s] */
} PC_Speed_Typedef;

/* Externs ------------------------------------------------------------------ */
extern PC_TxFrame_Typedef PC_TxFrame;
extern uint8_t PC_TxBuf[PC_TX_FRAME_LEN];

/* Functions ---------------------------------------------------------------- */
void PC_Info_Update(uint8_t *buff, uint16_t len);
void PC_Info_Upload(float vx, float vy, float vw);
void PC_Set_Chassis_Mode(uint8_t mode);

/* -------------------------------------------------------------------------- */
#endif /* __PC_COMM_H */
