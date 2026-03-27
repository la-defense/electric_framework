#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include <stdint.h>

#define VISION_RECV_SIZE 64u
#define VISION_SEND_SIZE 64u

#pragma pack(push, 1)

/* 上位机 -> 下位机，和 sp_vision_25 的 VisionToGimbal 对齐 */
typedef struct
{
    uint8_t head[2];   // 'S', 'P'
    uint8_t mode;      // 0: 不控制, 1: 控制云台不开火, 2: 控制云台并开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} Vision_Recv_s;

/* 下位机 -> 上位机，和 sp_vision_25 的 GimbalToVision 对齐 */
typedef struct
{
    uint8_t head[2];   // 'S', 'P'
    uint8_t mode;      // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];        // w, x, y, z
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} Vision_Send_s;

#pragma pack(pop)

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);
void VisionSend(void);

/* 每个控制周期把下位机状态塞给上位机 */
void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count);

#endif