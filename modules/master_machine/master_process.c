#include "master_process.h"

#include "daemon.h"
#include "bsp_log.h"
#include "bsp_usb.h"

#include <string.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static uint8_t *vis_recv_buff;

/* 和 sp_vision_25 一致的 CRC16/X25 */
static uint16_t VisionCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }

    return crc;
}

static uint8_t VisionCheckCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t rx_crc = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    uint16_t calc_crc = VisionCRC16(data, len - 2);
    return (rx_crc == calc_crc);
}

static void VisionOfflineCallback(void *id)
{
    (void)id;
    LOGWARNING("[vision] vision offline.");
}

void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count)
{
    send_data.head[0] = 'S';
    send_data.head[1] = 'P';
    send_data.mode = mode;

    send_data.q[0] = q0;
    send_data.q[1] = q1;
    send_data.q[2] = q2;
    send_data.q[3] = q3;

    send_data.yaw = yaw;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch = pitch;
    send_data.pitch_vel = pitch_vel;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

static void DecodeVision(uint16_t recv_len)
{
    if (recv_len < sizeof(Vision_Recv_s))
        return;

    for (uint16_t i = 0; i + sizeof(Vision_Recv_s) <= recv_len; i++)
    {
        Vision_Recv_s pkt;
        memcpy(&pkt, vis_recv_buff + i, sizeof(Vision_Recv_s));

        if (pkt.head[0] != 'S' || pkt.head[1] != 'P')
            continue;

        if (!VisionCheckCRC16((const uint8_t *)&pkt, sizeof(Vision_Recv_s)))
            continue;

        recv_data = pkt;
        DaemonReload(vision_daemon_instance);
        return;
    }
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    (void)_handle;

    memset(&recv_data, 0, sizeof(recv_data));
    memset(&send_data, 0, sizeof(send_data));

    USB_Init_Config_s conf = {
        .rx_cbk = DecodeVision
    };

    vis_recv_buff = USBInit(conf);

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5,
    };

    vision_daemon_instance = DaemonRegister(&daemon_conf);
    return &recv_data;
}

void VisionSend(void)
{
    Vision_Send_s tx_pkt = send_data;

    tx_pkt.head[0] = 'S';
    tx_pkt.head[1] = 'P';
    tx_pkt.crc16 = VisionCRC16((const uint8_t *)&tx_pkt, sizeof(Vision_Send_s) - 2);

    USBTransmit((uint8_t *)&tx_pkt, sizeof(Vision_Send_s));
}