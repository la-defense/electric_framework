#include "gimbal.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "xmmotor.h"

#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static Gimbal_Config_s *gimbal_config;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *mini_yaw_motor, *pitch_motor;
static XMMotorInstance *pitch_xmmotor;
//static gimbal_mode_e last_mode;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源

    gimbal_config = GimbalConfigFeed();
  
    // yaw轴初始化
    gimbal_config->yaw_motor_config.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle;
    gimbal_config->yaw_motor_config.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2];
    yaw_motor = DJIMotorInit(&gimbal_config->yaw_motor_config);
    
    // 小yaw轴初始化
    if (gimbal_config->gimbal_type == MINI_GIMBAL)
   //     mini_yaw_motor = DJIMotorInit(&gimbal_config->mini_yaw_motor_config);
    
#ifdef _IS_IMU_ROLL
    // PITCH
    gimbal_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->Roll;
    gimbal_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[1];
#else
    // PITCH
    gimbal_config->pitch_motor_config.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->Pitch;
    gimbal_config->pitch_motor_config.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0];
#endif

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    switch (gimbal_config->pitch_motor_config.motor_type)
    {
    case GM6020:
        pitch_motor = DJIMotorInit(&gimbal_config->pitch_motor_config);
        break;
    case XMCY:
        pitch_xmmotor = XMMotorInit(&gimbal_config->pitch_motor_config);
        break;
    default:
        break;
    }

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

      // 只要是从无力模式切回控制模式，强制同步目标值
    // if (gimbal_cmd_recv.last_mode == GIMBAL_ZERO_FORCE && gimbal_cmd_recv.gimbal_mode == GIMBAL_GYRO_MODE) 
    // {
    //     gimbal_cmd_recv.pitch = gimba_IMU_data->Pitch; // 目标 = 当前实际位置
    // }

    // gimbal_cmd_recv.last_mode = gimbal_cmd_recv.gimbal_mode;
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);

        if (gimbal_config->gimbal_type == MINI_GIMBAL)
            DJIMotorStop(mini_yaw_motor);

        switch (gimbal_config->pitch_motor_config.motor_type)
        {
        case GM6020:
            DJIMotorStop(pitch_motor);
            break;
        case XMCY:
            XMMotorStop(pitch_xmmotor);
            break;
        default:
            break;
        }
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: 
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈

        if (gimbal_config->gimbal_type == MINI_GIMBAL)
        {
            DJIMotorEnable(mini_yaw_motor);
            DJIMotorSetRef(mini_yaw_motor, gimbal_cmd_recv.mini_yaw); 
        }
    
        switch (gimbal_config->pitch_motor_config.motor_type)
        {
        case GM6020:
            DJIMotorEnable(pitch_motor);
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
            DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            break;
        case XMCY:
            XMMotorEnable(pitch_xmmotor);
            XMMotorChangeFeed(pitch_xmmotor, ANGLE_LOOP, OTHER_FEED);
            XMMotorChangeFeed(pitch_xmmotor, SPEED_LOOP, OTHER_FEED);
            XMMotorSetRef(pitch_xmmotor, gimbal_cmd_recv.pitch);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd,还有云台电机状态
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 根据电机状态反馈是否离线
    // DJIMotorIsOnline(yaw_motor);
    // DJIMotorIsOnline(pitch_motor);
    // // 反馈电机离线数量
    // if (yaw_motor->online_flag == MOTOR_OFFLINE || pitch_motor->online_flag == MOTOR_OFFLINE)
    // {
    //     gimbal_feedback_data.motor_offline_count = 1;
    //     gimbal_feedback_data.motor_state = GIMBAL_MOTOR_OFFLINE;
    // }
    // else if(yaw_motor->online_flag == MOTOR_OFFLINE && pitch_motor->online_flag == MOTOR_OFFLINE)
    // {
    //     gimbal_feedback_data.motor_offline_count = 2;
    //     gimbal_feedback_data.motor_state = GIMBAL_MOTOR_OFFLINE;
    // }
    // else 
    // {
    //     gimbal_feedback_data.motor_offline_count = 0;
    //     gimbal_feedback_data.motor_state = GIMBAL_MOTOR_ONLINE;
    // }

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}