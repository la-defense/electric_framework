// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "referee_task.h"
#include "referee_UI.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "buzzer.h"
#include "led.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

/* 根据remote_control.h中的通道值自动计算的参数 */
#define HALF_RC_CH_MAX ((RC_CH_VALUE_MAX - RC_CH_VALUE_MIN) / 2.0f)
#define RAD_TO_DEG 57.295779513f
#define DEG_TO_RAD (1.0f / 57.295779513f)
#define YAW_GEAR_RATIO 1.25f  // yaw轴皮带传动比 1:1.25
#define PITCH_ZERO_OFFSET 0.0f  // pitch轴零点偏移，调整此值使上位机发送0时云台水平

/* cmd应用包含的模块实例指针和交互信息存储*/
static Robot_Config_s *robot_config;

#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static referee_info_t* referee_data; // 用于获取裁判系统的数据 
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static Robot_Status_e robot_state; // 机器人整体工作状态

BuzzzerInstance *buzzer_alarm;
LEDInstance *led_alarm;

void RobotCMDInit()
{
    robot_config = RobotConfigInit(SENTINEL_ROBOT);

    Buzzer_config_s buzzer_config ={
            .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
            .loudness=  0.4, //设置响度
            .octave=  OCTAVE_1, // 设置音阶
        };
    buzzer_alarm = BuzzerRegister(&buzzer_config);    

    LED_config_s LED_config ={
            .color = PINK,
            .flash = ALWAYS_ON,
            .flash_count = 0,
        };
    led_alarm = LEDRegister(&LED_config);

    rc_data = RemoteControlInit(&huart3);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    referee_data = UITaskInit(&huart6,&ui_data); // 裁判系统初始化,会同时初始化UI 

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    float yaw_align_angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    yaw_align_angle = robot_config->gimbal_param.gimbal_offset.yaw_offset * ECD_ANGLE_COEF_DJI;
    if (robot_config->gimbal_param.gimbal_offset.yaw_offset > 4096)                               // 如果大于180度
    {
        if (angle > yaw_align_angle && angle <= 180.0f + yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else if (angle > yaw_align_angle - 180.0f && angle <= yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else
            chassis_cmd_send.offset_angle = 360.0f - yaw_align_angle + angle;
    }
    else
    {
        if (angle > yaw_align_angle && angle <= 180.0f + yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else if (angle > 0.0f  && angle <= yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else
            chassis_cmd_send.offset_angle = angle - 360.0f - yaw_align_angle;
    }
}


/**
 * @brief 根据gimbal fetch data进行yaw和pitch限幅
 *        yaw轴范围为[-90,90],pitch轴范围为[PITCH_MIN_ANGLE,PITCH_MAX_ANGLE]
 *
 */
static void LimitSetAngle(float add_yaw, float add_pitch)
{
    static float bias_angle;    // yaw轴角度误差值
    bias_angle = gimbal_cmd_send.yaw + gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;     // yaw轴角度正负相反
    // 云台软件限位,向左/向上偏移为正
    if (chassis_cmd_send.offset_angle + fabs(bias_angle) >= 90.0f || 
        gimbal_cmd_send.pitch >= robot_config->gimbal_param.gimbal_offset.pitch_max_angle)
    { // 云台yaw轴/pitch轴正向偏移
        if (add_yaw < 0.0f)
            gimbal_cmd_send.yaw -= add_yaw;

        if (add_pitch > 0.0f)
            gimbal_cmd_send.pitch -= add_pitch;
    }
    if (chassis_cmd_send.offset_angle - fabs(bias_angle) <= -90.0f ||
        gimbal_cmd_send.pitch <= robot_config->gimbal_param.gimbal_offset.pitch_min_angle)
    { // 云台yaw轴/pitch轴负向偏移            
        if (add_yaw > 0.0f)
            gimbal_cmd_send.yaw -= add_yaw;

        if (add_pitch < 0.0f)
            gimbal_cmd_send.pitch -= add_pitch;    
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */ 
static void RemoteControlSet()
{
    static float add_yaw, add_pitch; // 角度增量
    static shoot_mode_e last_shoot_mode = SHOOT_ZERO_FORCE; 

#ifdef ONE_BOARD
    // 单独控制云台和底盘

     if (gimbal_cmd_send.last_mode == GIMBAL_ZERO_FORCE && gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) 
    {
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch; // 目标 = 当前实际位置
    }

    gimbal_cmd_send.last_mode = gimbal_cmd_send.gimbal_mode;
    
    if (switch_is_down(rc_data[TEMP].rc.switch_left))      
    {   // 左侧开关状态[下],右侧开关状态不为[下],底盘无力,云台陀螺仪模式
        if (!switch_is_down(rc_data[TEMP].rc.switch_right))
        {
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        }
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_right))      
    {   // 左侧开关状态不为[下],右侧开关状态为[下],底盘编码器控制,云台无力
        if (!switch_is_down(rc_data[TEMP].rc.switch_left))
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;   
        }
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }
    // 组合控制底盘和云台
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))       
    {  
        // 左侧开关状态[中],右侧开关状态为[中],底盘跟随云台
        if (switch_is_mid(rc_data[TEMP].rc.switch_left))
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

        // 左侧开关状态[上],右侧开关状态为[中],底盘小陀螺
        if (switch_is_up(rc_data[TEMP].rc.switch_left))
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
#endif
#ifdef GIMBAL_BOARD
    if (switch_is_down(rc_data[TEMP].rc.switch_right))      
    {   // 左侧开关状态不为[下],右侧开关状态为[下],底盘编码器控制,云台无力
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    }

    if (switch_is_mid(rc_data[TEMP].rc.switch_right))      
    {   // 左侧开关状态不为[下],右侧开关状态为[下],底盘编码器控制,云台无力
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
#endif

    // 云台陀螺仪模式,并且视觉未识别到目标,纯遥控器拨杆控制
    add_yaw = RC_TO_YAW_ANGLE * (float)rc_data[TEMP].rc.rocker_l_;
    add_pitch = RC_TO_PITCH_ANGLE * (float)rc_data[TEMP].rc.rocker_l1;

    if (robot_state == ROBOT_READY)
    {
        // 视觉接管优先级最高
        if (vision_recv_data->mode == 1 || vision_recv_data->mode == 2)
        {
            float vision_yaw = vision_recv_data->yaw * RAD_TO_DEG;
            float vision_pitch = vision_recv_data->pitch * RAD_TO_DEG + PITCH_ZERO_OFFSET;
            if (vision_recv_data->mode == 2) {
                shoot_cmd_send.shoot_mode = SHOOT_ON;
                shoot_cmd_send.friction_mode = FRICTION_ON;
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                shoot_cmd_send.shoot_rate = 10;
            } else { // mode == 1: 识别到目标但不射击
                shoot_cmd_send.load_mode = LOAD_STOP;
                shoot_cmd_send.friction_mode = FRICTION_ON;
            }
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            gimbal_cmd_send.yaw = vision_yaw;
            gimbal_cmd_send.pitch = vision_pitch;
            add_yaw = 0.0f;
            add_pitch = 0.0f;
        }
        else // mode == 0: 未识别到目标，切换回手动控制
        {
            // 恢复手动控制，使用遥控器摇杆控制云台
            // add_yaw 和 add_pitch 已经在前面计算好了
        }
        if (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE)
        { // 按照摇杆的输出大小进行角度增量,拨杆向右/向上为正
            if (vision_recv_data->mode == 0) { // 只有在视觉未识别到目标时才允许遥控器调整云台角度
                gimbal_cmd_send.yaw += add_yaw;
                gimbal_cmd_send.pitch += add_pitch;
            }
            
        }
    }

    // 云台角度限幅,要放在增量以后
 //   LimitSetAngle(add_yaw, add_pitch);

 VAL_LIMIT(gimbal_cmd_send.pitch, 
          robot_config->gimbal_param.gimbal_offset.pitch_min_angle, 
          robot_config->gimbal_param.gimbal_offset.pitch_max_angle);

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数为2.0m/s(经测量,空转转速最大3.0m/s,实际速度和地面接触受力有关)
    chassis_cmd_send.vx = 3.0f / HALF_RC_CH_MAX * (float)rc_data[TEMP].rc.rocker_r_;    // 横移方向
    chassis_cmd_send.vy = 3.5f / HALF_RC_CH_MAX * (float)rc_data[TEMP].rc.rocker_r1;    // 前进方向
    chassis_cmd_send.wz = 2.5f / HALF_RC_CH_MAX * (float)rc_data[TEMP].rc.rocker_l_;    // 旋转方向

    // chassis_cmd_send.vx = vision_recv_data->data.speed_vector.vx;
    // chassis_cmd_send.vy = vision_recv_data->data.speed_vector.vy;
    // chassis_cmd_send.wz = 0;
    // chassis_cmd_send.wz = -vision_recv_data->data.speed_vector.wz;

    // // 发射参数
    // if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 右侧开关状态[上],弹舱打开
    //     ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    // else
    //     ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 右侧开关[中],发射开启
    if (switch_is_mid(rc_data[TEMP].rc.switch_right)) 
    {   // 快拨右侧开关[中]->右侧开关[上]->右侧开关[中],摩擦轮开启
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        if (last_shoot_mode == SHOOT_OFF && shoot_cmd_send.friction_mode == FRICTION_ON)
            shoot_cmd_send.friction_mode = FRICTION_OFF;
        else if (last_shoot_mode == SHOOT_OFF && shoot_cmd_send.friction_mode != FRICTION_ON)
            shoot_cmd_send.friction_mode = FRICTION_ON;
         
        if (rc_data[TEMP].rc.dial < 10)
            shoot_cmd_send.load_mode = LOAD_STOP;     
        
            
        if (rc_data[TEMP].rc.dial > 600)
        {
            // 射频控制
            shoot_cmd_send.shoot_rate = 10; 
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        }     
    }
    
    // 右侧开关[上],发射关闭
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) 
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    // 左右侧开关均为[下],摩擦轮关闭
    if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_OFF;      
    }
    // 保留上一次发射模式 
    last_shoot_mode = shoot_cmd_send.shoot_mode;


}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    static float add_yaw, add_pitch; // 角度增量
    add_yaw = (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    add_pitch = -(float)rc_data[TEMP].mouse.y / 660 * 10;

    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].a * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2) // G键设置模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;
    case 1:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
        break;
    }

    shoot_cmd_send.bullet_speed = 15;
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    case 1:
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;    
    default:
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    }

    if (shoot_cmd_send.friction_mode == FRICTION_ON)    // 开启摩擦轮后左键发射
    {
        switch (rc_data[TEMP].mouse.press_l) 
        {
        case 0:
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        case 1:
            // 射频控制
            shoot_cmd_send.shoot_rate = 10; 
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            break;
        }
    }

    switch (rc_data[TEMP].mouse.press_r)
    {
    case 0:
        break;
    case 1:
        if (vision_recv_data->mode == 1 || vision_recv_data->mode == 2)
        {
            gimbal_cmd_send.yaw = vision_recv_data->yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch;

            add_yaw = 0.0f;
            add_pitch = 0.0f;

            if (vision_recv_data->mode == 2 && shoot_cmd_send.friction_mode == FRICTION_ON)
            {
                shoot_cmd_send.shoot_rate = 10;
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            }
        }
        break;


        // if (vision_recv_data->target_state == TARGET_CONVERGING)
        // {
        //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        // }
        // else
        // {
        //     shoot_cmd_send.load_mode = LOAD_STOP;
        // }
        break;
    default:
        break;
    }

    // gimbal_cmd_send.yaw += add_yaw;
    // gimbal_cmd_send.pitch += add_pitch;

    if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
        // 云台角度限幅,要放在增量以后
    //    LimitSetAngle(add_yaw, add_pitch);
    
        VAL_LIMIT(gimbal_cmd_send.pitch, 
        robot_config->gimbal_param.gimbal_offset.pitch_min_angle, 
        robot_config->gimbal_param.gimbal_offset.pitch_max_angle);
    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    // {
    // case 0:
    //     shoot_cmd_send.lid_mode = LID_OPEN;
    //     break;
    // default:
    //     shoot_cmd_send.lid_mode = LID_CLOSE;
    //     break;
    // }

    // switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    // {
    // case 0:
    //     chassis_cmd_send.chassis_speed_buff = 40;
    //     break;
    // case 1:
    //     chassis_cmd_send.chassis_speed_buff = 60;
    //     break;
    // case 2:
    //     chassis_cmd_send.chassis_speed_buff = 80;
    //     break;
    // default:
    //     chassis_cmd_send.chassis_speed_buff = 100;
    //     break;
    // }

    // switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    // {
    // case 1:
    //     break;
    // default:
    //     break;
    // }

}

/**
 * @brief  报警处理，对于不同重要模块离线或者遥控器离线，进行不同类型报警
 *
 */
static void AlarmHandler()
{
    uint8_t temp = 0;

    if (robot_state == ROBOT_READY)
    {
        // 无力模式下，绿灯慢闪并且不报警
        if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE && gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE)
        {
            LEDSetStatus(led_alarm, ALARM_ON); 
            LEDSetFlash(led_alarm, GREEN, FLASH_LOW); 
            BuzzerSetStatus(buzzer_alarm, ALARM_OFF);
        }
        else
        { // 有力模式下，粉灯呼吸变化并且不报警
            LEDSetStatus(led_alarm, ALARM_OFF); 
            BuzzerSetStatus(buzzer_alarm, ALARM_OFF);
        }
    }
    if (robot_state == ROBOT_STOP) 
    {
        // 底盘模块离线
        if (chassis_fetch_data.motor_state == GIMBAL_MOTOR_OFFLINE)
        { // 黄灯快闪，闪烁次数对应不同电机，蜂鸣器报警声为Do
            temp++;
            LEDSetStatus(led_alarm, ALARM_ON); 
            LEDSetFlashTime(led_alarm, chassis_fetch_data.motor_offline_count);
            LEDSetFlash(led_alarm, YELLOW, FLASH_HIGH);
            BuzzerSetStatus(buzzer_alarm, ALARM_ON);
            BuzzerSetOctave(buzzer_alarm, OCTAVE_1);
        }

        // 云台模块离线
        if (gimbal_fetch_data.motor_state == CHASSIS_MOTOR_OFFLINE)
        { // 红灯快闪，闪烁次数对应不同电机，蜂鸣器报警声为Re
            temp++;
            LEDSetStatus(led_alarm, ALARM_ON); 
            LEDSetFlashTime(led_alarm, gimbal_fetch_data.motor_offline_count);
            LEDSetFlash(led_alarm, RED, FLASH_HIGH);
            BuzzerSetStatus(buzzer_alarm, ALARM_ON);
            BuzzerSetOctave(buzzer_alarm, OCTAVE_2);
        }

        // 发射模块离线

        // 遥控器离线或者急停模式
        if (temp == 0)
        { // 黄灯常亮并且不报警
            LEDSetStatus(led_alarm, ALARM_ON); 
            LEDSetFlash(led_alarm, YELLOW, ALWAYS_ON);
            BuzzerSetStatus(buzzer_alarm, ALARM_OFF);
        }

        if (temp > 1 || chassis_fetch_data.motor_offline_count > 1 || gimbal_fetch_data.motor_offline_count > 1)
        { // 超过一个模块或多个电机离线，红灯常量并且不报警
            LEDSetStatus(led_alarm, ALARM_ON); 
            LEDSetFlash(led_alarm, RED, ALWAYS_ON);
            BuzzerSetStatus(buzzer_alarm, ALARM_OFF);      
        }
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向上拨超过一半或者遥控器离线进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial < -400 || !RemoteControlIsOnline())
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // if (gimbal_fetch_data.motor_state == GIMBAL_MOTOR_OFFLINE || chassis_fetch_data.motor_state == CHASSIS_MOTOR_OFFLINE)  // 还需加入发射机构离线判断
    // {
    //     robot_state = ROBOT_STOP;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    //     chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    //     shoot_cmd_send.shoot_mode = SHOOT_OFF;
    //     shoot_cmd_send.friction_mode = FRICTION_OFF;
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    //     LOGERROR("[CMD] motor offline!");         
    // }
    // 遥控器左侧和右侧开关均为[下]，遥控器和模块均在线,恢复正常运行
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right) && RemoteControlIsOnline())
    {
        // if (gimbal_fetch_data.motor_state == GIMBAL_MOTOR_ONLINE && chassis_fetch_data.motor_state == CHASSIS_MOTOR_ONLINE)
        // {
            robot_state = ROBOT_READY;       
            LOGINFO("[CMD] reinstate, robot ready");            
        // }
    }

    AlarmHandler(); // 报警处理
}

/**
 * @brief  裁判系统反馈，包括UI数据/小电脑数据/裁判系统限制
 *
 */
static void RefereeHandler()
{
    // 裁判系统功率控制
    if (!referee_data->GameRobotState.chassis_power_limit)
        chassis_cmd_send.power_limit = 45;
    else
        chassis_cmd_send.power_limit = referee_data->GameRobotState.chassis_power_limit;

    // 裁判系统热量控制
    if (referee_data->PowerHeatData.shooter_17mm_1_barrel_heat > 300)
        shoot_cmd_send.load_mode = LOAD_STOP;

    // 视觉数据反馈
    float q[4];
    EularAngleToQuaternion(gimbal_fetch_data.gimbal_imu_data.Yaw,
                           gimbal_fetch_data.gimbal_imu_data.Pitch,
                           gimbal_fetch_data.gimbal_imu_data.Roll,
                           q);

    VisionUpdateTx(
        (gimbal_cmd_send.gimbal_mode == GIMBAL_GYRO_MODE) ? 1 : 0,
        q[0], q[1], q[2], q[3],
        gimbal_fetch_data.gimbal_imu_data.YawTotalAngle / YAW_GEAR_RATIO,
        gimbal_fetch_data.gimbal_imu_data.Gyro[2],
        gimbal_fetch_data.gimbal_imu_data.Pitch,
        gimbal_fetch_data.gimbal_imu_data.Gyro[0],
        referee_data->ShootData.bullet_speed,
        referee_data->GameRobotState.shooter_barrel_cooling_value
    );
    VisionSend();
    // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // vision_send_data.bullet_speed = referee_data->GameRobotState.shooter_barrel_cooling_value;
    // vision_send_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // // UI数据反馈
    // ui_data.chassis_mode = chassis_cmd_send.chassis_mode;
    // ui_data.gimbal_mode = gimbal_cmd_send.gimbal_mode;
    // ui_data.shoot_mode = shoot_cmd_send.shoot_mode;
    // ui_data.friction_mode = shoot_cmd_send.friction_mode;
    // ui_data.lid_mode = shoot_cmd_send.lid_mode;

    // ui_data.chassis_offset_angle = chassis_cmd_send.offset_angle;
    // ui_data.pitch_angle = gimbal_fetch_data.gimbal_imu_data.Pitch;
    // ui_data.bullet_speed = shoot_cmd_send.bullet_speed;    
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    // B键开关遥控器
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 2) 
    {
    case 0:
        RemoteControlSet();
        rc_data[TEMP].key_count[KEY_PRESS][Key_G] = 0;  // 初始化键盘控制
        rc_data[TEMP].key_count[KEY_PRESS][Key_F] = 0;  
        ui_data.Referee_Interactive_Flag.refresh_flag = 0;
        break;    
    case 1:
        MouseKeySet();
        if (!ui_data.Referee_Interactive_Flag.refresh_flag)
            ui_data.Referee_Interactive_Flag.refresh_flag = 1; // UI界面刷新

        if (rc_data[TEMP].rc.dial < -400)
            rc_data[TEMP].key_count[KEY_PRESS][Key_B] = 0;     // 上拨切换回遥控器控制
        break;            
    default:
        RemoteControlSet();
        break;
    }

    // EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    RefereeHandler();       // 裁判系统反馈

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
