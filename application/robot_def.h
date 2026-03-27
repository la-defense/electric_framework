/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

#include "config.h"

// /* 开发板类型定义,无人机和飞镖使用云台板,其他兵种使用单板(待加入平衡) */
// #if (!defined(DART_ROBOT) && !defined(AERIAL_ROBOT))
//     #define ONE_BOARD 
// #endif
// // #define CHASSIS_BOARD //底盘板
// #if (defined(DART_ROBOT) || defined(AERIAL_ROBOT))
//     #define GIMBAL_BOARD  
// #endif

// // 检查是否存在该机器人ID,不存在则编译会自动报错
// #if (!defined(INFANTRY_ROBOT) && !defined(SENTINEL_ROBOT)) && \
//     (!defined(HERO_ROBOT) && !defined(ENGINEER_ROBOT)) && \
//     (!defined(AERIAL_ROBOT) && !defined(DART_ROBOT))
// #error None Robot ID definition! You can only define ID from 0 to 5.
// #endif

// // 检查是否出现底盘类型冲突,只允许一个底盘类型定义存在,否则编译会自动报错
// #if (defined(MECANUM_WHEEL) && defined(OMIN_WHEEL)) || \
//     (defined(MECANUM_WHEEL) && defined(STEER_WHEEL)) ||  \
//     (defined(OMIN_WHEEL) && defined(STEER_WHEEL))
// #error Conflict chassis wheel definition! You can only define one chassis type.
// #endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;
// 底盘电机状态
typedef enum
{
    CHASSIS_MOTOR_OFFLINE = 0,    
    CHASSIS_MOTOR_ONLINE,
} chassis_motor_state_e;
// 底盘电机离线类型
typedef enum
{
    NONE_CHASSIS_TYPE = 0,    
    LF_MOTOR,
    RF_MOTOR,
    LB_MOTOR,
    RB_MOTOR,
} chassis_offline_type_e;

//云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;
// 云台电机状态
typedef enum
{
    GIMBAL_MOTOR_OFFLINE = 0,
    GIMBAL_MOTOR_ONLINE,
} gimbal_motor_state_e;

// 发射模式设置
typedef enum
{
    SHOOT_ZERO_FORCE = 0,
    SHOOT_OFF,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 弹速限制枚举,与裁判系统弹速等级对应
typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;


/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    float power_limit;  // 功率限制
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float mini_yaw;
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
    gimbal_mode_e last_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后底盘的真实速度
    float real_vx;
    float real_vy;
    float real_wz;
    // 底盘电机离线数量
    uint8_t motor_offline_count;       
    // 底盘电机状态
    chassis_motor_state_e motor_state;
   
} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;

    uint8_t motor_offline_count;                // 云台电机离线数量
    gimbal_motor_state_e motor_state;           // 云台电机状态
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
  
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

Robot_Config_s *RobotConfigInit(uint8_t id);
Chassis_Config_s *ChassisConfigFeed(void);
Gimbal_Config_s *GimbalConfigFeed(void);
Shoot_Config_s *ShootConfigFeed(void);
 
#endif // !ROBOT_DEF_H