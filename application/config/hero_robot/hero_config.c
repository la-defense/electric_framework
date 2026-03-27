#include "hero_config.h"

Robot_Config_s *HeroConfigInit(void) 
{
    static Robot_Config_s hero_config = { 
        .chassis_param = {
            .chassis_motor_id[MOTOR_LF] = 1,
            .chassis_motor_id[MOTOR_RF] = 4,
            .chassis_motor_id[MOTOR_RB] = 3,
            .chassis_motor_id[MOTOR_LB] = 2,

            .wheel_type = MECANUM_WHEEL,
            .wheel_measure = {
                .center_gimbal_offset_x = 0.0,
                .center_gimbal_offset_y = 0.0,
                .wheel_base = 370,
                .track_width = 420,
                .radius_wheel = 75,
                .reduction_ratio_wheel = 19.0,
            },

            .chassis_motor_config = {
                .can_init_config.can_handle = &hcan1,
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 5.0,
                        .Ki = 1.0,
                        .Kd = 0.0, 
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 12000,
                    },
                    .current_PID = {
                        .Kp = 0.5, 
                        .Ki = 0.0,   
                        .Kd = 0.0,
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 15000,
                    },
                },   
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                },
                .motor_type = M3508,             
            }
        },


        .gimbal_param = {
            .gimbal_type = SINGLE_GIMBAL,
            .yaw_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 1,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 10.0,
                        .Ki = 0.0, 
                        .Kd = 0.5, 
                        .DeadBand = 0.1,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 500,
                    },
                    .speed_PID = {
                        .Kp = 200.0, 
                        .Ki = 150.0, 
                        .Kd = 0.0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 3000,
                        .MaxOut = 20000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
                .motor_type = GM6020
            },        

            .pitch_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 0x7f,
                    .ext_id = 0x7f,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 10, // 10
                        .Ki = 1,
                        .Kd = 0.5,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 100,
                    },
                    .speed_PID = {
                        .Kp = 0.05,  // 50
                        .Ki = 0, // 350
                        .Kd = 0.003,   // 0
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 2500,
                        .MaxOut = 30,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
                .motor_type = XMCY
            }                
        },

        .shoot_param = {
            .friction_motor_id[MOTOR_LF] = 3,
            .friction_motor_id[MOTOR_RF] = 5,
            .friction_motor_id[MOTOR_LB] = 1,
            .friction_motor_id[MOTOR_RB] = 4,

            .shoot_type = FOUR_SHOOT,
            .friction_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                },
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 15.0, 
                        .Ki = 1.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 15000,
                    },
                    .current_PID = {
                        .Kp = 0.4, 
                        .Ki = 0.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 15000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
                .motor_type = M3508
            },

            .loader_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 6,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                        .Kp = 10.0, 
                        .Ki = 1.0,
                        .Kd = 0.0,
                        .MaxOut = 200, 
                    },
                    .speed_PID = {
                        .Kp = 10.0, 
                        .Ki = 1.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 8000,
                        .MaxOut = 10000,
                    },
                    .current_PID = {
                        .Kp = 0.4, 
                        .Ki = 0.0, 
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 5000,
                        .MaxOut = 5000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED, 
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
                    .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                },
                .motor_type = M3508 // 英雄使用M3508
            }
        }
    };

    return &hero_config;
} 
