#include "infantry_config.h"

Robot_Config_s *InfantryConfigInit(uint8_t robot_id) 
{
    switch (robot_id)
    {
    /*******************************************************三号步兵**********************************************************************/
    case INFANTRY_ROBOT_3:
        static Robot_Config_s infantry_config_3 = { 
            .chassis_param = {
                .chassis_motor_id[MOTOR_LF] = 1,
                .chassis_motor_id[MOTOR_RF] = 2,
                .chassis_motor_id[MOTOR_RB] = 3,
                .chassis_motor_id[MOTOR_LB] = 4,

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
                .gimbal_offset = {
                    .yaw_offset = 2640,
                    .pitch_min_angle = -10.0f,//-18
                    .pitch_max_angle = 20.0f,//35
                },

                .yaw_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 1,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 8.0,
                            .Ki = 0.0, 
                            .Kd = 0.4, 
                            .DeadBand = 0.1,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 300.0, 
                            .Ki = 100.0, 
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
                        .can_handle = &hcan1,
                        .tx_id = 2,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 15.0,//15
                            .Ki = 0.0, 
                            .Kd = 0.4, //0.4
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 250.0, 
                            .Ki = 100.0, 
                            .Kd = 0.0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 2500,
                            .MaxOut = 20000,
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
                    .motor_type = GM6020
                }                
            },

            .shoot_param = {
                .friction_motor_id[MOTOR_LF] = 1,
                .friction_motor_id[MOTOR_RF] = 2,
                .friction_motor_reverse_flag[MOTOR_LF] = MOTOR_DIRECTION_NORMAL,
                .friction_motor_reverse_flag[MOTOR_RF] = MOTOR_DIRECTION_REVERSE,    

                .shoot_type = DUAL_SHOOT,
                .friction_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan2,
                    },
                    .controller_param_init_config = {
                        .speed_PID = {
                            .Kp = 10.0, 
                            .Ki = 1.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 10000,
                            .MaxOut = 25000,
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
                        .can_handle = &hcan2,
                        .tx_id = 5,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                            .Kp = 15.0, 
                            .Ki = 1.0,
                            .Kd = 0.0,
                            .MaxOut = 5000, 
                        },
                        .speed_PID = {
                            .Kp = 3.0, 
                            .Ki = 0.0, 
                            .Kd = 0.0,
                            .Improve = PID_Integral_Limit,
                            .IntegralLimit = 8000,
                            .MaxOut = 8000,
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
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
                    .motor_type = M2006 // 英雄使用M3508
                }
            }
        };
        return &infantry_config_3;
        break;
    /*******************************************************四号步兵**********************************************************************/
    case INFANTRY_ROBOT_4:
        static Robot_Config_s infantry_config_5 = { 
            .chassis_param = {
                .chassis_motor_id[MOTOR_LF] = 3,  
                .chassis_motor_id[MOTOR_RF] = 2,  
                .chassis_motor_id[MOTOR_RB] = 1, 
                .chassis_motor_id[MOTOR_LB] = 4, 

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
                .gimbal_offset = {
                    .yaw_offset = 5932,
                    .pitch_min_angle = -40.0f,
                    .pitch_max_angle = 15.0f,
                },

                .yaw_motor_config = {
                    .can_init_config = {
                        .can_handle = &hcan1,
                        .tx_id = 1,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 15.0,
                            .Ki = 0.0, 
                            .Kd = 0.7, 
                            .DeadBand = 0.1,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 300.0, 
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
                        .tx_id = 2,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            .Kp = 20.0,
                            .Ki = 0.0, 
                            .Kd = 0.4, 
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 100,
                            .MaxOut = 500,
                        },
                        .speed_PID = {
                            .Kp = 200.0, 
                            .Ki = 100.0, 
                            .Kd = 0.0,
                            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                            .IntegralLimit = 2500,
                            .MaxOut = 20000,
                        },
                    },
                    .controller_setting_init_config = {
                        .angle_feedback_source = OTHER_FEED,
                        .speed_feedback_source = OTHER_FEED,
                        .outer_loop_type = ANGLE_LOOP,
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
                    },
                    .motor_type = GM6020
                }                
            },

            .shoot_param = {
                .friction_motor_id[MOTOR_LF] = 3,
                .friction_motor_id[MOTOR_RF] = 4,

                .friction_motor_reverse_flag[MOTOR_LF] = MOTOR_DIRECTION_REVERSE,
                .friction_motor_reverse_flag[MOTOR_RF] = MOTOR_DIRECTION_NORMAL,    
                .shoot_type = DUAL_SHOOT,
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
                        .can_handle = &hcan2,
                        .tx_id = 7,
                    },
                    .controller_param_init_config = {
                        .angle_PID = {
                            // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                            .Kp = 10.0, 
                            .Ki = 1.0,
                            .Kd = 0.0,
                            .MaxOut = 5000, 
                        },
                        .speed_PID = {
                            .Kp = 3.0, 
                            .Ki = 0.0, 
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
                        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    },
                    .motor_type = M2006 // 英雄使用M3508
                }
            }
        };
        return &infantry_config_5;
        break;
    /*******************************************************五号步兵**********************************************************************/
    case INFANTRY_ROBOT_5:
        break;
    default:
        break;
    }
} 

static Graph_Data_t UI_pitch_line[10]; // pitch准线
static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_shoot_oval[10]; // 瞄准框架
static Graph_Data_t UI_graph_Arc[3];   // 功率圆弧
static Graph_Data_t UI_graph_Rectangle[3]; // 绘制射速矩形

static Graph_Data_t UI_State_dyn[6];  // 机器人状态,动态先add才能change

static uint32_t shoot_line_location[7] = {460, 380, 300, 538, 538, 959};
void InfantryStaticUI(referee_info_t *referee_recv_info)
{
    // 绘制发射基准线
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_Cyan, 2, 1029, shoot_line_location[0], 880, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_Cyan, 3, 750, shoot_line_location[1], 1160, shoot_line_location[1]);
    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 645, shoot_line_location[2], 1274, shoot_line_location[2]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 600, shoot_line_location[3], 920, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 1000, shoot_line_location[4], 1220, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Cyan, 4, shoot_line_location[5],300, shoot_line_location[5],505);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_shoot_line[5]);

    // 绘制pitch基准线
    UILineDraw(&UI_pitch_line[0], "so1", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1320, 538, 1341, 538);
    UILineDraw(&UI_pitch_line[1], "so2", UI_Graph_ADD, 7, UI_Color_Cyan, 15, 1307, 679, 1258, 651);
    UILineDraw(&UI_pitch_line[2], "so3", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1297, 600, 1329, 613);
    UILineDraw(&UI_pitch_line[3], "so4", UI_Graph_ADD, 7, UI_Color_Cyan, 15, 1191, 768, 1227, 804);
    UILineDraw(&UI_pitch_line[4], "so5", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 1254, 725, 1282, 742);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_pitch_line[0], UI_pitch_line[1], UI_pitch_line[2], UI_pitch_line[3], UI_pitch_line[4]); 
    
    UIArcDraw(&UI_pitch_line[5], "so6", UI_Graph_ADD, 5, UI_Color_Purplish_red, 88, 92, 20, 958, 538, 360, 360);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_pitch_line[5]);

    // 绘制功率圆弧
    UIArcDraw(&UI_graph_Arc[0], "ss0", UI_Graph_ADD, 7, UI_Color_Cyan, 45, 135, 6, 950, 540, 400, 400);
    UIArcDraw(&UI_graph_Arc[1], "ss1", UI_Graph_ADD, 7, UI_Color_Cyan, 225, 270, 15, 960, 530, 400, 400);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_graph_Arc[0], UI_graph_Arc[1]);
    
    UIArcDraw(&UI_graph_Arc[2], "ss2", UI_Graph_ADD, 7, UI_Color_Cyan, 0, 360, 15, 1524, 725, 100, 100);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Arc[2]);
    UIArcDraw(&UI_graph_Arc[3], "ss3", UI_Graph_ADD, 7, UI_Color_Cyan, 0, 0, 15, 1524, 725, 100, 100);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Arc[3]);

    // 绘制射速矩形
    UIRectangleDraw(&UI_graph_Rectangle[0], "sj1", UI_Graph_ADD, 7, UI_Color_Cyan, 4, 46, 834, 136, 858);

    UIRectangleDraw(&UI_graph_Rectangle[1], "sj2", UI_Graph_ADD, 7, UI_Color_Cyan, 5, 1481, 672, 1507, 780);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_graph_Rectangle[0], UI_graph_Rectangle[1]);
    UIRectangleDraw(&UI_graph_Rectangle[2], "sj3", UI_Graph_ADD, 7, UI_Color_Cyan, 5, 1555, 672, 1581, 780);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Rectangle[2]);

    UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Purplish_red, 4, 1481, 682, 1507, 682);
    UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_ADD, 7, UI_Color_Purplish_red, 4, 1555, 682, 1581, 682);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[7]); 
    
}
void InfantryDynamciscUI(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    // // shoot
    // if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    // {
    //     // UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
    //     // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);

    //     UIRectangleDraw(&UI_graph_Rectangle[0], "sj1", UI_Graph_ADD, 7, UI_Color_Purplish_red, 4, 46, 834, 136, 858);
    //     UIRectangleDraw(&UI_graph_Rectangle[1], "sj2", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5, 1481, 672, 1507, 780);
    //     UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_graph_Rectangle[0], UI_graph_Rectangle[1]);
    //     UIRectangleDraw(&UI_graph_Rectangle[2], "sj3", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5, 1555, 672, 1581, 780);
    //     UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Rectangle[2]);
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    // }
    if (_Interactive_data->chassis_offset_angle < 180 && _Interactive_data->chassis_offset_angle >= 0)
        UIArcDraw(&UI_graph_Arc[3], "ss3", UI_Graph_Change, 7, UI_Color_Pink, _Interactive_data->chassis_offset_angle - 10, _Interactive_data->chassis_offset_angle + 10, 15, 1524, 725, 100, 100);
    else if (_Interactive_data->chassis_offset_angle > -180 && _Interactive_data->chassis_offset_angle < 0)
        UIArcDraw(&UI_graph_Arc[3], "ss3", UI_Graph_Change, 7, UI_Color_Pink, 360 + (_Interactive_data->chassis_offset_angle - 10), 360 + (_Interactive_data->chassis_offset_angle + 10), 15, 1524, 725, 100, 100);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_graph_Arc[3]);

    UIArcDraw(&UI_pitch_line[5], "so6", UI_Graph_Change, 5, UI_Color_Purplish_red, 88 - _Interactive_data->pitch_angle, 92 - _Interactive_data->pitch_angle, 20, 958, 538, 360, 360);
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_pitch_line[5]);

    if (_Interactive_data->friction_mode == FRICTION_OFF)
    {
        UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1481, 682, 1507, 682);
        UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1555, 682, 1581, 682);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[7]); 
    }
    else 
    {
        if (_Interactive_data->bullet_speed == SMALL_AMU_15)
        {
            UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1481, 712, 1507, 712);
            UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1555, 712, 1581, 712);
            UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[7]); 
        }
        else if (_Interactive_data->bullet_speed == SMALL_AMU_18)
        {
            UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1481, 742, 1507, 742);
            UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1555, 742, 1581, 742);
            UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[7]); 
        }
        else
        {
            UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1481, 772, 1507, 772);
            UILineDraw(&UI_shoot_line[7], "sl7", UI_Graph_Change, 7, UI_Color_Purplish_red, 4, 1555, 772, 1581, 772);
            UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[6], UI_shoot_line[7]); 
        }        
    }



    // UILineDraw(&UI_pitch_line[5], "so6", UI_Graph_Change, 5, UI_Color_Purplish_red, 15, 1275, 538, 1310, 538);
    // UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_pitch_line[5]);
}