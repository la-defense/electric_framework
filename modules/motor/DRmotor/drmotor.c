// #include "drmotor.h"
// #include "memory.h"
// #include "general_def.h"
// #include "user_lib.h"
// #include "cmsis_os.h"
// #include "string.h"
// #include "daemon.h"
// #include "stdlib.h"
// #include "bsp_log.h"


    // Motor_Init_Config_s dr_motor_config = {
    //     .can_init_config.can_handle = &hcan1,
    //     .controller_param_init_config = {
    //         .speed_PID = {
    //             .Kp = 0, // 4.5
    //             .Ki = 0,  // 0
    //             .Kd = 0,  // 0
    //             .IntegralLimit = 3000,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .MaxOut = 0,
    //         },
    //         .current_PID = {
    //             .Kp = 0, // 0.4
    //             .Ki = 0,   // 0
    //             .Kd = 0,
    //             .IntegralLimit = 3000,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .MaxOut = 0,
    //         },
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = SPEED_LOOP,
    //         .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
    //     },
    //     .motor_type = DR04,
    // };   
    // dr_motor_config.can_init_config.tx_id = 0x01;
    // dr_motor_config.can_init_config.rx_id = 0x3E;
    // drmotor = DRMotorInit(&dr_motor_config);


// static uint8_t idx;
// static DRMotorInstance *dr_motor_instance[DR_MOTOR_CNT]; 
// static osThreadId dr_task_handle[DR_MOTOR_CNT];
// /* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
// static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
// {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
// }
// static float uint_to_float(int x_int, float x_min, float x_max, int bits)
// {
//     float span = x_max - x_min;
//     float offset = x_min;
//     return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
// }

// static void DRMotorSetMode(DRMotor_Mode_e cmd, DRMotorInstance *motor)
// {
//     memset(motor->motor_can_instace->tx_buff, 0, 7);  // 发送电机指令的时候前面7bytes都是0

//     motor->motor_can_instace->tx_buff[0] = (uint8_t)cmd;
//     motor->motor_can_instace->tx_buff[1] = (uint8_t)(cmd >> 8);
//     motor->motor_can_instace->tx_buff[2] = 0x03;
    
//     if (DR_CMD_READ_MOTOR == cmd || DR_CMD_TEMP_ERROR == cmd)  
//         motor->motor_can_instace->tx_buff[4] = 0x01;
//     else 
//         motor->motor_can_instace->tx_buff[4] = 0x00;
        
//     motor->motor_can_instace->tx_id = (uint8_t)(motor->measure.id << 5) + 0x1f;     
//     motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;

//     // if (DR_CMD_NO_READ == cmd)
//     // {
//     //     memset(motor->motor_can_instace->tx_buff, 0, 7);  // 发送电机指令的时候前面7bytes都是0
//     //     motor->motor_can_instace->tx_buff[2] = 0x48;
//     //     motor->motor_can_instace->tx_buff[3] = 0xC2;
//     //     motor->motor_can_instace->tx_buff[6] = 0x01;
//     //     motor->motor_can_instace->tx_id = 0x3C;     
//     //     motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;
//     // }
//     CANTransmit(motor->motor_can_instace, 1);
// }

// /**
//  * @brief 解析电机反馈值
//  *
//  * @param motor_can 收到
//  */
// static void DRMotorDecode(CANInstance *motor_can) 
// {
//     uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
//     uint8_t *rxbuff = motor_can->rx_buff;
//     DRMotorInstance *motor = (DRMotorInstance *)motor_can->id;
//     DR_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

//     DaemonReload(motor->motor_daemon);

//     measure->last_position = measure->position;
//     measure->position = *((float*)(&rxbuff[0]));

//     // tmp = (uint16_t)((rxbuff[5] << 8) | rxbuff[4]);
//     // measure->velocity = uint_to_float(tmp, DR_V_MIN, DR_V_MAX, 16); 

//     // tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
//     // measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

//     // measure->T_Mos = (float)rxbuff[6];
//     // measure->T_Rotor = (float)rxbuff[7];
// }

// static void DRMotorSaveConfig(DRMotorInstance *motor)
// {
//     memset(motor->motor_can_instace->tx_buff, 0, 7);  // 发送电机指令的时候前面7bytes都是0

//     motor->motor_can_instace->tx_buff[0] = 0x01;
//     motor->motor_can_instace->tx_id = (uint8_t)(motor->measure.id << 5) + 0x08; // tx_id是命令id
//     motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;

//     CANTransmit(motor->motor_can_instace, 1);
//     DWT_Delay(0.1);
// }

// static void DRMotorLostCallback(void *motor_ptr)
// {
//     DRMotorInstance *motor = (DRMotorInstance *)motor_ptr;
//     LOGWARNING("[dr_motor] motor %d lost\n", motor->motor_can_instace->tx_id);
// }

// // void DMMotorCaliEncoder(DMMotorInstance *motor)
// // {
// //     DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
// //     DWT_Delay(0.1);
// // }

// DRMotorInstance *DRMotorInit(Motor_Init_Config_s *config)
// {
//     DRMotorInstance *motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
//     memset(motor, 0, sizeof(DRMotorInstance));
    
//     motor->motor_settings = config->controller_setting_init_config;
//     PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
//     PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
//     PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
//     motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
//     motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

//     config->can_init_config.can_module_callback = DRMotorDecode;
//     config->can_init_config.id = motor;
//     motor->motor_can_instace = CANRegister(&config->can_init_config);

//     Daemon_Init_Config_s conf = {
//         .callback = DRMotorLostCallback,
//         .owner_id = motor,
//         .reload_count = 0.1,  // 10ms
//     }; 
//     motor->motor_daemon = DaemonRegister(&conf);

//     motor->measure.id = config->can_init_config.tx_id;  // 保存电机id,tx_id会因为cmd改变
//     DRMotorEnable(motor);
//     // DRMotorSetMode(DR_CMD_READ_MOTOR, motor);
//     DWT_Delay(0.1);
//     // DRMotorSetMode(DR_CMD_NO_LIMIT, motor);
//     // DWT_Delay(0.1);
//     // DRMotorSaveConfig(motor);
//     // DRMotorCaliEncoder(motor);
//     dr_motor_instance[idx++] = motor;   
//     return motor;
// }

// void DRMotorSetRef(DRMotorInstance *motor, float ref)
// {
//     motor->pid_ref = ref;
// }

// void DRMotorEnable(DRMotorInstance *motor)
// {
//     motor->stop_flag = MOTOR_ENALBED;
// }

// void DRMotorStop(DRMotorInstance *motor)//不使用使能模式是因为需要收到反馈
// {
//     motor->stop_flag = MOTOR_STOP;
// }

// void DRMotorOuterLoop(DRMotorInstance *motor, Closeloop_Type_e type)
// {
//     motor->motor_settings.outer_loop_type = type;
// }

// uint8_t DRMotorIsOnline(DRMotorInstance *motor)
// {
//     if(DaemonIsOnline(motor->motor_daemon)) 
//     {
//         motor->online_flag = MOTOR_ONLINE;
//         return MOTOR_ONLINE;
//     }
//     else
//     {
//         motor->online_flag = MOTOR_OFFLINE;
//         return MOTOR_OFFLINE;
//     }
// }

// /**
//  * @brief 为了避免总线堵塞,为每个电机创建一个发送任务
//  * @param argument 传入的电机指针
//  */
// //@Todo: 目前只实现了力控，更多位控PID等请自行添加
// __attribute__((noreturn)) void DRMotorTask(void const *argument)
// {
//     float  pid_ref, set;
//     DRMotorInstance *motor = (DRMotorInstance *)argument;
//     DR_Motor_Measure_s *measure = &motor->measure;
//     Motor_Control_Setting_s *setting = &motor->motor_settings;
//     CANInstance *motor_can = motor->motor_can_instace;
//     uint16_t tmp;
//     DRMotor_Send_s motor_send_mailbox;

//     while (1)
//     {
//         pid_ref = motor->pid_ref;

//         set = pid_ref;
//         if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
//             set *= -1;

//         // if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
//         // {
//         //     if (setting->angle_feedback_source == OTHER_FEED)
//         //         pid_measure = *motor->other_angle_feedback_ptr;
//         //     else
//         //         pid_measure = measure->total_angle;
//         //     // measure单位是rad,ref是角度,统一到angle下计算,方便建模
//         //     pid_ref = PIDCalculate(&motor->angle_PID, pid_measure * RAD_2_DEGREE, pid_ref);
//         // }

//         // if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
//         // {
//         //     if (setting->feedforward_flag & SPEED_FEEDFORWARD)
//         //         pid_ref += *motor->speed_feedforward_ptr;

//         //     if (setting->angle_feedback_source == OTHER_FEED)
//         //         pid_measure = *motor->other_speed_feedback_ptr;
//         //     else
//         //         pid_measure = measure->speed_rads;
//         //     // measure单位是rad / s ,ref是angle per sec,统一到angle下计算
//         //     pid_ref = PIDCalculate(&motor->speed_PID, pid_measure * RAD_2_DEGREE, pid_ref);
//         // }

//         // if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
//         //     pid_ref += *motor->current_feedforward_ptr;
//         // if (setting->close_loop_type & CURRENT_LOOP)
//         // {
//         //     pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
//         // }

//         // set = pid_ref;

//         // LIMIT_MIN_MAX(set, T_MIN, T_MAX);
//         // tmp = float_to_uint(set, T_MIN, T_MAX, 12);
//         // if (motor->stop_flag == MOTOR_STOP)
//         //     tmp = float_to_uint(0, T_MIN, T_MAX, 12);
//         // motor_can->tx_buff[6] = (tmp >> 8);
//         // motor_can->tx_buff[7] = tmp & 0xff;
//         // LIMIT_MIN_MAX(set, DR_T_MIN, DR_T_MAX);
//         // motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
//         // motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
//         // motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
//         // motor_send_mailbox.Kp = 0;
//         // motor_send_mailbox.Kd = 0;

//         // if(motor->stop_flag == MOTOR_STOP)
//         //     motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
        
//         // motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
//         // motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
//         // motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
//         // motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
//         // motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
//         // motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
//         // motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
//         // motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

//         // DRMotorSetMode(DR_CMD_READ_DATA, motor);
//         // memset(motor->motor_can_instace->tx_buff, 0, 7);
//         // motor->motor_can_instace->tx_id = (uint8_t)30;
//         // motor->motor_can_instace->tx_buff[0] = (uint8_t)0x72;
//         // motor->motor_can_instace->tx_buff[1] = (uint8_t)0x94;
//         // motor->motor_can_instace->tx_buff[2] = (uint8_t)0;
//         // motor->motor_can_instace->tx_buff[3] = (uint8_t)0;
//         // motor->motor_can_instace->tx_buff[4] = (uint8_t)0;
//         // motor->motor_can_instace->tx_buff[5] = (uint8_t)0;
//         // motor->motor_can_instace->tx_buff[6] = (uint8_t)0x01;
//         // motor->motor_can_instace->tx_buff[7] = (uint8_t)0;
//         // CANTransmit(motor->motor_can_instace, 1);
//         osDelay(2);
//     }
// }
// void DRMotorControlInit()
// {
//     char dr_task_name[5] = "dr";

//     // 遍历所有电机实例,创建任务
//     if (!idx)
//         return;
//     for (size_t i = 0; i < idx; i++)
//     {
//         char dr_id_buff[2] = {0};
//         __itoa(i, dr_id_buff, 10);
//         strcat(dr_task_name, dr_id_buff);
//         osThreadDef(dr_task_name, DRMotorTask, osPriorityNormal, 0, 128);
//         dr_task_handle[i] = osThreadCreate(osThread(dr_task_name), dr_motor_instance[i]);
//     }
// }