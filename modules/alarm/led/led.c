#include "bsp_pwm.h"
#include "bsp_dwt.h"
#include "led.h"
#include "string.h"

static PWMInstance *led_r, *led_g, *led_b;

static LEDInstance *led;

/**
 * @brief LED灯初始化
 *
 */
void LEDInit()
{
    PWM_Init_Config_s led_config = {
        .htim = &htim5,
        .dutyratio = 0,
        .period = 0.001,
    };

    led_config.channel = TIM_CHANNEL_3;
    led_r = PWMRegister(&led_config);

    led_config.channel = TIM_CHANNEL_2;
    led_g = PWMRegister(&led_config);

    led_config.channel = TIM_CHANNEL_1;
    led_b = PWMRegister(&led_config);
}

LEDInstance *LEDRegister(LED_config_s *config)
{
    LEDInstance *led_temp = (LEDInstance *)malloc(sizeof(LEDInstance));
    memset(led_temp, 0, sizeof(LEDInstance));

    led_temp->color = config->color;
    led_temp->flash = config->flash;
    led->flash_count = config->flash_count;
    led_temp->alarm_state = ALARM_OFF;

    led = led_temp;
    return led_temp;
}

void LEDSetStatus(LEDInstance *led, AlarmState_e state)
{
    led->alarm_state = state;
}

void LEDSetFlash(LEDInstance *led, color_e color, flash_e flash)
{
    led->color = color;
    led->flash = flash;
}

void LEDSetFlashTime(LEDInstance *led, uint8_t flash_count)
{
    led->flash_count = flash_count;
}

/**
 * @brief          设置LED灯颜色和频率
 * @param[in]      color: 红色/黄色/蓝色/绿色/粉色，[RED/YELLOW/BLUE/GREEN/PINK]
 * @param[in]      flash: 快闪/慢闪/常亮，[FLASH_HIGH/FLASH_LOW/ALWAYS_ON]
 * @note           粉色为呼吸灯变化
 * @retval         none 
 */

static void LEDShowFlash(color_e color, flash_e flash)
{
    static uint16_t time_temp;
    static uint32_t color_temp;
    static uint16_t n, count;
    static int8_t temp, alpha;

    if (flash == FLASH_HIGH)
        time_temp = HightTime;
    else if (flash == FLASH_LOW)
        time_temp = LowTime;

    n++;
    if (n >= time_temp || flash == ALWAYS_ON)
    {
        switch (color)
        {
        case RED:
            PWMSetDutyRatio(led_r, 1.0f);
            PWMSetDutyRatio(led_g, 0.0f);
            PWMSetDutyRatio(led_b, 0.0f);
            break;
        case YELLOW:
            PWMSetDutyRatio(led_r, 1.0f);
            PWMSetDutyRatio(led_g, 1.0f);
            PWMSetDutyRatio(led_b, 0.0f);   
            break; 
        case BLUE:
            PWMSetDutyRatio(led_r, 0.0f);
            PWMSetDutyRatio(led_g, 0.0f);
            PWMSetDutyRatio(led_b, 1.0f);
            break;
        case GREEN:
            PWMSetDutyRatio(led_r, 0.0f);
            PWMSetDutyRatio(led_g, 1.0f);
            PWMSetDutyRatio(led_b, 0.0f);   
            break;  
        case PINK: 
            temp++;
            if (temp > 1)
            {
                temp = 0;
                alpha += 2;
            }

            PWMSetDutyRatio(led_r, (1.0f / (255.0f - 100.0f)) * (float)abs(alpha));
            PWMSetDutyRatio(led_g, 0.0f);
            PWMSetDutyRatio(led_b, (1.0f / (255.0f - 100.0f)) * (float)abs(alpha));   
            break;           
        default:
            break;
        }
    }
    
    if (n < time_temp && flash != ALWAYS_ON)
    {
        PWMSetDutyRatio(led_r, 0);
        PWMSetDutyRatio(led_g, 0);
        PWMSetDutyRatio(led_b, 0);           
    }

    if ((count >= led->flash_count && flash == FLASH_HIGH) || n >= 2 * time_temp)
    {
        n = 0;
        count++;
    }

    if (count >= 4 * time_temp)
    {
        count = 0;
    }            
}

void LEDTask()
{
    if (led->alarm_state == ALARM_OFF)
    {
        LEDShowFlash(PINK, ALWAYS_ON);
    }
    else
    {
        LEDShowFlash(led->color, led->flash);
    }
}
