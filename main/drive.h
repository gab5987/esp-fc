#ifndef _DRIVE_H_
#define _DRIVE_H_

#include <array>
#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <freertos/FreeRTOS.h>

#include "cfg.h"

class Drive
{
    private:
    static constexpr u32 TIMEBASE_RESOLUTION_HZ =
        1000000;                             /* 1MHz, 1 us per tick */
    static constexpr f32 PWM_FREQUENCY = 50; /* 50 Hz */
    static constexpr u32 TIMEBASE_PERIOD =
        TIMEBASE_RESOLUTION_HZ / PWM_FREQUENCY;

    static constexpr gpio_num_t MOTOR1_PWM_PIN = GPIO_NUM_25;
    static constexpr gpio_num_t MOTOR2_PWM_PIN = GPIO_NUM_26;
    static constexpr gpio_num_t MOTOR3_PWM_PIN = GPIO_NUM_32;
    static constexpr gpio_num_t MOTOR4_PWM_PIN = GPIO_NUM_33;

    static constexpr u32 STEADY_STATE_VALUE = 1050;

    static constexpr f32 MAX_PULSE_WIDTH = 2000.0F;
    static constexpr f32 MIN_PULSE_WIDTH = 1000.0F;

    static constexpr f32 MAX_PERCENTAGE = 100.0F;
    static constexpr f32 MIN_PERCENTAGE = 0.0F;

    /* Those delays are kinda high but thats what it was said on the
     * datasheet...*/
    static constexpr TickType_t ARM_THROTTLE_TRIGGER_TIME = pdMS_TO_TICKS(5000);
    static constexpr TickType_t ARM_PREINIT_SAFETY_TIME   = pdMS_TO_TICKS(2000);
    static constexpr u32        ARM_TRIGGER_VALUE         = 1200;

    struct Timer
    {
        i32                  group;
        mcpwm_timer_handle_t timer = nullptr;

        explicit inline Timer(i32 gp) : group(gp){};

        esp_err_t initialize(void);
        esp_err_t start(void);
        esp_err_t stop(void);
    };

    struct MotorHandler
    {
        const char *tag;
        gpio_num_t  pwm_pin = GPIO_NUM_MAX;

        mcpwm_cmpr_handle_t  comparator = nullptr;
        mcpwm_gen_handle_t   generator  = nullptr;
        mcpwm_oper_handle_t *oper       = nullptr;

        f32 throttle = 0.0F;
    };

    static Timer               timer0;
    static mcpwm_oper_handle_t oper0;

    static Timer               timer1;
    static mcpwm_oper_handle_t oper1;

    std::array<MotorHandler, 4> power_train = {
        (MotorHandler){
            .tag     = "Forward Left(1)",
            .pwm_pin = MOTOR1_PWM_PIN,
            .oper    = &oper0,
        },
        (MotorHandler){
            .tag     = "Forward Right(2)",
            .pwm_pin = MOTOR2_PWM_PIN,
            .oper    = &oper0,
        },
        (MotorHandler){
            .tag     = "Backward Left(3)",
            .pwm_pin = MOTOR3_PWM_PIN,
            .oper    = &oper1,
        },
        (MotorHandler){
            .tag     = "Backward Right(4)",
            .pwm_pin = MOTOR4_PWM_PIN,
            .oper    = &oper1,
        },
    };

    esp_err_t initializeMotor(MotorHandler *motor);
    esp_err_t initializeTimer(
        Timer *timer, mcpwm_oper_handle_t *oper, i32 gpid);

    public:
    enum Motor
    {
        FORWARD_LEFT   = 0,
        FORWARD_RIGHT  = 1,
        BACKWARD_LEFT  = 2,
        BACKWARD_RIGHT = 3,
    };

    esp_err_t initialize(void);

    esp_err_t arm(void);
    esp_err_t setPercentage(Motor motor, f32 pct = 0.0F);
};

#endif

