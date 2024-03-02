#ifndef _DRIVE_H_
#define _DRIVE_H_

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <freertos/FreeRTOS.h>

#include "cfg.h"

struct Timer
{
    i32                  group;
    mcpwm_timer_handle_t timer = nullptr;

    explicit inline Timer(i32 gp) : group(gp){};

    esp_err_t initialize(void);
    esp_err_t start(void);
    esp_err_t stop(void);
};

class Drive
{
    private:
    static constexpr f32 MAX_PULSE_WIDTH = 2000.0F;
    static constexpr f32 MIN_PULSE_WIDTH = 1000.0F;

    static constexpr f32 MAX_PERCENTAGE = 100.0F;
    static constexpr f32 MIN_PERCENTAGE = 0.0F;

    static constexpr f32        ARM_THROTTLE = 50.0F;
    static constexpr TickType_t ARM_DELAY    = pdMS_TO_TICKS(50);

    mcpwm_cmpr_handle_t comparator = nullptr;
    mcpwm_gen_handle_t  generator  = nullptr;
    gpio_num_t          pwm_pin    = GPIO_NUM_MAX;
    i32                 op_id      = 0;

    f32 throttle = 0.0F;

    u32 percentageToComparator(f32 value);

    public:
    inline Drive(gpio_num_t pin, i32 id) : pwm_pin(pin), op_id(id){};

    esp_err_t initialize(Timer *timer);

    esp_err_t arm(void);
    esp_err_t setPercentage(f32 pct = 0.0F);
};

#endif

