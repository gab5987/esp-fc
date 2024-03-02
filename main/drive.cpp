#include <esp_log.h>

#include "drive.h"

static constexpr char TAG[] = "drive";

static constexpr u32 TIMEBASE_RESOLUTION_HZ = 1000000; /* 1MHz, 1 us per tick */
static constexpr f32 PWM_FREQUENCY          = 50;      /* 50 Hz */
static constexpr u32 TIMEBASE_PERIOD = TIMEBASE_RESOLUTION_HZ / PWM_FREQUENCY;

/**
 * @brief Allocates space and initiates a new hardware timer pwm.
 *
 * @return
 * - ESP_OK On success.
 * */
esp_err_t Timer::initialize(void)
{
    mcpwm_timer_config_t timer_config = {
        .group_id      = this->group,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = TIMEBASE_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = TIMEBASE_PERIOD,
    };

    return mcpwm_new_timer(&timer_config, &this->timer);
}

/**
 * @brief Starts the timer in the NO_STOP mode.
 *
 * @return
 * - ESP_OK If the timer could be enabled and started.
 * - ESP_ERR_INVALID_* In case of a program fault.
 * - ESP_FAIL In case of panic.
 * */
esp_err_t Timer::start(void)
{
    esp_err_t ret = mcpwm_timer_enable(this->timer);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(
            TAG, "Failed to enable timer %" PRIi32 ": %s", this->group, name);

        return ret;
    }

    ret = mcpwm_timer_start_stop(this->timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(
            TAG, "Failed to start timer %" PRIi32 ": %s", this->group, name);

        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Stops the timer pwm.
 *
 * @return
 * - ESP_OK If the timer could be stopped.
 * - ESP_ERR_INVALID_* In case of a program fault.
 * - ESP_FAIL In case of panic.
 * */
esp_err_t Timer::stop(void)
{
    return mcpwm_timer_disable(this->timer);
}

/**
 * @brief Sets up all the requirements for the driver, such as comparators,
 * generators and timers.
 *
 * @param timer Pointer to a previously setup timer.
 *
 * @return
 * - ESP_OK If the driver is setup correctly.
 * - ESP_ERR_INVALID_ARG If the timer pointer is null.
 * - Other mcpwm errors otherwise.
 * */
esp_err_t Drive::initialize(Timer *timer)
{
    if (timer == nullptr)
    {
        ESP_LOGE(TAG, "Received Timer obj was a nullptr");
        return ESP_ERR_INVALID_ARG;
    }

    mcpwm_oper_handle_t     oper            = nullptr;
    mcpwm_operator_config_t operator_config = {.group_id = this->op_id};

    esp_err_t ret = mcpwm_new_operator(&operator_config, &oper);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(
            TAG, "Could not create a new operator for timer %" PRIi32 ": %s",
            timer->group, name);

        return ret;
    }

    ret = mcpwm_operator_connect_timer(oper, timer->timer);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(
            TAG,
            "Could not connect the timer %" PRIi32 " to the new operator: %s",
            timer->group, name);

        return ret;
    }

    mcpwm_generator_config_t generator_config;
    generator_config.gen_gpio_num = this->pwm_pin;

    mcpwm_comparator_config_t comparator_config = {};
    // comparator_config.intr_priority             = 0;
    comparator_config.flags.update_cmp_on_tez = true;

    mcpwm_gen_timer_event_action_t   tm_act;
    mcpwm_gen_compare_event_action_t cm_act;

    ret = mcpwm_new_comparator(oper, &comparator_config, &this->comparator);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Could not create a new comparator: %s", name);

        return ret;
    }

    ret = mcpwm_new_generator(oper, &generator_config, &this->generator);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Could not create a new generator: %s", name);

        return ret;
    }

    tm_act = MCPWM_GEN_TIMER_EVENT_ACTION(
        MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
        MCPWM_GEN_ACTION_HIGH);

    ret = mcpwm_generator_set_action_on_timer_event(this->generator, tm_act);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Failed to setup the generator action: %s", name);

        return ret;
    }

    cm_act = MCPWM_GEN_COMPARE_EVENT_ACTION(
        MCPWM_TIMER_DIRECTION_UP, this->comparator, MCPWM_GEN_ACTION_LOW);

    ret = mcpwm_generator_set_action_on_compare_event(this->generator, cm_act);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Failed to setup the comparator action: %s", name);

        return ret;
    }

    // for (auto &handler : this->handlers) pwmpinConfig(&handler, &oper);

    return ESP_OK;
}

/**
 * @brief Sets the throttle percentage of this driver.
 *
 * @param pct Percentage value, if not in range it will be saturated.
 *
 * @return
 * - ESP_OK If the comparator is set up correctly.
 * - ESP_ERR_INVALID_* In case of a program fault.
 * - ESP_FAIL In case of panic.
 * */
esp_err_t Drive::setPercentage(f32 pct)
{
    static constexpr f32 PULSE_DIFF = MAX_PULSE_WIDTH - MIN_PULSE_WIDTH;
    static constexpr f32 VAL_DIFF   = MAX_PERCENTAGE - MIN_PERCENTAGE;

    if (pct > MAX_PERCENTAGE)
        pct = MAX_PERCENTAGE;
    else if (pct < MIN_PERCENTAGE)
        pct = MIN_PERCENTAGE;

    u32 ticks =
        (pct - MIN_PERCENTAGE) * PULSE_DIFF / VAL_DIFF + MIN_PULSE_WIDTH;

    ESP_LOGI(
        TAG, "Engine %" PRIi32 " throttle pecentage change: %.2f -> %.2f",
        this->op_id, this->throttle, pct);

    this->throttle = pct;

    return mcpwm_comparator_set_compare_value(this->comparator, ticks);
}

/**
 * @brief Sends the 'arm' command to the ESC and sets the throttle to zero.
 *
 * @return
 * - ESP_OK On success.
 * */
esp_err_t Drive::arm(void)
{
    esp_err_t ret = this->setPercentage(ARM_THROTTLE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Arm failed for %" PRIi32 "! The percentage level could not be set",
            this->op_id);
        return ret;
    }

    vTaskDelay(ARM_DELAY);

    return this->setPercentage();
}

