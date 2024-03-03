#include <esp_log.h>

#include "drive.h"

static constexpr char TAG[] = "drive";

/**
 * @brief Allocates space and initiates a new hardware timer pwm.
 *
 * @return
 * - ESP_OK On success.
 * */
esp_err_t Drive::Timer::initialize(void)
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
esp_err_t Drive::Timer::start(void)
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
esp_err_t Drive::Timer::stop(void)
{
    return mcpwm_timer_disable(this->timer);
}

/**
 * @brief Initializes a timer group and a operator.
 *
 * @param timer Pointer to a set up timer.
 * @param oper Pointer to a statically allocated operator object.
 * @param gpid The id of the group.
 *
 * @return
 * - ESP_OK On success.
 * */
esp_err_t Drive::initializeTimer(
    Timer *timer, mcpwm_oper_handle_t *oper, i32 gpid)
{
    esp_err_t ret = timer->initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize timer %" PRIi32, gpid);
        return ret;
    }

    mcpwm_operator_config_t operator_config = {
        .group_id      = gpid,
        .intr_priority = 0,
    };

    ret = mcpwm_new_operator(&operator_config, oper);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(
            TAG, "Could not create a new operator for timer %" PRIi32 ": %s",
            timer0.group, name);

        return ret;
    }

    ret = mcpwm_operator_connect_timer(*oper, timer->timer);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(
            TAG,
            "Could not connect the timer %" PRIi32 " to the new operator: %s",
            timer0.group, name);

        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Sets up all the requirements for the driver, such as comparators,
 * generators and timers.
 *
 * @param motor Pointer to a previously setup MotorHandler object.
 *
 * @return
 * - ESP_OK If the driver is setup correctly.
 * - ESP_ERR_INVALID_ARG If the timer pointer is null.
 * - Other mcpwm errors otherwise.
 * */
esp_err_t Drive::initializeMotor(MotorHandler *motor)
{
    // if (timer == nullptr)
    // {
    //     ESP_LOGE(TAG, "Received Timer obj was a nullptr");
    //     return ESP_ERR_INVALID_ARG;
    // }

    mcpwm_generator_config_t generator_config;
    generator_config.gen_gpio_num = motor->pwm_pin;

    mcpwm_comparator_config_t comparator_config = {};
    // comparator_config.intr_priority             = 0;
    comparator_config.flags.update_cmp_on_tez = true;

    esp_err_t ret = mcpwm_new_comparator(
        *motor->oper, &comparator_config, &motor->comparator);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Could not create a new comparator: %s", name);

        return ret;
    }

    ret =
        mcpwm_new_generator(*motor->oper, &generator_config, &motor->generator);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Could not create a new generator: %s", name);

        return ret;
    }

    mcpwm_gen_timer_event_action_t tm_act = MCPWM_GEN_TIMER_EVENT_ACTION(
        MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
        MCPWM_GEN_ACTION_HIGH);

    ret = mcpwm_generator_set_action_on_timer_event(motor->generator, tm_act);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Failed to setup the generator action: %s", name);

        return ret;
    }

    mcpwm_gen_compare_event_action_t cm_act = MCPWM_GEN_COMPARE_EVENT_ACTION(
        MCPWM_TIMER_DIRECTION_UP, motor->comparator, MCPWM_GEN_ACTION_LOW);

    ret = mcpwm_generator_set_action_on_compare_event(motor->generator, cm_act);
    if (ret != ESP_OK)
    {
        const char *name = esp_err_to_name(ret);
        ESP_LOGE(TAG, "Failed to setup the comparator action: %s", name);

        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Sets the throttle percentage of a motor.
 *
 * @param motor Which motor index to set the percentage.
 * @param pct Percentage value, if not in range it will be saturated.
 *
 * @return
 * - ESP_OK If the comparator is set up correctly.
 * - ESP_ERR_INVALID_* In case of a program fault.
 * - ESP_FAIL In case of panic.
 * */
esp_err_t Drive::setPercentage(Motor motor, f32 pct)
{
    static constexpr f32 PULSE_DIFF = MAX_PULSE_WIDTH - MIN_PULSE_WIDTH;
    static constexpr f32 VAL_DIFF   = MAX_PERCENTAGE - MIN_PERCENTAGE;

    if (pct > MAX_PERCENTAGE)
        pct = MAX_PERCENTAGE;
    else if (pct < MIN_PERCENTAGE)
        pct = MIN_PERCENTAGE;

    u32 ticks =
        (pct - MIN_PERCENTAGE) * PULSE_DIFF / VAL_DIFF + MIN_PULSE_WIDTH;

    MotorHandler *mt = &this->power_train[motor];

    ESP_LOGI(
        TAG,
        "Engine %s throttle pecentage change: %.2f%% -> %.2f%% (%" PRIu32
        " us)",
        mt->tag, mt->throttle, pct, ticks);

    mt->throttle = pct;

    return mcpwm_comparator_set_compare_value(mt->comparator, ticks);
}

/**
 * @brief Sends the 'arm' command to the ESC and sets the throttle to zero.
 *
 * @return
 * - ESP_OK On success.
 * */
esp_err_t Drive::arm(void)
{
    ESP_LOGI(TAG, "Running ESC ARM sequence...");

    // More info can be found on the ESC documentation.
    // https://bluerobotics.com/wp-content/uploads/2018/10/BLHeli_S-manual-SiLabs-Rev16.x.pdf

    for (MotorHandler &mt : this->power_train)
        mcpwm_comparator_set_compare_value(mt.comparator, ARM_TRIGGER_VALUE);

    vTaskDelay(ARM_THROTTLE_TRIGGER_TIME);

    for (MotorHandler &mt : this->power_train)
        mcpwm_comparator_set_compare_value(mt.comparator, MIN_PULSE_WIDTH);

    vTaskDelay(ARM_PREINIT_SAFETY_TIME);

    for (MotorHandler &mt : this->power_train)
        mcpwm_comparator_set_compare_value(mt.comparator, STEADY_STATE_VALUE);

    ESP_LOGI(TAG, "ESC ARM sequence finished");

    return ESP_OK;
}

/**
 * @brief Initializes all the timers and motors.
 *
 * @return
 * - ESP_OK On success.
 * */
esp_err_t Drive::initialize(void)
{
    this->initializeTimer(&timer0, &oper0, 0);
    this->initializeTimer(&timer1, &oper1, 1);

    for (MotorHandler &mt : this->power_train) this->initializeMotor(&mt);

    this->timer0.start();
    this->timer1.start();

    return ESP_OK;
}

Drive::Timer        Drive::timer0 = Drive::Timer(0);
mcpwm_oper_handle_t Drive::oper0  = nullptr;

Drive::Timer        Drive::timer1 = Drive::Timer(1);
mcpwm_oper_handle_t Drive::oper1  = nullptr;

