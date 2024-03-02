#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "drive.h"

extern "C" void app_main(void)
{
    static constexpr gpio_num_t MOTOR4_PWM_PIN = GPIO_NUM_33;

    Timer timer0 = Timer(0);

    Drive *drive = new Drive(MOTOR4_PWM_PIN, 0);
    drive->initialize(&timer0);

    drive->arm();

    drive->setPercentage(10);

    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
