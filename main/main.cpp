#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "drive.h"

extern "C" void app_main(void)
{
    Drive *drive = new Drive();

    drive->initialize();

    drive->arm();

    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
