#include "board.h"
#include "app_rtos.h"

int main(void)
{
    SCB->VTOR = FLASH_BASE;
    __DSB();
    __ISB();
    SystemCoreClockUpdate();
    g_gps.stage = GPS_STAGE_HAL;
    if (HAL_Init() != HAL_OK) Board_Fatal(1U);
    SystemClock_Config();
    Board_PeripheralsInit();
    App_Start();
}
