#include "board.h"
#if APP_ENABLE_OLED
soft_i2c_t oled_i2c = {
    OLED_SCL_PORT, OLED_SCL_PIN, OLED_SDA_PORT, OLED_SDA_PIN, APP_SOFT_I2C_HALF_PERIOD_US
};
#endif
soft_i2c_t imu_i2c = {
    IMU_SCL_PORT, IMU_SCL_PIN, IMU_SDA_PORT, IMU_SDA_PIN, APP_SOFT_I2C_HALF_PERIOD_US
};
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
volatile app_stage_t g_app_stage = APP_STAGE_RESET;
volatile uint32_t g_fault_code;
board_clock_t g_board_clock;

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* Keep SWD enabled for ST-Link. */
}
static HAL_StatusTypeDef clock_pll(bool use_hse)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clock = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.HSEState = use_hse ? RCC_HSE_ON : RCC_HSE_OFF;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = use_hse ? RCC_PLLSOURCE_HSE : RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = use_hse ? RCC_PLL_MUL9 : RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        return HAL_ERROR;
    }
    clock.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV2;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_2);
}
void SystemClock_Config(void)
{
    g_app_stage = APP_STAGE_CLOCK;
    bool hse_ok = false;
#if !APP_FORCE_HSI
    hse_ok = clock_pll(true) == HAL_OK;
#endif
    if (!hse_ok) {
        if (HAL_RCC_DeInit() != HAL_OK || clock_pll(false) != HAL_OK) {
            Board_Fatal(1U);
        }
    }
    SystemCoreClockUpdate();
    g_board_clock.sysclk_hz = HAL_RCC_GetSysClockFreq();
    g_board_clock.hse_in_use = hse_ok;
    if (g_board_clock.sysclk_hz != (hse_ok ? 72000000U : 64000000U)) {
        Board_Fatal(2U);
    }
    /* HAL_RCC_ClockConfig refreshes the TIM2 HAL timebase from actual HCLK. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
void Board_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock/1000000U)*us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}
uint32_t Board_Micros(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t ms = HAL_GetTick();
    uint32_t phase = __HAL_TIM_GET_COUNTER(&htim2);
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        ++ms;
        phase = __HAL_TIM_GET_COUNTER(&htim2);
    }
    __set_PRIMASK(primask);
    return ms*1000U + phase;
}
void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART1) {
        return;
    }
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
    __HAL_LINKDMA(uart, hdmatx, hdma_usart1_tx);
    HAL_NVIC_SetPriority(USART1_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}
bool Board_PeripheralsInit(void)
{
    g_app_stage = APP_STAGE_PERIPHERALS;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init(LED_PORT, &gpio);
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) {
        return false;
    }
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    huart1.Instance = USART1;
    huart1.Init.BaudRate = APP_UART_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    __HAL_RCC_GPIOB_CLK_ENABLE();
#if APP_ENABLE_OLED
    SoftI2C_Init(&oled_i2c);
#endif
    SoftI2C_Init(&imu_i2c);
    return HAL_UART_Init(&huart1) == HAL_OK;
}
void Board_SetLed(bool on)
{
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void Board_Fatal(uint32_t code)
{
    g_app_stage = APP_STAGE_FATAL;
    g_fault_code = code;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &gpio);
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        for (volatile uint32_t i = 0; i < 500000U; ++i) {
            __NOP();
        }
    }
}
