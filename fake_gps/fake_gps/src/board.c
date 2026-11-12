#include "board.h"
#include "gps_config.h"
#include "app_config.h"
#include "serial_hal.h"

volatile gps_diagnostics_t g_gps;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
soft_i2c_t imu_i2c = {
    IMU_SCL_PORT, IMU_SCL_PIN, IMU_SDA_PORT, IMU_SDA_PIN, APP_SOFT_I2C_HALF_PERIOD_US
};
sync_clock_t g_sync_clock;
static gps_time_t current_time = {
    GPS_START_YEAR, GPS_START_MONTH, GPS_START_DAY,
    GPS_START_HOUR, GPS_START_MINUTE, GPS_START_SECOND
};
static volatile bool sentence_pending;
static gps_time_t pending_time;
static uint32_t pending_sequence;
static bool first_pps = true;

_Static_assert(HSE_VALUE == 8000000U, "PLL x9 requires an 8MHz HSE crystal");
_Static_assert(GPS_PERIOD_TICKS <= 65536U, "TIM1 is 16 bit");
_Static_assert(GPS_PPS_COMPARE > GPS_TX_LATEST_TICK, "Invalid NMEA window");
_Static_assert(GPS_RMC_SECOND_OFFSET == 0 || GPS_RMC_SECOND_OFFSET == 1,
               "Only 0 (normal) and +1 second associations are supported");

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* PA13/PA14 SWD remains available. */
}

static HAL_StatusTypeDef clock_pll(bool hse)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.HSEState = hse ? RCC_HSE_ON : RCC_HSE_OFF; /* Crystal, NOT bypass. */
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = hse ? RCC_PLLSOURCE_HSE : RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = hse ? RCC_PLL_MUL9 : RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return HAL_ERROR;
    RCC_ClkInitTypeDef clock = {0};
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
    g_gps.stage = GPS_STAGE_CLOCK;
    /* Safe even if another startup has already selected PLL: switch to HSI
     * BEFORE reconfiguring PLL. Do not call SystemInit() again from main.
     * TIM4 HAL tick must remain active for every RCC timeout below.
     */
    if (HAL_RCC_DeInit() != HAL_OK) Board_Fatal(1U);
    bool hse = false;
    if (!GPS_FORCE_HSI) hse = clock_pll(true) == HAL_OK;
    if (!hse) {
        if (HAL_RCC_DeInit() != HAL_OK || clock_pll(false) != HAL_OK) Board_Fatal(2U);
    }
    SystemCoreClockUpdate();
    g_gps.sysclk_hz = HAL_RCC_GetSysClockFreq();
    g_gps.hse_in_use = hse ? 1U : 0U;
    if (g_gps.sysclk_hz != (hse ? 72000000U : 64000000U)) Board_Fatal(3U);
    /* HAL_RCC_ClockConfig reinitializes the TIM4 HAL tick from actual HCLK. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (hse) HAL_RCC_EnableCSS();
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_AFIO_REMAP_USART2_DISABLE();
        GPIO_InitTypeDef pc = {0};
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        pc.Pin = GPIO_PIN_2;
        pc.Mode = GPIO_MODE_AF_PP;
        pc.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &pc);
        pc.Pin = GPIO_PIN_3;
        pc.Mode = GPIO_MODE_INPUT;
        pc.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &pc);
        __HAL_LINKDMA(uart, hdmatx, hdma_usart2_tx);
        HAL_NVIC_SetPriority(USART2_IRQn, 7U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        return;
    }
    if (uart->Instance != USART1) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_AFIO_REMAP_USART1_DISABLE();
    GPIO_InitTypeDef pin = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); /* UART idle HIGH. */
    pin.Pin = GPIO_PIN_9;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &pin);
    __HAL_LINKDMA(uart, hdmatx, hdma_usart1_tx);
    HAL_NVIC_SetPriority(USART1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn); /* Required: DMA completion -> UART TC. */
}

void Board_PeripheralsInit(void)
{
    g_gps.stage = GPS_STAGE_PERIPHERALS;
    if (!gps_time_valid(&current_time)) Board_Fatal(4U);
    g_sync_clock.epoch_start_us = Clock_EpochUs(&current_time);
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    pin.Pin = GPIO_PIN_13;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pin);

    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK) Board_Fatal(5U);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    huart1.Instance = USART1;
    huart1.Init.BaudRate = GPS_UART_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Board_Fatal(6U);

    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init = hdma_usart1_tx.Init;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) Board_Fatal(0x31U);
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    huart2.Instance = USART2;
    huart2.Init = huart1.Init;
    huart2.Init.BaudRate = APP_UART_BAUD;
    huart2.Init.Mode = UART_MODE_TX_RX;
    if (HAL_UART_Init(&huart2) != HAL_OK) Board_Fatal(0x32U);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    SoftI2C_Init(&imu_i2c);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_AFIO_REMAP_TIM1_DISABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    pin.Pin = GPIO_PIN_8;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &pin); /* Remain LOW until timer is fully prepared. */

    uint32_t timer_hz = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) timer_hz *= 2U;
    g_gps.timer_hz = timer_hz;
    if (timer_hz % GPS_TIMER_HZ != 0U || timer_hz / GPS_TIMER_HZ > 65536U)
        Board_Fatal(7U);
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
    TIM1->PSC = timer_hz / GPS_TIMER_HZ - 1U;
    TIM1->ARR = GPS_PERIOD_TICKS - 1U;
    TIM1->CCR1 = GPS_PPS_COMPARE;
    TIM1->CCR2 = GPS_NMEA_COMPARE;
    /* PWM2 CH1: hardware drives both PPS edges. CH2 is internal timing only;
     * CC2E stays clear, so PA9 is exclusively USART1_TX, NOT TIM1_CH2.
     */
    TIM1->CCMR1 = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE;
    TIM1->CCER = TIM_CCER_CC1E;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->CR2 = TIM_CR2_MMS_2; /* OC1REF -> TRGO -> TIM2 ITR0 -> IC2. */
    TIM1->EGR = TIM_EGR_UG; /* Load prescaler before the FIRST interval. */
    TIM1->SR = 0U;
    TIM1->CNT = 0U;
    TIM1->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE;
    TIM1->BDTR = TIM_BDTR_MOE; /* Advanced timer output enable, mandatory. */
    pin.Mode = GPIO_MODE_AF_PP;
    HAL_GPIO_Init(GPIOA, &pin); /* PWM2 at CNT=0 is inactive LOW. */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0U, 0U);
    HAL_NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    __HAL_DBGMCU_FREEZE_TIM1(); /* Debug breakpoints do not emit stale PPS. */
    Board_CaptureInit();
}

void Board_TimerStart(void)
{
    g_gps.stage = GPS_STAGE_RUNNING;
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void TIM1_CC_IRQHandler(void)
{
    uint32_t flags = TIM1->SR & TIM1->DIER;
    /* W0C register: do not use read/modify/write to clear timer flags. */
    TIM1->SR = ~flags;
    if ((flags & TIM_SR_CC1IF) != 0U) {
        if (first_pps) first_pps = false;
        else gps_time_advance(&current_time);
        ++g_gps.pps_count;
        if (!gps_time_valid(&current_time)) Board_Fatal(8U);
    }
    if ((flags & TIM_SR_CC2IF) != 0U && !first_pps) {
        if (sentence_pending) ++g_gps.nmea_dropped;
        pending_time = current_time;
        if (GPS_RMC_SECOND_OFFSET == 1) gps_time_advance(&pending_time);
        pending_sequence = g_gps.pps_count;
        sentence_pending = true;
    }
}

bool GPS_TakeSentence(gps_time_t *time, uint32_t *sequence)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool ready = sentence_pending;
    if (ready) {
        *time = pending_time;
        *sequence = pending_sequence;
        sentence_pending = false;
    }
    __set_PRIMASK(primask);
    return ready;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) ++g_gps.nmea_sent;
    else if (uart->Instance == USART2) Serial_OnTxComplete();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2) Serial_OnRxComplete();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) {
        ++g_gps.uart_errors;
        Board_Fatal(9U); /* Stop PPS if sentences cannot be delivered. */
    } else if (uart->Instance == USART2) Serial_OnError();
}

void Board_Fatal(uint32_t code)
{
    __disable_irq();
    g_gps.stage = GPS_STAGE_FATAL;
    g_gps.fault_code = code;
    /* GPIO overrides peripheral outputs even after CSS changed the clock. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef pin = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    pin.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &pin);
    __HAL_RCC_GPIOC_CLK_ENABLE();
    pin.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &pin);
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t i = 0U; i < 150000U; ++i) __NOP();
    }
}
