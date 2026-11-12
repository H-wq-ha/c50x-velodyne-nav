#include "board.h"
#include "app_rtos.h"

/* STM32F103 TIM2 is 16 bit. Extend it; do not assume F4's 32-bit counter. */
static volatile uint64_t overflow_base;

void Board_CaptureInit(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_AFIO_REMAP_TIM2_DISABLE();
    uint32_t hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) hz *= 2U;
    if (hz < 1000000U || hz % 1000000U != 0U) Board_Fatal(0x33U);
    GPIO_InitTypeDef pin = {0};
    pin.Pin = GPIO_PIN_0;
    pin.Mode = GPIO_MODE_INPUT;
    pin.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &pin);
    TIM2->PSC = hz / 1000000U - 1U;
    TIM2->ARR = 65535U;
    /* CH1=TI1 (PA0 IMU INT). CH2=TRC (internal PPS, NO extra jumper).
     * TS=ITR0 selects TIM1; SMS=0, so PPS does not reset the counter.
     */
    TIM2->SMCR = 0U;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S;
    TIM2->CCER = TIM_CCER_CC2E;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CNT = 0U;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE | TIM_DIER_CC2IE;
    HAL_NVIC_SetPriority(TIM2_IRQn, 6U, 0U); /* FromISR permitted, >=5. */
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    __HAL_DBGMCU_FREEZE_TIM2();
    __HAL_DBGMCU_FREEZE_TIM4();
}

void Board_EnableImuCapture(bool enabled)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    TIM2->CCER &= ~TIM_CCER_CC1E;
    TIM2->DIER &= ~TIM_DIER_CC1IE;
    TIM2->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
    if (enabled) {
        TIM2->DIER |= TIM_DIER_CC1IE;
        TIM2->CCER |= TIM_CCER_CC1E;
    }
    __set_PRIMASK(saved);
}

uint64_t Board_MonotonicUs(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    uint64_t base = overflow_base;
    uint32_t phase = TIM2->CNT;
    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        base += 65536ULL;
        phase = TIM2->CNT;
    }
    __set_PRIMASK(saved);
    return base + phase;
}
uint32_t Board_Micros(void) { return (uint32_t)Board_MonotonicUs(); }
void Board_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) __NOP();
}
bool Board_Stamp(uint64_t monotonic_us, sync_stamp_t *stamp)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    bool valid = Clock_Stamp(&g_sync_clock, monotonic_us, stamp);
    __set_PRIMASK(saved);
    return valid;
}

void TIM2_IRQHandler(void)
{
    uint32_t flags = TIM2->SR;
    bool wrap = (flags & TIM_SR_UIF) != 0U;
    uint64_t base = overflow_base;
    uint16_t imu_capture = 0U, pps_capture = 0U;
    /* SR then CCR read clears CCxIF. Preserve new CCxIF after that read. */
    if ((flags & TIM_SR_CC1IF) != 0U) imu_capture = (uint16_t)TIM2->CCR1;
    if ((flags & TIM_SR_CC2IF) != 0U) pps_capture = (uint16_t)TIM2->CCR2;
    TIM2->SR = ~(flags & (TIM_SR_UIF | TIM_SR_CC1OF | TIM_SR_CC2OF));
    if (wrap) overflow_base = base + 65536ULL;
    if ((flags & TIM_SR_CC2OF) != 0U) Board_Fatal(0x34U);
    if ((flags & TIM_SR_CC2IF) != 0U)
        Clock_OnPps(&g_sync_clock, Clock_ExtendCapture(base, pps_capture, wrap));
    if ((flags & TIM_SR_CC1IF) != 0U)
        App_ImuCaptureFromISR(Clock_ExtendCapture(base, imu_capture, wrap),
                              (flags & TIM_SR_CC1OF) != 0U);
}
