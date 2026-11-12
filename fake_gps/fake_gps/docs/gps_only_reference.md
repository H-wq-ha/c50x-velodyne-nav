> 历史 GPS-only 版本说明；当前联合固件以工程根目录 README.md 为准。

# STM32F103C8 → VLP-16 模拟 GPS 授时

工程路径：`tools/fake_gps/fake_gps`（相对仓库根目录）。使用 PlatformIO + STM32Cube HAL，适用于 **8MHz 外部晶振的 STM32F103C8T6**，按真实 64KB Flash / 20KB RAM 编译。

本工程输出 1Hz PPS 和 9600/8N1 `$GPRMC`，供 VLP-16 建立统一的模拟时基。默认第一次 PPS 的时间是 **2026-01-01 00:00:00 UTC**；每次复位重新开始。在 `include/gps_config.h` 修改 `GPS_START_*`。它没有接收 GNSS、NTP、电脑时间或 IMU 时钟，也不保存断电时间，不能直接宣称具有真实 UTC 精度或已完成雷达与独立 IMU 的时间同步。

## 接线与电气条件

| STM32 / 元件 | 连接位置 | 功能 |
| --- | --- | --- |
| PA9 / USART1_TX | 74HC04 第一反相器输入 1A（你的 A1） | 普通 UART，空闲高 |
| 第一反相器输出 1Y（你的 B1） | VLP-16 接线柱 **GPS RECEIVE** | 反相串口，空闲低；不是 PPS |
| PA8 / TIM1_CH1 | VLP-16 接线柱 **GPS PULSE** | PPS，直接连接，正脉冲，不反相 |
| STM32 GND | 74HC04 GND + VLP-16 **GROUND** | 信号共地 |
| 3.3V | 74HC04 VCC | 按当前接线供电 |

标准 14 脚 74HC04 的第一路一般是 1 脚 1A、2 脚 1Y，14 脚 VCC、7 脚 GND；模块上的 A1/B1 名称请以实际原理图为准。VCC/GND 附近加 100nF 去耦；未使用的反相器**输入**固定到 GND 或 VCC，未使用的输出悬空。

依据 VLP-16 Rev-F §7.4，接收端高电平须 >3.0V 且 <5.0V，低电平 <1.2V，高电平驱动能力至少 2mA。**3.3V 供电的 74HC04 负载后不一定还能保证 >3.0V**，PA8 也需要同样检查。必须在雷达接入后、接线柱处实测。若不合格，应调整缓冲/电平转换硬件；这不是波特率或程序能修复的问题。不能仅把 HC04 改为 5V 供电就假定 3.3V 输入满足其 VIH，应按具体器件数据表设计。

PA10 未启用，PA13/PA14 保留 SWD。PA9 同时具有 TIM1_CH2 复用功能，但本程序仅启用内部 CH2 比较中断，**没有开启 CH2 引脚输出**。

## 实现与时序

采用“单一硬件计时基准 + PWM PPS + 同一定时器比较触发报文 + DMA”的方式。PPS 两个边沿由 TIM1 硬件驱动，不用 `HAL_Delay()`、软件翻转或串口发送耗时拼接一秒。

以每个 PPS 上升沿为 `t=0`：

| 相对时间 | 输出/动作 |
| --- | --- |
| 0ms | PA8 上升，锁存本秒模拟时间 |
| 100ms | PA8 下降 |
| 200ms | TIM1 CH2 通知主循环，格式化本秒 GPRMC，启动 USART1 DMA |
| 约 272ms | 正常报文 69 字节，发送耗时 71.875ms，完成输出 |
| 1000ms | 下一个 PPS 上升 |

TIM1 计数频率 10kHz、ARR=9999，PWM2 的 CCR1=9000，因此从启动计数到第一次 PPS 有 900ms 的静默期，第一条 NMEA 再延后 200ms。CCR2=1000 指的是计数器回绕后的第 100ms，即相对前一个 PPS 的 +200ms；初次启动时的 CH2 事件会被忽略。100μs 是定时器计数分辨率，不是本工程承诺的授时精度。

正常报文关联**它之前最近的 PPS**，时间是该上升沿锁存的整数秒，不把 UART 发送延迟加进时间字段。默认 `GPS_RMC_SECOND_OFFSET=0`。只有通过雷达实际数据确认特定固件的秒关联不同，才可使用配置支持的 `+1`；不能为掩盖延迟或未锁定而随意调整。

发送窗口限定为 PPS 后 200～300ms。即使按 NMEA 最大 82 字节计算，9600/8N1 线时也小于 86ms，报文结束在 PPS 后 386ms 以内。既避开脉冲，也保留超过 300ms 的下个 PPS 前处理时间；也兼顾早期手册的 PPS 后 500ms 内发送完要求。迟到的工作直接丢弃，不积压补发。

报文包含 `HHMMSS.000`、`DDMMYY`、XOR 校验和、大写十六进制及 `\r\n`。DMA 不发送字符串末尾的 NUL。日期处理覆盖 2000～2099 年的闰年、月末和跨年；超出可表达年代停止输出。默认有效标志 `A` 和 0N/0E 坐标均为**模拟值**，不要用于定位。需要模拟失锁可将 `GPS_RMC_STATUS` 改成 `'V'`。

DMA 使用 USART1_TX 对应的 DMA1_Channel4，DMA 和 USART1 中断均已实现。USART TC 中断负责最终释放 HAL UART 状态；缺少它会出现“只发第一条”。发送缓冲区静态保存，DMA 结束前不修改。UART/DMA 故障或运行期间 HSE 失效时关闭 PPS，PA9 回到高电平空闲，记录故障。

## PlatformIO 与 CubeMX 时钟差异

参考了相邻 `imu_for_slam/imu_for_slam/src/board.c` 的 HSE/HSI 方案。本机 `ststm32@20.0.0`、`framework-stm32cubef1@1.8.7` 的 CMSIS `SystemInit()` 不替应用配置 72MHz，初始 `SystemCoreClock` 为 8MHz。CubeMX 则一般由生成的 `main()` 调用生成的 `SystemClock_Config()`，实际频率取决于 `.ioc`。**不要把某个工具链一概当成默认 72MHz。**

本工程的启动顺序：

1. CMSIS 启动文件调用一次 `SystemInit()`，初始化 C 运行环境。
2. `main()` 更新实际 `SystemCoreClock`，`HAL_Init()` 建立 SysTick；`SysTick_Handler()` 调用 `HAL_IncTick()`，使 RCC 等待有真实超时。
3. 显式 `HAL_RCC_DeInit()` 回到 HSI，再配置 PLL，避免对正在作为 SYSCLK 的 PLL 直接改源/倍频。
4. 优先 **HSE 8MHz ×9 = 72MHz**；HSE 起振超时（100ms）后清理 RCC，回退 **HSI/2 ×16 = 64MHz**。
5. AHB=/1、APB1=/2、APB2=/1、Flash 等待周期=2。HAL 切换后刷新 SysTick；TIM1 和 UART 使用实际外设时钟。

HSE 模式 TIM1 PSC=7199，HSI 模式 PSC=6399。PPS 参数保持一致。`board_build.f_cpu` 是构建元数据，**本身不配置硬件 RCC**。`custom_system_setup=no` 保留 PlatformIO 的 CMSIS 启动文件，不能为了添加 `SystemClock_Config()` 就改成 `yes` 而漏掉启动文件。

HSI 回退用于启动诊断和连通性测试，RC 漂移可能影响雷达锁定和长期精度，不能把它当高精度授时源。即使使用 HSE，无外部参考的晶振也会漂移；例如 20ppm 约为 1.728 秒/天，实际误差需测量。要完成与 IMU/主机的精确同步，需要它们共享时基或另行建立时钟映射。

## 编译、烧录和验证

在工程目录执行：

```bash
pio run -e genericSTM32F103C8
pio run -e genericSTM32F103C8 -t upload
```

默认通过 ST-Link 烧录，BOOT0=0。产物为 `.pio/build/genericSTM32F103C8/firmware.bin` 和 `firmware.elf`。本次只进行了构建与主机验证，未对硬件执行烧录。

如需排查晶振问题，可构建并烧录强制 HSI 版本：

```bash
pio run -e genericSTM32F103C8_hsi
pio run -e genericSTM32F103C8_hsi -t upload
```

主机逻辑回归检查（不需要开发板）：

```bash
python3 tests/run_checks.py
```

测试覆盖 100 年全部 36525 个日期进位、一整天逐秒进位、跨年报文、无效日期、校验和/CRLF/NUL、缓冲区边界、发送窗口以及两种构建的中断向量绑定。它们不能替代实际电压、晶振误差或雷达固件行为的硬件测试。

硬件验收按下列顺序进行：

1. 示波器量 PA8：启动计数后约 900ms 首个上升沿，后续周期约 1s，高电平 100ms，低电平 900ms。同步测雷达 GPS PULSE 柱端的电平。HSE 起振失败时，启动计数之前还会有约 100ms 的等待。
2. 量 PA9：空闲高、9600/8N1；量 74HC04 输出：空闲低。逻辑分析仪在 PA9 使用普通 UART 解码，在 74HC04 输出处开启串口反相解码。每秒只能有一条完整 `$GPRMC...*XX\r\n`。
3. 同时量两路：NMEA 应在 PPS 上升后约 200ms 开始，与 PPS 下降相隔约 100ms，报文结束距下一次上升大于 300ms。
4. 打开 VLP-16 网页，等待 PPS 状态变为 `Locked`；用 Wireshark 检查 UDP 8308 的 Position Packet 包含本程序 GPRMC，UDP 2368 的时间戳按当前小时内微秒递增。通过跨秒/跨分钟数据确认秒关联。只有 PPS 锁定、没有可见 NMEA 时，优先检查反相、电平、公共地及 USART 中断。
5. 在调试器观察 `g_gps`：`stage=GPS_STAGE_RUNNING`，`sysclk_hz=72000000`、`hse_in_use=1`，`pps_count` 和 `nmea_sent` 每秒递增；`nmea_dropped`、`uart_errors` 应为 0。HSI 回退为 64000000/0。PC13 正常每 500ms 翻转，HSI 每 125ms 翻转。错误状态快速闪烁，不同板卡的 LED 极性可能不同。

断点时冻结 TIM1，因此调试暂停会破坏外部连续授时；正式锁定测试请脱离断点运行。

故障码记录于 `g_gps.fault_code`：1/2 时钟切换失败，3 频率不符，4 初始日期无效，5 DMA 初始化，6 UART 初始化，7 定时器分频，8 年代超范围，9 UART/DMA 错误，10 前次发送仍忙，11 报文格式/长度，12 启动发送失败，0x10～0x13 内核故障，0x20 运行中 HSE 丢失，0x21 其他 NMI。故障状态保持 GPIO 安全电平并循环闪灯，便于调试器定位。

## 依据与适用范围

- [Velodyne/Ouster VLP-16 User Manual Rev-F](https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf)，§7.4、故障排查与时间同步附录：端子、电气条件、PPS/NMEA 时序和反相串口规则。
- [早期 VLP-16 厂商手册副本](https://docs.clearpathrobotics.com/assets/files/clearpath_robotics_011703-TDS1-e0837d8436bb21caf9d84acc8237d4ea.pdf)：兼顾旧版接口约束。
- [Faizullin 等，Open-Source LiDAR Time Synchronization System by Mimicking GNSS-clock](https://arxiv.org/abs/2107.02625) 及[作者实测系统源码](https://github.com/MobileRoboticsSkoltech/lidar-sync-mimics-gps)：参考 MCU 模拟 GNSS 时基、硬件计时与串口反相架构，本实现独立编写，未照抄该项目代码。
- [ST RM0008](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)：RCC、APB 定时器倍频、TIM1 PWM/比较、DMA/USART。
- [PlatformIO STM32Cube 配置](https://docs.platformio.org/en/stable/frameworks/stm32cube.html)及 [`board_build.f_cpu`](https://docs.platformio.org/en/latest/projectconf/sections/env/options/platform/board_build.f_cpu.html)：启动文件与时钟元数据。
- [TI SN74HC04 数据表](https://www.ti.com/lit/ds/symlink/sn74hc04.pdf)：引脚、负载后的 VOH、去耦和未使用输入处理。

这里采用厂商接口规范和已有硬件实测研究方法；**这份新固件尚未经过你的电路、雷达固件及工业现场的长期验证，不声称获得工业认证或复现论文精度**。是否锁定、实际抖动、长期漂移和秒关联，以硬件验收结果为准。
