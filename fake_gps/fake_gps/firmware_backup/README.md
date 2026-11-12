# 修改前的 GPS-only 固件备份

这两个 bin 在加入 FreeRTOS/IMU 之前从已构建产物保存，均为 STM32F103C8，起始地址 0x08000000。
- gps_only_hse.bin：HSE 优先、HSI 超时回退。
- gps_only_hsi.bin：强制 HSI 诊断版本。

不包含本次 IMU 功能；当前联合固件在 .pio/build/ 下。历史接线和使用说明见 ../docs/gps_only_reference.md。
