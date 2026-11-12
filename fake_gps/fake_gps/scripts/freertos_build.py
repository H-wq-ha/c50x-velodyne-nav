"""Build the FreeRTOS kernel already bundled with the STM32CubeF1 framework."""
from pathlib import Path

Import("env")

framework = Path(env.PioPlatform().get_package_dir("framework-stm32cubef1"))
kernel = framework / "Middlewares" / "Third_Party" / "FreeRTOS" / "Source"
port = kernel / "portable" / "GCC" / "ARM_CM3"
if not (port / "port.c").is_file():
    raise RuntimeError("STM32CubeF1 FreeRTOS ARM_CM3 port is missing")

env.Append(CPPPATH=[str(kernel / "include"), str(port)])
env.BuildSources(
    "$BUILD_DIR/freertos",
    str(kernel),
    src_filter=[
        "-<*>", "+<tasks.c>", "+<queue.c>", "+<list.c>",
        "+<portable/GCC/ARM_CM3/port.c>",
    ],
)
