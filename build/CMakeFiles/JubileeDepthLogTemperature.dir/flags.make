# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# compile ASM with /usr/bin/arm-none-eabi-gcc
# compile C with /usr/bin/arm-none-eabi-gcc
# compile CXX with /usr/bin/arm-none-eabi-g++
ASM_DEFINES = -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DCYW43_LWIP=1 -DLIB_PICO_ASYNC_CONTEXT_POLL=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_CYW43_ARCH=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_USB=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DMAJOR_VERSION=0 -DMINOR_VERSION=1 -DPICO_BOARD=\"pico_w\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_CYW43_ARCH_POLL=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1 -DPICO_TARGET_NAME=\"JubileeDepthLogTemperature\" -DPICO_USE_BLOCKED_RAM=0 -DPROJECT=JubileeDepthLogTemperature

ASM_INCLUDES = -I/home/sam/pico/projects/JubileeDepthLogTemperature/include -I/usr/share/pico-sdk/src/common/pico_stdlib/include -I/usr/share/pico-sdk/src/rp2_common/hardware_gpio/include -I/usr/share/pico-sdk/src/common/pico_base/include -I/home/sam/pico/projects/JubileeDepthLogTemperature/build/generated/pico_base -I/usr/share/pico-sdk/src/boards/include -I/usr/share/pico-sdk/src/rp2_common/pico_platform/include -I/usr/share/pico-sdk/src/rp2040/hardware_regs/include -I/usr/share/pico-sdk/src/rp2_common/hardware_base/include -I/usr/share/pico-sdk/src/rp2040/hardware_structs/include -I/usr/share/pico-sdk/src/rp2_common/hardware_claim/include -I/usr/share/pico-sdk/src/rp2_common/hardware_sync/include -I/usr/share/pico-sdk/src/rp2_common/hardware_irq/include -I/usr/share/pico-sdk/src/common/pico_sync/include -I/usr/share/pico-sdk/src/common/pico_time/include -I/usr/share/pico-sdk/src/rp2_common/hardware_timer/include -I/usr/share/pico-sdk/src/common/pico_util/include -I/usr/share/pico-sdk/src/rp2_common/hardware_uart/include -I/usr/share/pico-sdk/src/rp2_common/hardware_resets/include -I/usr/share/pico-sdk/src/rp2_common/hardware_clocks/include -I/usr/share/pico-sdk/src/rp2_common/hardware_pll/include -I/usr/share/pico-sdk/src/rp2_common/hardware_vreg/include -I/usr/share/pico-sdk/src/rp2_common/hardware_watchdog/include -I/usr/share/pico-sdk/src/rp2_common/hardware_xosc/include -I/usr/share/pico-sdk/src/rp2_common/hardware_divider/include -I/usr/share/pico-sdk/src/rp2_common/pico_runtime/include -I/usr/share/pico-sdk/src/rp2_common/pico_printf/include -I/usr/share/pico-sdk/src/common/pico_bit_ops/include -I/usr/share/pico-sdk/src/common/pico_divider/include -I/usr/share/pico-sdk/src/rp2_common/pico_double/include -I/usr/share/pico-sdk/src/rp2_common/pico_float/include -I/usr/share/pico-sdk/src/rp2_common/pico_malloc/include -I/usr/share/pico-sdk/src/rp2_common/pico_bootrom/include -I/usr/share/pico-sdk/src/common/pico_binary_info/include -I/usr/share/pico-sdk/src/rp2_common/pico_stdio/include -I/usr/share/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/usr/share/pico-sdk/src/rp2_common/pico_unique_id/include -I/usr/share/pico-sdk/src/rp2_common/hardware_flash/include -I/usr/share/pico-sdk/src/common/pico_usb_reset_interface/include -I/usr/share/pico-sdk/src/rp2_common/pico_int64_ops/include -I/usr/share/pico-sdk/src/rp2_common/pico_mem_ops/include -I/usr/share/pico-sdk/src/rp2_common/boot_stage2/include -I/usr/share/pico-sdk/lib/tinyusb/src -I/usr/share/pico-sdk/lib/tinyusb/src/common -I/usr/share/pico-sdk/lib/tinyusb/hw -I/usr/share/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/usr/share/pico-sdk/src/rp2_common/pico_async_context/include -I/usr/share/pico-sdk/src/rp2_common/pico_lwip/include -I/usr/share/pico-sdk/lib/lwip/src/include -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_arch/include -I/usr/share/pico-sdk/lib/cyw43-driver/src -I/usr/share/pico-sdk/lib/cyw43-driver/firmware -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_driver/cybt_shared_bus -I/usr/share/pico-sdk/src/rp2_common/hardware_pio/include -I/usr/share/pico-sdk/src/rp2_common/hardware_dma/include -I/usr/share/pico-sdk/src/rp2_common/hardware_exception/include -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_driver/include -I/usr/share/pico-sdk/src/rp2_common/pico_rand/include -I/home/sam/pico/projects/JubileeDepthLogTemperature/build/pico-sdk/src/rp2_common/pico_cyw43_driver -I/home/sam/pico/projects/JubileeDepthLogTemperature/external/json -I/home/sam/pico/projects/JubileeDepthLogTemperature/external/pico-onewire/api

ASM_FLAGS = -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections

C_DEFINES = -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DCYW43_LWIP=1 -DLIB_PICO_ASYNC_CONTEXT_POLL=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_CYW43_ARCH=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_USB=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DMAJOR_VERSION=0 -DMINOR_VERSION=1 -DPICO_BOARD=\"pico_w\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_CYW43_ARCH_POLL=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1 -DPICO_TARGET_NAME=\"JubileeDepthLogTemperature\" -DPICO_USE_BLOCKED_RAM=0 -DPROJECT=JubileeDepthLogTemperature

C_INCLUDES = -I/home/sam/pico/projects/JubileeDepthLogTemperature/include -I/usr/share/pico-sdk/src/common/pico_stdlib/include -I/usr/share/pico-sdk/src/rp2_common/hardware_gpio/include -I/usr/share/pico-sdk/src/common/pico_base/include -I/home/sam/pico/projects/JubileeDepthLogTemperature/build/generated/pico_base -I/usr/share/pico-sdk/src/boards/include -I/usr/share/pico-sdk/src/rp2_common/pico_platform/include -I/usr/share/pico-sdk/src/rp2040/hardware_regs/include -I/usr/share/pico-sdk/src/rp2_common/hardware_base/include -I/usr/share/pico-sdk/src/rp2040/hardware_structs/include -I/usr/share/pico-sdk/src/rp2_common/hardware_claim/include -I/usr/share/pico-sdk/src/rp2_common/hardware_sync/include -I/usr/share/pico-sdk/src/rp2_common/hardware_irq/include -I/usr/share/pico-sdk/src/common/pico_sync/include -I/usr/share/pico-sdk/src/common/pico_time/include -I/usr/share/pico-sdk/src/rp2_common/hardware_timer/include -I/usr/share/pico-sdk/src/common/pico_util/include -I/usr/share/pico-sdk/src/rp2_common/hardware_uart/include -I/usr/share/pico-sdk/src/rp2_common/hardware_resets/include -I/usr/share/pico-sdk/src/rp2_common/hardware_clocks/include -I/usr/share/pico-sdk/src/rp2_common/hardware_pll/include -I/usr/share/pico-sdk/src/rp2_common/hardware_vreg/include -I/usr/share/pico-sdk/src/rp2_common/hardware_watchdog/include -I/usr/share/pico-sdk/src/rp2_common/hardware_xosc/include -I/usr/share/pico-sdk/src/rp2_common/hardware_divider/include -I/usr/share/pico-sdk/src/rp2_common/pico_runtime/include -I/usr/share/pico-sdk/src/rp2_common/pico_printf/include -I/usr/share/pico-sdk/src/common/pico_bit_ops/include -I/usr/share/pico-sdk/src/common/pico_divider/include -I/usr/share/pico-sdk/src/rp2_common/pico_double/include -I/usr/share/pico-sdk/src/rp2_common/pico_float/include -I/usr/share/pico-sdk/src/rp2_common/pico_malloc/include -I/usr/share/pico-sdk/src/rp2_common/pico_bootrom/include -I/usr/share/pico-sdk/src/common/pico_binary_info/include -I/usr/share/pico-sdk/src/rp2_common/pico_stdio/include -I/usr/share/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/usr/share/pico-sdk/src/rp2_common/pico_unique_id/include -I/usr/share/pico-sdk/src/rp2_common/hardware_flash/include -I/usr/share/pico-sdk/src/common/pico_usb_reset_interface/include -I/usr/share/pico-sdk/src/rp2_common/pico_int64_ops/include -I/usr/share/pico-sdk/src/rp2_common/pico_mem_ops/include -I/usr/share/pico-sdk/src/rp2_common/boot_stage2/include -I/usr/share/pico-sdk/lib/tinyusb/src -I/usr/share/pico-sdk/lib/tinyusb/src/common -I/usr/share/pico-sdk/lib/tinyusb/hw -I/usr/share/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/usr/share/pico-sdk/src/rp2_common/pico_async_context/include -I/usr/share/pico-sdk/src/rp2_common/pico_lwip/include -I/usr/share/pico-sdk/lib/lwip/src/include -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_arch/include -I/usr/share/pico-sdk/lib/cyw43-driver/src -I/usr/share/pico-sdk/lib/cyw43-driver/firmware -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_driver/cybt_shared_bus -I/usr/share/pico-sdk/src/rp2_common/hardware_pio/include -I/usr/share/pico-sdk/src/rp2_common/hardware_dma/include -I/usr/share/pico-sdk/src/rp2_common/hardware_exception/include -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_driver/include -I/usr/share/pico-sdk/src/rp2_common/pico_rand/include -I/home/sam/pico/projects/JubileeDepthLogTemperature/build/pico-sdk/src/rp2_common/pico_cyw43_driver -I/home/sam/pico/projects/JubileeDepthLogTemperature/external/json -I/home/sam/pico/projects/JubileeDepthLogTemperature/external/pico-onewire/api

C_FLAGS = -mcpu=cortex-m0plus -mthumb -Og -g -std=gnu11 -ffunction-sections -fdata-sections

CXX_DEFINES = -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DCYW43_LWIP=1 -DLIB_PICO_ASYNC_CONTEXT_POLL=1 -DLIB_PICO_BIT_OPS=1 -DLIB_PICO_BIT_OPS_PICO=1 -DLIB_PICO_CYW43_ARCH=1 -DLIB_PICO_DIVIDER=1 -DLIB_PICO_DIVIDER_HARDWARE=1 -DLIB_PICO_DOUBLE=1 -DLIB_PICO_DOUBLE_PICO=1 -DLIB_PICO_FIX_RP2040_USB_DEVICE_ENUMERATION=1 -DLIB_PICO_FLOAT=1 -DLIB_PICO_FLOAT_PICO=1 -DLIB_PICO_INT64_OPS=1 -DLIB_PICO_INT64_OPS_PICO=1 -DLIB_PICO_MALLOC=1 -DLIB_PICO_MEM_OPS=1 -DLIB_PICO_MEM_OPS_PICO=1 -DLIB_PICO_PLATFORM=1 -DLIB_PICO_PRINTF=1 -DLIB_PICO_PRINTF_PICO=1 -DLIB_PICO_RAND=1 -DLIB_PICO_RUNTIME=1 -DLIB_PICO_STANDARD_LINK=1 -DLIB_PICO_STDIO=1 -DLIB_PICO_STDIO_USB=1 -DLIB_PICO_STDLIB=1 -DLIB_PICO_SYNC=1 -DLIB_PICO_SYNC_CRITICAL_SECTION=1 -DLIB_PICO_SYNC_MUTEX=1 -DLIB_PICO_SYNC_SEM=1 -DLIB_PICO_TIME=1 -DLIB_PICO_UNIQUE_ID=1 -DLIB_PICO_UTIL=1 -DMAJOR_VERSION=0 -DMINOR_VERSION=1 -DPICO_BOARD=\"pico_w\" -DPICO_BUILD=1 -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_CYW43_ARCH_POLL=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_RP2040_USB_DEVICE_UFRAME_FIX=1 -DPICO_TARGET_NAME=\"JubileeDepthLogTemperature\" -DPICO_USE_BLOCKED_RAM=0 -DPROJECT=JubileeDepthLogTemperature

CXX_INCLUDES = -I/home/sam/pico/projects/JubileeDepthLogTemperature/include -I/usr/share/pico-sdk/src/common/pico_stdlib/include -I/usr/share/pico-sdk/src/rp2_common/hardware_gpio/include -I/usr/share/pico-sdk/src/common/pico_base/include -I/home/sam/pico/projects/JubileeDepthLogTemperature/build/generated/pico_base -I/usr/share/pico-sdk/src/boards/include -I/usr/share/pico-sdk/src/rp2_common/pico_platform/include -I/usr/share/pico-sdk/src/rp2040/hardware_regs/include -I/usr/share/pico-sdk/src/rp2_common/hardware_base/include -I/usr/share/pico-sdk/src/rp2040/hardware_structs/include -I/usr/share/pico-sdk/src/rp2_common/hardware_claim/include -I/usr/share/pico-sdk/src/rp2_common/hardware_sync/include -I/usr/share/pico-sdk/src/rp2_common/hardware_irq/include -I/usr/share/pico-sdk/src/common/pico_sync/include -I/usr/share/pico-sdk/src/common/pico_time/include -I/usr/share/pico-sdk/src/rp2_common/hardware_timer/include -I/usr/share/pico-sdk/src/common/pico_util/include -I/usr/share/pico-sdk/src/rp2_common/hardware_uart/include -I/usr/share/pico-sdk/src/rp2_common/hardware_resets/include -I/usr/share/pico-sdk/src/rp2_common/hardware_clocks/include -I/usr/share/pico-sdk/src/rp2_common/hardware_pll/include -I/usr/share/pico-sdk/src/rp2_common/hardware_vreg/include -I/usr/share/pico-sdk/src/rp2_common/hardware_watchdog/include -I/usr/share/pico-sdk/src/rp2_common/hardware_xosc/include -I/usr/share/pico-sdk/src/rp2_common/hardware_divider/include -I/usr/share/pico-sdk/src/rp2_common/pico_runtime/include -I/usr/share/pico-sdk/src/rp2_common/pico_printf/include -I/usr/share/pico-sdk/src/common/pico_bit_ops/include -I/usr/share/pico-sdk/src/common/pico_divider/include -I/usr/share/pico-sdk/src/rp2_common/pico_double/include -I/usr/share/pico-sdk/src/rp2_common/pico_float/include -I/usr/share/pico-sdk/src/rp2_common/pico_malloc/include -I/usr/share/pico-sdk/src/rp2_common/pico_bootrom/include -I/usr/share/pico-sdk/src/common/pico_binary_info/include -I/usr/share/pico-sdk/src/rp2_common/pico_stdio/include -I/usr/share/pico-sdk/src/rp2_common/pico_stdio_usb/include -I/usr/share/pico-sdk/src/rp2_common/pico_unique_id/include -I/usr/share/pico-sdk/src/rp2_common/hardware_flash/include -I/usr/share/pico-sdk/src/common/pico_usb_reset_interface/include -I/usr/share/pico-sdk/src/rp2_common/pico_int64_ops/include -I/usr/share/pico-sdk/src/rp2_common/pico_mem_ops/include -I/usr/share/pico-sdk/src/rp2_common/boot_stage2/include -I/usr/share/pico-sdk/lib/tinyusb/src -I/usr/share/pico-sdk/lib/tinyusb/src/common -I/usr/share/pico-sdk/lib/tinyusb/hw -I/usr/share/pico-sdk/src/rp2_common/pico_fix/rp2040_usb_device_enumeration/include -I/usr/share/pico-sdk/src/rp2_common/pico_async_context/include -I/usr/share/pico-sdk/src/rp2_common/pico_lwip/include -I/usr/share/pico-sdk/lib/lwip/src/include -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_arch/include -I/usr/share/pico-sdk/lib/cyw43-driver/src -I/usr/share/pico-sdk/lib/cyw43-driver/firmware -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_driver/cybt_shared_bus -I/usr/share/pico-sdk/src/rp2_common/hardware_pio/include -I/usr/share/pico-sdk/src/rp2_common/hardware_dma/include -I/usr/share/pico-sdk/src/rp2_common/hardware_exception/include -I/usr/share/pico-sdk/src/rp2_common/pico_cyw43_driver/include -I/usr/share/pico-sdk/src/rp2_common/pico_rand/include -I/home/sam/pico/projects/JubileeDepthLogTemperature/build/pico-sdk/src/rp2_common/pico_cyw43_driver -I/home/sam/pico/projects/JubileeDepthLogTemperature/external/json -I/home/sam/pico/projects/JubileeDepthLogTemperature/external/pico-onewire/api

CXX_FLAGS = -mcpu=cortex-m0plus -mthumb -Og -g -std=gnu++17 -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit

