set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.21)

#*******************************************************************************************#
# 设置编译器
set(CMAKE_C_COMPILER armclang.exe)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(C_TARGET_FLAG --target=arm-arm-none-eabi)

set(CMAKE_CXX_COMPILER armclang.exe)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
set(CXX_TARGET_FLAG --target=arm-arm-none-eabi)

set(CMAKE_ASM_COMPILER armclang.exe)
set(CMAKE_ASM_COMPILER_WORKS TRUE)
set(ASM_TARGET_FLAG --target=arm-arm-none-eabi)

#设置链接器
set(CMAKE_C_LINK_EXECUTABLE armlink.exe)
set(CMAKE_ASM_LINK_EXECUTABLE armlink.exe)
set(CMAKE_CXX_LINK_EXECUTABLE armlink.exe)

#*******************************************************************************************#
# 工程设置与添加FineMote文件
project(freertos_microros_dev1 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib) # 对于静态库



#*******************************************************************************************
# 2. 硬件和目标板特定配置
#*******************************************************************************************
set(BOARD_NAME MC_Board)
set(CHIP_NAME STM32F446xx)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

# 定义目标 CPU 和 Target Triple 标志
set(TARGET_CPU_FLAG "-mcpu=${CMAKE_SYSTEM_PROCESSOR}")
set(TARGET_TRIPLE_FLAG "--target=arm-arm-none-eabi")

# 定义 FPU 和其他全局编译选项
add_compile_options(
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
)

# 定义全局宏
add_compile_definitions(
        USE_HAL_DRIVER
        ${CHIP_NAME}
        # [关键修正] 添加缺失的 CMSIS 设备头文件宏定义
#        CMSIS_device_header=${DEVICE_HEADER_FILE}
        CMSIS_device_header="stm32f446xx.h"
        USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1
        ARM_MATH_CM4
        ARM_MATH_MATRIX_CHECK
        ARM_MATH_ROUNDING
        # 下面两个定义对于保持代码可移植性很有用
        __${BOARD_NAME}
        __CMAKE_APP
)

#*******************************************************************************************
# 3. 搜集头文件和源文件
#*******************************************************************************************

# --- A. 头文件路径 ---
include_directories(
        .
        Drivers/CMSIS/Include
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Core/Inc
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
        ThirdParty/CMSIS-DSP/Include
        ThirdParty/CMSIS-DSP/Include/dsp
        ThirdParty/CMSIS-DSP/PrivateInclude
)

# --- B. 源文件搜集 ---
file(GLOB_RECURSE CLEAN_SOURCES
        "Core/Src/*.c"
        "Core/Src/*.cpp"
        "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
        "Middlewares/Third_Party/FreeRTOS/Source/*.c"
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c"
        "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c"
        "MDK-ARM/startup_*.s"
)

# 组 B: 依赖 POSIX 的源文件 (micro-ROS 相关的)
file(GLOB_RECURSE POSIX_CLIENT_SOURCES
        "Core/Src/custom_memory_manager.c"
        "Core/Src/allocators.c"
        "Core/Src/microros_transports.c"
)

# 组 C: POSIX 实现自身的源文件
file(GLOB_RECURSE FREERTOS_POSIX_SRCS
        "lib/FreeRTOS-Plus-POSIX/source/*.c"
)

# 从 CLEAN_SOURCES 中剔除干扰文件
file(GLOB_RECURSE EXCLUDE_SRCS
        "Core/Src/syscalls.c"
        "Core/Src/sysmem.c"
)
list(REMOVE_ITEM CLEAN_SOURCES ${EXCLUDE_SRCS})
list(REMOVE_ITEM CLEAN_SOURCES ${POSIX_CLIENT_SOURCES})

# 移除标准的 FreeRTOS 堆实现，以避免与 micro-ROS 的 custom_memory_manager 冲突
#file(GLOB STANDARD_HEAP_FILES "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_*.c")
#list(REMOVE_ITEM CLEAN_SOURCES ${STANDARD_HEAP_FILES})

#*******************************************************************************************
# 4. 创建构建目标
#*******************************************************************************************
add_library(clean_lib OBJECT ${CLEAN_SOURCES})
# 将 micro-ROS 的头文件路径添加到这个库
target_include_directories(clean_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/micro-ROS/microros_static_library/include
)

add_library(freertos_posix OBJECT ${FREERTOS_POSIX_SRCS})
target_include_directories(freertos_posix PUBLIC
        lib/include
        lib/include/private
        lib/include/FreeRTOS_POSIX
        lib/include/FreeRTOS_POSIX/sys
        lib/FreeRTOS-Plus-POSIX/include
        lib/FreeRTOS-Plus-POSIX/include/portable
        lib/FreeRTOS-Plus-POSIX/include/portable/empty_portable
)

# 【关键修正】使用逻辑名称 ${BOARD_NAME} (即 MC_Board) 作为 Target 名称，而不是带 .elf 后缀
add_executable(${BOARD_NAME}
        ${POSIX_CLIENT_SOURCES}
        $<TARGET_OBJECTS:clean_lib>
        $<TARGET_OBJECTS:freertos_posix>
)

#*******************************************************************************************
# 5. 设置编译和链接选项
#*******************************************************************************************

target_link_libraries(${BOARD_NAME} PRIVATE
        # 链接 micro-ROS 的预编译库
        ${CMAKE_CURRENT_SOURCE_DIR}/micro-ROS/microros_static_library/libmicroros.a
        # 链接 object 库以继承其 PUBLIC 属性 (如 include 路径)
        clean_lib
        freertos_posix
)

#*******************************************************************************************#
# 6. 设置特定于编译和链接的选项 (严格遵循结构A的风格)
#*******************************************************************************************#

# --- 模拟顶层文件(文件2)中定义的变量 ---
set(C_TARGET_FLAG --target=arm-arm-none-eabi)
set(CXX_TARGET_FLAG --target=arm-arm-none-eabi)
set(ASM_TARGET_FLAG --target=arm-arm-none-eabi)

# --- BSP文件(文件1)中的设置 ---
set(COMPILE_RULE_FLAG "-mcpu=${CMAKE_SYSTEM_PROCESSOR}")
set(LINKER_TARGET_FLAG "--cpu=${CMAKE_SYSTEM_PROCESSOR}")

# 为 C/C++ 编译器设置标志 (来自结构A)
set(CMAKE_C_FLAGS "${C_TARGET_FLAG} ${COMPILE_RULE_FLAG} \
    -fno-rtti -funsigned-char -fshort-enums  \
    -gdwarf-4 -Os -ffunction-sections \
    -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -Wno-reserved-identifier \
    ")
set(CMAKE_CXX_FLAGS ${CMAKE_C_FLAGS})

# 为汇编器设置标志 (来自结构A, 假设使用 armclang)
# 注意：这要求 CMAKE_ASM_COMPILER 设置为 armclang.exe
set(CMAKE_ASM_FLAGS "${ASM_TARGET_FLAG} ${COMPILE_RULE_FLAG} -masm=auto -c -gdwarf-4")

# 设置链接器特定选项 (来自结构A, 但改进了路径)
# 我们保留了您代码中更稳健的路径设置方式
set(SECTIONS_SCRIPT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/MDK-ARM/${BOARD_NAME}/${BOARD_NAME}.sct)
set(CMAKE_EXE_LINKER_FLAGS " \
    ${LINKER_TARGET_FLAG} \
    --strict \
    --scatter ${SECTIONS_SCRIPT_PATH} \
    --info sizes --info totals --info unused --info veneers \
    --summary_stderr --info summarysizes --map --load_addr_map_info --xref --callgraph --symbols \
    --list ${CMAKE_BINARY_DIR}/${BOARD_NAME}.map \
    ")


#*******************************************************************************************

# 7. 后续处理步骤 (保持不变)
#*******************************************************************************************#

set(HEX_FILE ${CMAKE_BINARY_DIR}/${BOARD_NAME}.hex)
set(ELF_FILE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${BOARD_NAME}.elf) # CMake 自动为可执行目标添加 .elf 后缀
set(ARMCC_fromelf fromelf.exe)

add_custom_command(TARGET ${BOARD_NAME} POST_BUILD
        COMMAND ${ARMCC_fromelf} --i32combined --output="${HEX_FILE}" "${ELF_FILE}"
        COMMENT "Generating ${HEX_FILE}"
)