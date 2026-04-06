cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter -u _printf_float")


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME carte_lidar)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)



# canopen src
set(CANOPEN_STACK_SRC
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_ODinterface.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_NMT_Heartbeat.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_HBconsumer.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_Node_Guarding.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_Emergency.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_SDOserver.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_SDOclient.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_SYNC.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_PDO.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/301/CO_TIME.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/303/CO_LEDs.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/304/CO_GFC.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/304/CO_SRDO.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/305/CO_LSSslave.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/305/CO_LSSmaster.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/309/CO_gateway_ascii.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/extra/CO_trace.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/CANopen.c

	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/CO_app_STM32.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/CO_driver_STM32.c
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/CO_storageBlank.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/OD.c
)

# canopen inc
set(CANOPEN_STACK_INC
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/301/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/302/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/303/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/304/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/305/
	${CMAKE_CURRENT_SOURCE_DIR}/CANopenNode_STM32/309/
)


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/lidar_reader.c
    ${CMAKE_CURRENT_SOURCE_DIR}/User/Src/lipkg_copy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib_robot/src_lib/coords.c
	${CANOPEN_STACK_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths

    ${CMAKE_CURRENT_SOURCE_DIR}/User/Inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/lib_robot/inc_lib/
	${CANOPEN_STACK_INC}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
