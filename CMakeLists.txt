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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Basic_Framework_MC02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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

file(GLOB_RECURSE USER_SOURCES
 "Middlewares/Third_Party/SEGGER/*.c"
 "bsp/*.c"
 "bsp/*.s"
 "modules/*.c"
 "application/*.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
 ${USER_SOURCES}
)
# Add include paths
function(recurse_find_directory root)
 file(GLOB children RELATIVE ${root} ${root}/*)
 foreach(child ${children})
 if(IS_DIRECTORY ${root}/${child})
 message("Adding include directory: " ${root}/${child})
 target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
${root}/${child})
 recurse_find_directory(${root}/${child})
 endif()
 endforeach()
endfunction(recurse_find_directory)
recurse_find_directory("${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER")
recurse_find_directory("${CMAKE_SOURCE_DIR}/bsp")
recurse_find_directory("${CMAKE_SOURCE_DIR}/modules")
recurse_find_directory("${CMAKE_SOURCE_DIR}/application")
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
 # Add user defined include paths
 "bsp"
 "modules"
 "application"
)

