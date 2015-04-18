cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

set(CMAKE_TOOLCHAIN_FILE $ENV{ARDUINO_CMAKE}/cmake/ArduinoToolchain.cmake)
set(ARDUINO_1_5 true)

project(fischerbot C CXX)

link_directories(${ARDUINO_SDK_PATH}/libraries)
link_directories(${ARDUINO_SDK_PATH}/hardware/arduino/avr/libraries)
link_directories($ENV{ARDUINO_LIBRARIES})

generate_arduino_firmware(
    fischerbot
    SRCS fischerbot.cpp
    BOARD mega1284-16
    PROGRAMMER dragon
    PORT USB)

