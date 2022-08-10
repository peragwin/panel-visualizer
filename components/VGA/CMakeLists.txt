cmake_minimum_required(VERSION 3.5)

idf_component_register(
    SRCS
    "ESP32Lib/src/VGA/VGA.cpp"
    "ESP32Lib/src/I2S/I2S.cpp"

    INCLUDE_DIRS
    "ESP32Lib/src"
    "ESP32Lib/src/VGA"
    "ESP32Lib/src/I2S"

    REQUIRES arduino
)
