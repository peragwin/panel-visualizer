idf_component_register(
    SRCS
    "main.cpp"
    "../lib/Render/hsluv.cpp"
    "../lib/Render/Render.cpp"
    "../lib/Audio/fft.cpp"
    "../lib/Configlets/Configlets.cpp"

    # "../lib/Audio/bucketer.cpp"
    # "../lib/Audio/FrequencySensor.cpp"
    INCLUDE_DIRS
    "."
    "../lib/Render"
    "../lib/Audio"
    "../lib/Configlets"

    REQUIRES
    arduino
    ArduinoJson
    ParticleLife
    ESP32-HUB75-MatrixPanel-I2S-DMA

    # app_update
)
