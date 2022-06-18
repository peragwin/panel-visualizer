cmake_minimum_required(VERSION 3.5)

idf_component_register(
    SRCS "ParticleLife/Universe.cpp"
    INCLUDE_DIRS "ParticleLife"
    REQUIRES arduino
)
