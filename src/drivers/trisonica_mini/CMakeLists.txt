px4_add_module(
    MODULE drivers__trisonica_mini
    MAIN trisonica_mini
    COMPILE_FLAGS
    SRCS
        trisonica_mini.cpp
        trisonica_mini.h
    DEPENDS
)
