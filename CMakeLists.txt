cmake_minimum_required(VERSION 3.15)
project(GasTurbineEngineModel VERSION 0.1.0 LANGUAGES CXX)

# C++17 required — uses structured bindings, noexcept, and other C++17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header files location
include_directories(include)

# Source files — all engine element implementations
set(SOURCES
    src/thermodynamics/ISA.cpp
    src/elements/Inlet.cpp
    src/elements/Compressor.cpp
    src/elements/Combustor.cpp
    src/elements/Turbine.cpp
    src/elements/Nozzle.cpp
    src/main.cpp
)

# Build target — produces the GasTurbineEngineModel executable
add_executable(GasTurbineEngineModel ${SOURCES})

# Compiler warnings — catch common mistakes early
target_compile_options(GasTurbineEngineModel PRIVATE
    -Wall       # Enable all standard warnings
    -Wextra     # Enable extra warnings
    -Wpedantic  # Enforce strict ISO C++ compliance
)