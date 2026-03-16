cmake_minimum_required(VERSION 3.15)
project(GasTurbineEngineModel VERSION 0.1.0 LANGUAGES CXX)

# C++17 required — range-based for, structured bindings, and guaranteed copy elision
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   # Disable GNU extensions — enforce strict ISO C++

# Source files — all engine component implementations
set(SOURCES
    src/thermodynamics/ISA.cpp
    src/thermodynamics/Thermo.cpp
    src/elements/Inlet.cpp
    src/elements/Fan.cpp
    src/elements/Splitter.cpp
    src/elements/Compressor.cpp
    src/elements/Combustor.cpp
    src/elements/Turbine.cpp
    src/elements/Nozzle.cpp
    src/elements/FanNozzle.cpp
    src/elements/Afterburner.cpp
    src/mechanics/Shaft.cpp
    src/solver/Solver.cpp
    src/main.cpp
)

# Build target — produces the GasTurbineEngineModel executable
add_executable(GasTurbineEngineModel ${SOURCES})

# Target-scoped properties — must come after add_executable
target_include_directories(GasTurbineEngineModel PRIVATE include)

# Compiler warnings — catch common mistakes early
target_compile_options(GasTurbineEngineModel PRIVATE
    -Wall       # Enable all standard warnings
    -Wextra     # Enable extra warnings
    -Wpedantic  # Enforce strict ISO C++ compliance
)

# =============================================================================
# Unit tests — each test is a standalone executable, no engine element deps
# =============================================================================

# test_thermo — validates NASA 7-coefficient polynomial functions
add_executable(test_thermo
    tests/test_thermo.cpp
    src/thermodynamics/Thermo.cpp
)
target_include_directories(test_thermo PRIVATE include)
target_compile_options(test_thermo PRIVATE -Wall -Wextra -Wpedantic)