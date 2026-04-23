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
    src/maps/MapReader.cpp
    src/maps/CompressorMap.cpp
    src/maps/TurbineMap.cpp
    src/maps/NozzleMap.cpp
    src/maps/InletMap.cpp
    src/maps/CombustorMap.cpp
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
# Unit tests
# =============================================================================

# test_thermo — validates NASA 7-coefficient polynomial functions in isolation
add_executable(test_thermo
    tests/test_thermo.cpp
    src/thermodynamics/Thermo.cpp
)
target_include_directories(test_thermo PRIVATE include)
target_compile_options(test_thermo PRIVATE
    -Wall       # Enable all standard warnings
    -Wextra     # Enable extra warnings
    -Wpedantic  # Enforce strict ISO C++ compliance
)

# test_maps — validates all map classes against synthetic map files
add_executable(test_maps
    tests/test_maps.cpp
    src/maps/MapReader.cpp
    src/maps/CompressorMap.cpp
    src/maps/TurbineMap.cpp
    src/maps/NozzleMap.cpp
    src/maps/InletMap.cpp
    src/maps/CombustorMap.cpp
    src/thermodynamics/Thermo.cpp
)
target_include_directories(test_maps PRIVATE include)
target_compile_options(test_maps PRIVATE
    -Wall       # Enable all standard warnings
    -Wextra     # Enable extra warnings
    -Wpedantic  # Enforce strict ISO C++ compliance
)

# =============================================================================
# Tools
# =============================================================================

# generate_maps — generates synthetic performance map files
add_executable(generate_maps
    tools/generate_maps.cpp
)
target_compile_options(generate_maps PRIVATE
    -Wall       # Enable all standard warnings
    -Wextra     # Enable extra warnings
    -Wpedantic  # Enforce strict ISO C++ compliance
)

# data_reduction — Phase 6 data reduction model
# Runs turbofan cycle at multiple power settings, computes Kurzke-Riegler
# scale factors per component, writes updated scale factors to map files.
# Replace synthetic operating points with real test cell data when available.
add_executable(data_reduction
    tools/data_reduction.cpp
    src/thermodynamics/ISA.cpp
    src/thermodynamics/Thermo.cpp
    src/maps/MapReader.cpp
    src/maps/CompressorMap.cpp
    src/maps/TurbineMap.cpp
    src/maps/NozzleMap.cpp
    src/maps/InletMap.cpp
    src/maps/CombustorMap.cpp
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
)
target_include_directories(data_reduction PRIVATE include)
target_compile_options(data_reduction PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)