cmake_minimum_required(VERSION 3.24)
project(htcw_winduino VERSION 1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(FIND_LIBRARY_USE_LIB64_PATHS True)
set( DXLIBS d2d1 )
set(CMAKE_STATIC_LIBRARY_PREFIX "")
set(CMAKE_SHARED_LIBRARY_PREFIX "")

add_library(htcw_winduino 
                src/Arduino.cpp
                src/WString.cpp
                src/stdlib_noniso.c
                src/Print.cpp
                src/Stream.cpp
                src/StreamString.cpp
                src/WMath.cpp
                src/HardwareSerial.cpp
                src/StdioFSImpl.cpp
                src/FS.cpp
                src/SD.cpp
                src/SPI.cpp
                src/Wire.cpp)
target_link_libraries(htcw_winduino ${DXLIBS} )
target_include_directories(htcw_winduino PUBLIC
    "${PROJECT_SOURCE_DIR}/src"    
    "${PROJECT_BINARY_DIR}"
)

