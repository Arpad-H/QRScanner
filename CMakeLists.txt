cmake_minimum_required(VERSION 3.29)
project(QRScanner)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -static-libgcc -static")

include_directories("lib/libBV/include")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
if (TARGET Eigen3::Eigen)
    # Use the imported target
endif (TARGET Eigen3::Eigen)

add_executable(QRScanner main.cpp
        Vector.cpp
        Vector.h
        Entzerren.cpp
        Entzerren.h
        Matrix.cpp
        Matrix.h
)
add_library(libBV STATIC lib/libBV/lib/Grafik.cxx
        lib/libBV/lib/SimpleFont.cxx
        lib/libBV/lib/AviRead.cxx
        lib/libBV/lib/AviWrite.cxx
        lib/libBV/lib/BmpRead.cxx
        lib/libBV/lib/BmpWrite.cxx
        lib/libBV/lib/FFT.cxx
        lib/libBV/lib/FileIO.cxx
        lib/libBV/lib/Grafik.cxx
)
target_link_libraries(QRScanner libBV)
target_link_libraries(QRScanner Eigen3::Eigen)

