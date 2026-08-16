cmake_minimum_required(VERSION 3.16)
project(alpine_ar_overlay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(GDAL REQUIRED)

add_executable(calibrate src/main.cpp)
target_include_directories(calibrate PRIVATE include)
target_link_libraries(calibrate PRIVATE ${OpenCV_LIBS})

add_executable(dem_info src/dem_info.cpp)
target_include_directories(dem_info PRIVATE include)
target_link_libraries(dem_info PRIVATE GDAL::GDAL)
