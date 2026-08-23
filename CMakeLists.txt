cmake_minimum_required(VERSION 3.16)
project(alpine_ar_overlay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(OpenCV REQUIRED)
find_package(GDAL REQUIRED)
find_package(OpenMP REQUIRED)

add_library(dem src/dem.cpp)
target_include_directories(dem PUBLIC include)
target_link_libraries(dem PUBLIC GDAL::GDAL)

add_executable(mesh_info src/mesh_info.cpp)
target_link_libraries(mesh_info PRIVATE dem)

add_executable(overlay src/overlay.cpp)
target_link_libraries(overlay PRIVATE dem ${OpenCV_LIBS})

add_executable(pick src/pick.cpp)
target_link_libraries(pick PRIVATE ${OpenCV_LIBS})

add_executable(fit_yaw src/fit_yaw.cpp)
target_link_libraries(fit_yaw PRIVATE dem OpenMP::OpenMP_CXX)

add_executable(export_web src/export_web.cpp)
target_link_libraries(export_web PRIVATE dem)
