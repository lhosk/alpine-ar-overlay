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

add_executable(mesh_info src/mesh_info.cpp src/dem.cpp)
target_include_directories(mesh_info PRIVATE include)
target_link_libraries(mesh_info PRIVATE GDAL::GDAL)

add_executable(hillshade src/hillshade.cpp src/dem.cpp)
target_include_directories(hillshade PRIVATE include)
target_link_libraries(hillshade PRIVATE GDAL::GDAL ${OpenCV_LIBS})

add_executable(export_obj src/export_obj.cpp src/dem.cpp)
target_include_directories(export_obj PRIVATE include)
target_link_libraries(export_obj PRIVATE GDAL::GDAL)

add_executable(render_view src/render_view.cpp src/dem.cpp)
target_include_directories(render_view PRIVATE include)
target_link_libraries(render_view PRIVATE GDAL::GDAL ${OpenCV_LIBS})

add_executable(overlay src/overlay.cpp src/dem.cpp)
target_include_directories(overlay PRIVATE include)
target_link_libraries(overlay PRIVATE GDAL::GDAL ${OpenCV_LIBS})
