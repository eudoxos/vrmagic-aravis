cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(vrmagic-aravis-cxx)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(ARAVIS REQUIRED aravis-0.6)

find_package(Qt5 COMPONENTS Core Widgets Charts REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

link_directories(
	${GLIB_LIBRARY_DIRS}
	${ARAVIS_LIBRARY_DIRS}
)
include_directories(
	${GLIB_INCLUDE_DIRS}
	${ARAVIS_INCLUDE_DIRS}
)
add_executable(read-trsf-params read-trsf-params.cpp)
target_link_libraries(read-trsf-params PUBLIC
	${GLIB_LIBRARIES}
	${ARAVIS_LIBRARIES}
)

add_executable(live-plot live-plot.cpp)
target_link_libraries(live-plot PUBLIC
	Eigen3::Eigen
	Qt5::Core Qt5::Widgets Qt5::Charts
	${GLIB_LIBRARIES}
	${ARAVIS_LIBRARIES}
)


