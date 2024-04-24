cmake_minimum_required(VERSION 3.17)

project(dufomap
	VERSION 1.0.0
	DESCRIPTION "DUFOMap: Efficient Dynamic Awareness Mapping"
	LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(ufomap)
include_directories(${CMAKE_SOURCE_DIR}/ufomap)

add_executable(dufomap_run src/dufomap.cpp)
set_target_properties(dufomap_run
	PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)
target_link_libraries(dufomap_run PRIVATE UFO::Map)
target_compile_features(dufomap_run
	PUBLIC
	cxx_std_20
)
target_compile_options(dufomap_run
	PRIVATE
	# -Wall
	# -Werror
	# -Wextra
	# -Wpedantic
	# -Wunreachable-code
	# -Wconversion
	# -Wcast-align
	# -Wunused
	# -Wno-unused-parameter
	# -Wold-style-cast
	# -Wpointer-arith
	# -Wcast-qual
	# -Wno-missing-braces
	# -Wdouble-promotion

	# -Wshadow
	# -march=native
)
