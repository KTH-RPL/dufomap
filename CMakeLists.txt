cmake_minimum_required(VERSION 3.17)

project(ufomap
	VERSION 2.0.0
	DESCRIPTION "UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown"
	LANGUAGES CXX
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
		STRING "Choose the type of build." FORCE)

	# Set the possible values of build type for cmake-gui
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
		"Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Maybe remove this under
# IF (CMAKE_COMPILER_IS_GNUCC)
# SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-error")
# SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error -Wpedantic -march=native")
# SET (CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops -DNDEBUG")
# SET (CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
# # Shared object compilation under 64bit (vtable)
# ADD_DEFINITIONS(-fPIC)
# ENDIF()

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
	# Support folders in IDE's
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)

	# Should this be here?
	# set(MODERN_CMAKE_BUILD_TESTING ON)
	include(CTest)

	find_package(Doxygen)

	if(Doxygen_FOUND)
		add_subdirectory(docs)
	else()
		message(STATUS "Doxygen not found, not building docs")
	endif()
endif()

# To make headers show up in IDEs
set(HEADER_LIST

	# "${PROJECT_SOURCE_DIR}/include/ufo/geometry/aabb.h"
)

set(SRC_LIST
	"${PROJECT_SOURCE_DIR}/src/map/io.cpp"
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)
pkg_check_modules(LZF REQUIRED liblzf)

find_package(OpenMP)

add_library(Map SHARED ${SRC_LIST} ${HEADER_LIST})
add_library(UFO::Map ALIAS Map)

set_target_properties(Map
	PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)

target_include_directories(Map
	PUBLIC
	$<INSTALL_INTERFACE:include>
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_link_libraries(Map
	PUBLIC
	${LZ4_LIBRARIES}
	OpenMP::OpenMP_CXX
)

find_package(TBB QUIET)

IF(TBB_FOUND)
	message(STATUS "TBB found, enabling parallel execution")
	target_compile_definitions(Map
		PUBLIC
		UFO_PARALLEL=1
	)
	target_link_libraries(Map
		PUBLIC
		TBB::tbb
	)
ELSE()
	message(STATUS "TBB not found, disabling parallel execution")
ENDIF()

target_compile_features(Map
	PUBLIC
	cxx_std_20
)

target_compile_options(Map
	PUBLIC
	-fPIC
)

# add_subdirectory(src)

add_executable(dufomap src/dufomap.cpp)
set_target_properties(dufomap
	PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)
target_link_libraries(dufomap PRIVATE UFO::Map)
target_compile_features(dufomap
	PUBLIC
	cxx_std_20
)
target_compile_options(dufomap
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

add_executable(fix_pcd src/fix_pcd.cpp)
set_target_properties(fix_pcd
	PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 20
	CXX_STANDARD_REQUIRED YES
	CXX_EXTENSIONS NO
)
target_link_libraries(fix_pcd PRIVATE UFO::Map)
target_compile_features(fix_pcd
	PUBLIC
	cxx_std_20
)

IF(TBB_FOUND)
	message(STATUS "TBB found, enabling parallel execution")
	target_compile_definitions(fix_pcd
		INTERFACE
		UFO_PARALLEL=1
	)
	target_link_libraries(fix_pcd
		PUBLIC
		TBB::tbb
	)
ENDIF()

set(UFOMAP_BMI2 FALSE CACHE BOOL "Enable/disable BMI2 instructions")

if(DEFINED ENV{UFOMAP_BMI2})
	set(UFOMAP_BMI2 $ENV{UFOMAP_BMI2})
endif(DEFINED ENV{UFOMAP_BMI2})

if(UFOMAP_BMI2)
	message(STATUS "UFOMAP BMI2 instructions enabled")
	target_compile_options(Map
		PUBLIC
		-mbmi2
	)
else()
	message(STATUS "UFOMAP BMI2 instructions disabled")
endif(UFOMAP_BMI2)

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

install(TARGETS Map
	EXPORT ufomapTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufomapTargets
	FILE "ufomapTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufomapConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/ufomapConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)