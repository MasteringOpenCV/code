cmake_minimum_required(VERSION 2.6)

project(V3D)

set (V3D_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set (V3D_INCLUDE_DIRS ${V3D_DIR}/.)

include (Config/v3d_macros.cmake)

include (Config/local_config.cmake)


file(GLOB COLAMD_SRC 
	COLAMD/Source/* 
	COLAMD/Include/* 
	SuiteSparse_config/SuiteSparse_config.h 
	SuiteSparse_config/SuiteSparse_config.c
	)
include_directories("SuiteSparse_config/")
add_library(colamd ${COLAMD_SRC})
set ( V3D_INCLUDE_DIRS ${V3D_INCLUDE_DIRS} "COLAMD/Include" )



include_directories(${V3D_INCLUDE_DIRS} ${EXTRA_INC_DIRS})


source_group("Base" REGULAR_EXPRESSION Base/.*cpp|Base.*h)
set (BASE_SRC
    Base/v3d_exception.h
    Base/v3d_serialization.h
    Base/v3d_vrmlio.h
)
if (V3DLIB_ENABLE_SOCKETS)
    set (BASE_SRC ${BASE_SRC} Base/v3d_socket.h)
endif (V3DLIB_ENABLE_SOCKETS)

source_group("Math" REGULAR_EXPRESSION Math/.*cpp|Math.*h)
set (MATH_SRC
    Math/v3d_linear.h
    Math/v3d_linearbase.h
    Math/v3d_mathutilities.h
    Math/v3d_nonlinlsq.h
    Math/v3d_nonlinlsq.cpp
    Math/v3d_optimization.h
    Math/v3d_optimization.cpp
    Math/v3d_optimization_lm.cpp
)

source_group("Geometry" REGULAR_EXPRESSION Geometry/.*cpp|Geometry.*h)
set (GEOMETRY_SRC
    Geometry/v3d_cameramatrix.h
    Geometry/v3d_distortion.h
    Geometry/v3d_metricbundle.h
    Geometry/v3d_metricbundle.cpp
    Geometry/v3d_stereobundle.h
    Geometry/v3d_stereobundle.cpp
)

set (ALL_SRC
    ${BASE_SRC} ${MATH_SRC} ${GEOMETRY_SRC} ${GL_SRC} ${CUDA_SRC}
)

add_library(V3D ${ALL_SRC})
add_subdirectory(Apps)
