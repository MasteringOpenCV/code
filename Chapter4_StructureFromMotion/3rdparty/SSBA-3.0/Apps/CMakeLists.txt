include_directories(${V3D_INCLUDE_DIRS} ${EXTRA_INC_DIRS})
link_directories(${V3D_DIR} ${EXTRA_LIB_DIRS})
link_libraries (V3D ${EXTRA_LIBRARIES} ${EXTRA_LIBS})

set (CMAKE_BUILD_TYPE Release)
#set (CMAKE_BUILD_TYPE Debug)

if (V3DLIB_ENABLE_CUDA)
  add_subdirectory(CUDA)
endif (V3DLIB_ENABLE_CUDA)

if (V3DLIB_ENABLE_SUITESPARSE)
    add_v3d_executable (bundle_common bundle_common.cpp)
    add_v3d_executable (bundle_varying bundle_varying.cpp)
    add_v3d_executable (bundle_varying_nonlinlsq bundle_varying_nonlinlsq.cpp)
endif (V3DLIB_ENABLE_SUITESPARSE)

if (V3DLIB_ENABLE_CUDA AND V3DLIB_ENABLE_GPGPU AND (V3DLIB_ENABLE_SIFTGPU OR V3DLIB_ENABLE_VLFEAT OR V3DLIB_ENABLE_OPENCV))
   add_subdirectory (SfM)
endif (V3DLIB_ENABLE_CUDA AND V3DLIB_ENABLE_GPGPU AND (V3DLIB_ENABLE_SIFTGPU OR V3DLIB_ENABLE_VLFEAT OR V3DLIB_ENABLE_OPENCV))

if (V3DLIB_ENABLE_LIBDAI)
   add_subdirectory (CycleInference)
endif (V3DLIB_ENABLE_LIBDAI)
