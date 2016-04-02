# Other optional libraries
enable_feature (V3DLIB_ENABLE_SUITESPARSE)
enable_feature_inc_path (V3DLIB_ENABLE_SUITESPARSE /usr/include/suitesparse)
enable_feature_libraries (V3DLIB_ENABLE_SUITESPARSE colamd)

# Debug/release mode selection
set (CMAKE_BUILD_TYPE Release)
#set (CMAKE_BUILD_TYPE Debug)
