# -*- CMake -*-

macro (enable_feature feature)
  set (${feature} 1)
  add_definitions(-D${feature})
endmacro (enable_feature)

macro (enable_conditional_feature feature dep_feature)
  if (${dep_feature})
    set (${feature} 1)
    add_definitions(-D${feature})
  endif (${dep_feature})
endmacro (enable_conditional_feature)

macro (enable_feature_inc_path feature)
  if (${feature})
    set (EXTRA_INC_DIRS ${EXTRA_INC_DIRS} ${ARGN})
  endif (${feature})
endmacro (enable_feature_inc_path)

macro (enable_feature_lib_path feature)
  if (${feature})
    set (EXTRA_LIB_DIRS ${EXTRA_LIB_DIRS} ${ARGN})
  endif (${feature})
endmacro (enable_feature_lib_path)

macro (enable_feature_libraries feature)
  if (${feature})
    set (EXTRA_LIBRARIES ${EXTRA_LIBRARIES} ${ARGN})
  endif (${feature})
endmacro (enable_feature_libraries)

macro (add_v3d_executable target)
  add_executable(${target} ${ARGN})
  add_dependencies(${target} V3D)
endmacro (add_v3d_executable)

macro (add_v3d_executable_with_openmp target)
  add_executable(${target} ${ARGN})
  add_dependencies(${target} V3D)
  set_target_properties (${target} PROPERTIES LINK_FLAGS -fopenmp)
  set_source_files_properties (${ARGN} PROPERTIES COMPILE_FLAGS -fopenmp)
endmacro (add_v3d_executable_with_openmp)

macro (add_simple_v3d_executable target)
  add_executable(${target} ${target}.cpp)
  add_dependencies(${target} V3D)
endmacro (add_simple_v3d_executable)
