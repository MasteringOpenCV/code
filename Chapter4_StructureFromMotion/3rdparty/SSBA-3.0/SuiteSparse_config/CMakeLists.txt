project( SuiteSparse_config )
cmake_minimum_required(VERSION 2.8)

file(GLOB SRCS
	*.h
	*.c)

add_library( SuiteSparse_config ${SRCS} )

set( SuiteSparse_config_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )