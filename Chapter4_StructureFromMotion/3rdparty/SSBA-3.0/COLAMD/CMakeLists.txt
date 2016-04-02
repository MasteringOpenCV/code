project(colamd)
cmake_minimum_required(VERSION 2.8)

file(GLOB SRCS
	Source/*
	Include/*)

include_directories(
	"Include"
	"../SuiteSparse_config/"
	)

add_library( colamd ${SRCS} )

target_link_libraries( colamd SuiteSparse_config )