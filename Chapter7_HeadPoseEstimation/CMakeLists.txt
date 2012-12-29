cmake_minimum_required (VERSION 2.6) 

PROJECT(HeadOrientation)

# Requires OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

SET(SRC
    main.cpp
    PAW.cpp
    Triangle.cpp
    09-1m.jpg
    09-2m.jpg
    09-3m.jpg
    simple-aam.txt
)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} )

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to HeadOrientation
    COMMAND ${CMAKE_COMMAND} -E copy_if_different           # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/09-1m.jpg"                   # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to HeadOrientation
    COMMAND ${CMAKE_COMMAND} -E copy_if_different           # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/09-2m.jpg"                   # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to HeadOrientation
    COMMAND ${CMAKE_COMMAND} -E copy_if_different           # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/09-3m.jpg"                   # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to HeadOrientation
    COMMAND ${CMAKE_COMMAND} -E copy_if_different           # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/simple-aam.txt"              # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

