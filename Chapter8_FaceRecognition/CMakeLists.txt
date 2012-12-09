cmake_minimum_required (VERSION 2.6) 

PROJECT(WebcamFaceRec)

# Requires OpenCV v2.4.1 or later
FIND_PACKAGE( OpenCV REQUIRED )
IF (${OpenCV_VERSION} VERSION_LESS 2.4.1)
    MESSAGE(FATAL_ERROR "OpenCV version is not compatible : ${OpenCV_VERSION}. FaceRec requires atleast OpenCV v2.4.1")
ENDIF()

SET(SRC
    main.cpp
    detectObject.cpp
    preprocessFace.cpp
    recognition.cpp
    ImageUtils_0.7.cpp
)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} )
