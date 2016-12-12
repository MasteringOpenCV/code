cmake_minimum_required (VERSION 2.6) 

PROJECT(WebcamFaceRec)

# Requires OpenCV v3.0 or later
FIND_PACKAGE( OpenCV 3 REQUIRED )

SET(SRC
    main.cpp
    detectObject.cpp
    preprocessFace.cpp
    recognition.cpp
    ImageUtils_0.7.cpp
)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} )
