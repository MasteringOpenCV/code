cmake_minimum_required (VERSION 2.6) 

PROJECT(ANPR)
SET(PROJECT_VERSION "0.1")

# Opencv Package
FIND_PACKAGE( OpenCV REQUIRED )
IF (${OpenCV_VERSION} VERSION_LESS 2.3.0)
	MESSAGE(FATAL_ERROR "OpenCV version is not compatible : ${OpenCV_VERSION}")
ENDIF()

SET(SRC
 main.cpp
 DetectRegions.cpp
 OCR.cpp
 Plate.cpp
)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRC} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME}  ${OpenCV_LIBS} )

ADD_EXECUTABLE( trainSVM trainSVM.cpp )
TARGET_LINK_LIBRARIES( trainSVM  ${OpenCV_LIBS} )

ADD_EXECUTABLE( trainOCR trainOCR.cpp OCR.cpp )
TARGET_LINK_LIBRARIES( trainOCR  ${OpenCV_LIBS} )

ADD_EXECUTABLE( evalOCR evalOCR.cpp OCR.cpp )
TARGET_LINK_LIBRARIES( evalOCR  ${OpenCV_LIBS} )

SET(RESOURCES
	README.txt
	SVM.xml
	OCR.xml
	test
)
foreach(f ${RESOURCES})
file( COPY ${f} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
endforeach(f)
