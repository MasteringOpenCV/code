add_executable(markerless_ar_demo ARDrawingContext.cpp
ARDrawingContext.hpp
CameraCalibration.cpp
CameraCalibration.hpp
GeometryTypes.cpp
GeometryTypes.hpp
main.cpp
ARPipeline.hpp
ARPipeline.cpp
Pattern.cpp
Pattern.hpp
PatternDetector.cpp
PatternDetector.hpp
DebugHelpers.hpp
)

target_link_libraries( markerless_ar_demo ${OpenCV_LIBRARIES} )
target_link_libraries( markerless_ar_demo ${OPENGL_LIBRARIES} )
     
install (TARGETS markerless_ar_demo DESTINATION bin)
