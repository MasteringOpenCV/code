******************************************************************************
*   Non-Rigid Face Tracking
******************************************************************************
*   by Jason Saragih, 5th Dec 2012
*   http://jsaragih.org/
******************************************************************************
*   Ch6 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
******************************************************************************


----------------------------------------------------------
Building the project using CMake from the command-line:
----------------------------------------------------------
Linux:
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -D OpenCV_DIR=$OpenCV_DIR ../src
    make 

MacOSX (Xcode):
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -G Xcode -D OpenCV_DIR=$OpenCV_DIR ../src
    open OPENCV_HOTSHOTS.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ../src
    start OPENCV_HOTSHOTS.sln 
    
- A static library will be written to the "lib" directory.
- The execuables can be found in the "bin" directory.

----------------------------------------------------------
Running the various programs:
----------------------------------------------------------
* On Linux or Mac:	./bin/program_name --help
* On Windows:		bin\Debug\program_name --help
