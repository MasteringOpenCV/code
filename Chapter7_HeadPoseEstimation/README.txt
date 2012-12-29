******************************************************************************
*   3D Head Pose Estimation using AAM and POSIT
******************************************************************************
*   by Daniel Lélis Baggio, 29th Dec 2012
*   http://code.google.com/p/ehci/
******************************************************************************
*   Ch7 of the book "Mastering OpenCV with Practical Computer Vision Projects"
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
    cmake -D OpenCV_DIR=$OpenCV_DIR ..
    make 

MacOSX (Xcode):
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -G Xcode -D OpenCV_DIR=$OpenCV_DIR ..
    open HeadOrientation.xcodeproj

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
    start HeadOrientation.sln 
    
----------------------------------------------------------
Running the project:
----------------------------------------------------------
Simply run the HeadOrientation executable. This should load a simple-aam as well as some example images so the reader can become familiar with eigenvalues used in the image PCA. Sliding the bars will instance a new AAM which will be displayed on top of the training images. Hitting 'c' will close the application. The keys '1', '2', and '3' will change the displayed training image.

