==============================================================================
Mastering OpenCV with Practical Computer Vision Projects
==============================================================================
Full source-code for the book.
--------------------------------------------------------------------------------

    Source-Code:    https://github.com/MasteringOpenCV/code
    Book:           http://www.packtpub.com/cool-projects-with-opencv/book
    Copyright:      Packt Publishing 2012.


--------------------------------------------------------------------------------
To build & run the projects for the book:
--------------------------------------------------------------------------------
- Install OpenCV (versions between 2.4.2 to 2.4.11 are supported, whereas OpenCV 3.0 is not yet supported). eg: go to "http://opencv.org/", click on
  Downloads, download the latest OpenCV 2.4 version (including prebuilt library), and extract
  it to "C:\OpenCV" for Windows or "~/OpenCV" for Linux. In OpenCV v2.4.3, the
  prebuilt OpenCV library is in "C:\OpenCV\build" or "~/OpenCV/build", such as
  "C:\OpenCV\build\x64\vc9" for MS Visual Studio 2008 (or "vs10" folder for MS 
  Visual Studio 2010, or the "x86" parent folder for 32-bit Windows).

- Install all the source code of the book. eg: extract the code to
  "C:\MasteringOpenCV" for Windows or "~/MasteringOpenCV" for Linux.
  
- Install CMake v2.8 or later from "http://www.cmake.org/".

Each chapter of the book is for a separate project. Therefore there are 9
projects for the 9 chapters (remember that Chapter 9 is an online chapter that
can be downloaded from "http://www.packtpub.com/cool-projects-with-opencv/book").
You can run each project separately, they each contain a README.md text file
describing how to build that project, using CMake in most cases, because CMake
can be used with many compilers and many operating systems.


--------------------------------------------------------------------------------
Chapters:
--------------------------------------------------------------------------------
- Ch1) Cartoonifier and Skin Changer for Android, by Shervin Emami.
- Ch2) Marker-based Augmented Reality on iPhone or iPad, by Khvedchenia Ievgen.
- Ch3) Marker-less Augmented Reality, by Khvedchenia Ievgen.
- Ch4) Exploring Structure from Motion using OpenCV, by Roy Shilkrot.
- Ch5) Number Plate Recognition using SVM and Neural Networks, by David Escrivá.
- Ch6) Non-rigid Face Tracking, by Jason Saragih.
- Ch7) 3D Head Pose Estimation using AAM and POSIT, by Daniel Lélis Baggio.
- Ch8) Face Recognition using Eigenfaces or Fisherfaces, by Shervin Emami.
- Ch9) Developing Fluid Wall using the Microsoft Kinect, by Naureen Mahmood.


--------------------------------------------------------------------------------
What you need for this book:
--------------------------------------------------------------------------------
You don't need to have special knowledge in computer vision to read this book,
but you should have good C/C++ programming skills and basic experience with
OpenCV before reading this book. Readers without experience in OpenCV may wish to
read the book Learning OpenCV for an introduction to the OpenCV features, or read
"OpenCV 2 Cookbook" for examples on how to use OpenCV with recommended C/C++
patterns, because "Mastering OpenCV with Practical Computer Vision Projects" will
show you how to solve real problems, assuming you are already familiar with the
basics of OpenCV and C/C++ development.

In addition to C/C++ and OpenCV experience, you will also need a computer, and an
IDE of your choice (such as Visual Studio, XCode, Eclipse, or QtCreator, running
on Windows, Mac or Linux). Some chapters have further requirements, particularly:

- To develop the Android app, you will need an Android device, Android
  development tools, and basic Android development experience.
- To develop the iOS app, you will need an iPhone, iPad, or iPod Touch device,
  iOS development tools (including an Apple computer, XCode IDE, and an Apple
  Developer Certificate), and basic iOS and Objective-C development experience.
- Several desktop projects require a webcam connected to your computer. Any
  common USB webcam should suffice, but a webcam of at least 1 megapixel may be
  desirable.
- CMake is used in most projects, including OpenCV itself, to build across
  operating systems and compilers. A basic understanding of build systems is
  required, and knowledge of cross-platform building is recommended.
- An understanding of linear algebra is expected, such as basic vector and matrix
  operations and eigen decomposition.

Per-chapter Requirements:
- Ch1: webcam (for desktop app), or Android development system (for Android app).
- Ch2: iOS development system (to build an iOS app).
- Ch3: OpenGL built into OpenCV.
- Ch4: PCL (http://pointclouds.org/) and SSBA (http://www.inf.ethz.ch/personal/chzach/opensource.html).
- Ch5: nothing.
- Ch6: nothing, but requires training data for execution.
- Ch7: nothing.
- Ch8: webcam.
- Ch9: Kinect depth sensor.


--------------------------------------------------------------------------------
Screenshots:
--------------------------------------------------------------------------------
- Ch1) Cartoonifier and Skin Changer for Android:
![Ch1) Cartoonifier and Skin Changer for Android](https://raw.github.com/MasteringOpenCV/code/master/Chapter1_AndroidCartoonifier/screenshot.png)
- Ch2) Marker-based Augmented Reality on iPhone or iPad:
![Ch2) Marker-based Augmented Reality on iPhone or iPad](https://raw.github.com/MasteringOpenCV/code/master/Chapter2_iPhoneAR/screenshot.png)
- Ch3) Marker-less Augmented Reality:
![Ch3) Marker-less Augmented Reality](https://raw.github.com/MasteringOpenCV/code/master/Chapter3_MarkerlessAR/screenshot.png)
- Ch4) Exploring Structure from Motion using OpenCV:
![Ch4) Exploring Structure from Motion using OpenCV](https://raw.github.com/MasteringOpenCV/code/master/Chapter4_StructureFromMotion/screenshot.png)
- Ch5) Number Plate Recognition using SVM and Neural Networks:
![Ch5) Number Plate Recognition using SVM and Neural Networks](https://raw.github.com/MasteringOpenCV/code/master/Chapter5_NumberPlateRecognition/screenshot.png)
- Ch6) Non-rigid Face Tracking:
![Ch6) Non-rigid Face Tracking](https://raw.github.com/MasteringOpenCV/code/master/Chapter6_NonRigidFaceTracking/screenshot.png)
- Ch7) 3D Head Pose Estimation using AAM and POSIT:
![Ch7) 3D Head Pose Estimation using AAM and POSIT](https://raw.github.com/MasteringOpenCV/code/master/Chapter7_HeadPoseEstimation/screenshot.png)
- Ch8) Face Recognition using Eigenfaces or Fisherfaces:
![Ch8) Face Recognition using Eigenfaces or Fisherfaces](https://raw.github.com/MasteringOpenCV/code/master/Chapter8_FaceRecognition/screenshot.png)
- Ch9) Developing Fluid Wall using the Microsoft Kinect:
![Ch9) Developing Fluid Wall using the Microsoft Kinect](https://raw.github.com/MasteringOpenCV/code/master/Chapter9_FluidInteractionUsingKinect/screenshot.png)


