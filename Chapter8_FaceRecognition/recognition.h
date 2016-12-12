/*****************************************************************************
*   Face Recognition using Eigenfaces or Fisherfaces
******************************************************************************
*   by Shervin Emami, 5th Dec 2012
*   http://www.shervinemami.info/openCV.html
******************************************************************************
*   Ch8 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////
// recognition.h, by Shervin Emami (www.shervinemami.info) on 30th May 2012.
// Train the face recognition system on a given dataset, and recognize the person from a given image.
//////////////////////////////////////////////////////////////////////////////////////
// Requires OpenCV v2.4.1 or later (from June 2012), otherwise the FaceRecognizer will not compile or run!
//////////////////////////////////////////////////////////////////////////////////////

#pragma once


#include <stdio.h>
#include <iostream>
#include <vector>

// Include OpenCV's C++ Interface
#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"


using namespace cv;
using namespace cv::face;
using namespace std;



// Start training from the collected faces.
// The face recognition algorithm can be one of these and perhaps more, depending on your version of OpenCV, which must be atleast v2.4.1:
//    "FaceRecognizer.Eigenfaces":  Eigenfaces, also referred to as PCA (Turk and Pentland, 1991).
//    "FaceRecognizer.Fisherfaces": Fisherfaces, also referred to as LDA (Belhumeur et al, 1997).
// Note: The LBPH algorithm was also available using the 1st Edition of Mastering OpenCV (with OpenCV 2.4)
Ptr<BasicFaceRecognizer> learnCollectedFaces(const vector<Mat> preprocessedFaces, const vector<int> faceLabels, const string facerecAlgorithm = "FaceRecognizer.Eigenfaces");

// Show the internal face recognition data, to help debugging.
void showTrainingDebugData(const Ptr<BasicFaceRecognizer> model, const int faceWidth, const int faceHeight);

// Generate an approximately reconstructed face by back-projecting the eigenvectors & eigenvalues of the given (preprocessed) face.
Mat reconstructFace(const Ptr<BasicFaceRecognizer> model, const Mat preprocessedFace);

// Compare two images by getting the L2 error (square-root of sum of squared error).
double getSimilarity(const Mat A, const Mat B);
