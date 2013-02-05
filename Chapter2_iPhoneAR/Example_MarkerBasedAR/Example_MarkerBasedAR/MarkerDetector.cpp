/*****************************************************************************
*   MarkerDetector.cpp
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

////////////////////////////////////////////////////////////////////
// Standard includes:
#include <iostream>
#include <sstream>

////////////////////////////////////////////////////////////////////
// File includes:
#include "MarkerDetector.hpp"
#include "Marker.hpp"
#include "TinyLA.hpp"
#include "DebugHelpers.hpp"

MarkerDetector::MarkerDetector(CameraCalibration calibration)
    : m_minContourLengthAllowed(100)
    , markerSize(100,100)
{
    cv::Mat(3,3, CV_32F, const_cast<float*>(&calibration.getIntrinsic().data[0])).copyTo(camMatrix);
    cv::Mat(4,1, CV_32F, const_cast<float*>(&calibration.getDistorsion().data[0])).copyTo(distCoeff);

    bool centerOrigin = true;
    if (centerOrigin)
    {
        m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
        m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
    }
    else
    {
        m_markerCorners3d.push_back(cv::Point3f(0,0,0));
        m_markerCorners3d.push_back(cv::Point3f(1,0,0));
        m_markerCorners3d.push_back(cv::Point3f(1,1,0));
        m_markerCorners3d.push_back(cv::Point3f(0,1,0));    
    }

    m_markerCorners2d.push_back(cv::Point2f(0,0));
    m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,0));
    m_markerCorners2d.push_back(cv::Point2f(markerSize.width-1,markerSize.height-1));
    m_markerCorners2d.push_back(cv::Point2f(0,markerSize.height-1));
}

void MarkerDetector::processFrame(const BGRAVideoFrame& frame)
{
    std::vector<Marker> markers;
    findMarkers(frame, markers);

    m_transformations.clear();
    for (size_t i=0; i<markers.size(); i++)
    {
        m_transformations.push_back(markers[i].transformation);
    }
}

const std::vector<Transformation>& MarkerDetector::getTransformations() const
{
    return m_transformations;
}


bool MarkerDetector::findMarkers(const BGRAVideoFrame& frame, std::vector<Marker>& detectedMarkers)
{
    cv::Mat bgraMat(frame.height, frame.width, CV_8UC4, frame.data, frame.stride);

    // Convert the image to grayscale
    prepareImage(bgraMat, m_grayscaleImage);

    // Make it binary
    performThreshold(m_grayscaleImage, m_thresholdImg);

    // Detect contours
    findContours(m_thresholdImg, m_contours, m_grayscaleImage.cols / 5);

    // Find closed contours that can be approximated with 4 points
    findCandidates(m_contours, detectedMarkers);

    // Find is them are markers
    recognizeMarkers(m_grayscaleImage, detectedMarkers);

    // Calculate their poses
    estimatePosition(detectedMarkers);

    //sort by id
    std::sort(detectedMarkers.begin(), detectedMarkers.end());
    return false;
}

void MarkerDetector::prepareImage(const cv::Mat& bgraMat, cv::Mat& grayscale) const
{
    // Convert to grayscale
    cv::cvtColor(bgraMat, grayscale, CV_BGRA2GRAY);
}

void MarkerDetector::performThreshold(const cv::Mat& grayscale, cv::Mat& thresholdImg) const
{
    cv::threshold(grayscale, thresholdImg, 127, 255, cv::THRESH_BINARY_INV);

    /*
    cv::adaptiveThreshold(grayscale,   // Input image
    thresholdImg,// Result binary image
    255,         // 
    cv::ADAPTIVE_THRESH_GAUSSIAN_C, //
    cv::THRESH_BINARY_INV, //
    7, //
    7  //
    );
    */

#ifdef SHOW_DEBUG_IMAGES
    cv::showAndSave("Threshold image", thresholdImg);
#endif
}

void MarkerDetector::findContours(cv::Mat& thresholdImg, ContoursVector& contours, int minContourPointsAllowed) const
{
    ContoursVector allContours;
    cv::findContours(thresholdImg, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    contours.clear();
    for (size_t i=0; i<allContours.size(); i++)
    {
        int contourSize = allContours[i].size();
        if (contourSize > minContourPointsAllowed)
        {
            contours.push_back(allContours[i]);
        }
    }

#ifdef SHOW_DEBUG_IMAGES
    {
        cv::Mat contoursImage(thresholdImg.size(), CV_8UC1);
        contoursImage = cv::Scalar(0);
        cv::drawContours(contoursImage, contours, -1, cv::Scalar(255), 2, CV_AA);
        cv::showAndSave("Contours", contoursImage);
    }
#endif
}

void MarkerDetector::findCandidates
(
    const ContoursVector& contours, 
    std::vector<Marker>& detectedMarkers
) 
{
    std::vector<cv::Point>  approxCurve;
    std::vector<Marker>     possibleMarkers;

    // For each contour, analyze if it is a parallelepiped likely to be the marker
    for (size_t i=0; i<contours.size(); i++)
    {
        // Approximate to a polygon
        double eps = contours[i].size() * 0.05;
        cv::approxPolyDP(contours[i], approxCurve, eps, true);

        // We interested only in polygons that contains only four points
        if (approxCurve.size() != 4)
            continue;

        // And they have to be convex
        if (!cv::isContourConvex(approxCurve))
            continue;

        // Ensure that the distance between consecutive points is large enough
        float minDist = std::numeric_limits<float>::max();

        for (int i = 0; i < 4; i++)
        {
            cv::Point side = approxCurve[i] - approxCurve[(i+1)%4];            
            float squaredSideLength = side.dot(side);
            minDist = std::min(minDist, squaredSideLength);
        }

        // Check that distance is not very small
        if (minDist < m_minContourLengthAllowed)
            continue;

        // All tests are passed. Save marker candidate:
        Marker m;

        for (int i = 0; i<4; i++)
            m.points.push_back( cv::Point2f(approxCurve[i].x,approxCurve[i].y) );

        // Sort the points in anti-clockwise order
        // Trace a line between the first and second point.
        // If the third point is at the right side, then the points are anti-clockwise
        cv::Point v1 = m.points[1] - m.points[0];
        cv::Point v2 = m.points[2] - m.points[0];

        double o = (v1.x * v2.y) - (v1.y * v2.x);

        if (o < 0.0)		 //if the third point is in the left side, then sort in anti-clockwise order
            std::swap(m.points[1], m.points[3]);

        possibleMarkers.push_back(m);
    }


    // Remove these elements which corners are too close to each other.  
    // First detect candidates for removal:
    std::vector< std::pair<int,int> > tooNearCandidates;
    for (size_t i=0;i<possibleMarkers.size();i++)
    { 
        const Marker& m1 = possibleMarkers[i];

        //calculate the average distance of each corner to the nearest corner of the other marker candidate
        for (size_t j=i+1;j<possibleMarkers.size();j++)
        {
            const Marker& m2 = possibleMarkers[j];

            float distSquared = 0;

            for (int c = 0; c < 4; c++)
            {
                cv::Point v = m1.points[c] - m2.points[c];
                distSquared += v.dot(v);
            }

            distSquared /= 4;

            if (distSquared < 100)
            {
                tooNearCandidates.push_back(std::pair<int,int>(i,j));
            }
        }				
    }

    // Mark for removal the element of the pair with smaller perimeter
    std::vector<bool> removalMask (possibleMarkers.size(), false);

    for (size_t i=0; i<tooNearCandidates.size(); i++)
    {
        float p1 = perimeter(possibleMarkers[tooNearCandidates[i].first ].points);
        float p2 = perimeter(possibleMarkers[tooNearCandidates[i].second].points);

        size_t removalIndex;
        if (p1 > p2)
            removalIndex = tooNearCandidates[i].second;
        else
            removalIndex = tooNearCandidates[i].first;

        removalMask[removalIndex] = true;
    }

    // Return candidates
    detectedMarkers.clear();
    for (size_t i=0;i<possibleMarkers.size();i++)
    {
        if (!removalMask[i])
            detectedMarkers.push_back(possibleMarkers[i]);
    }
}

void MarkerDetector::recognizeMarkers(const cv::Mat& grayscale, std::vector<Marker>& detectedMarkers)
{
    std::vector<Marker> goodMarkers;

    // Identify the markers
    for (size_t i=0;i<detectedMarkers.size();i++)
    {
        Marker& marker = detectedMarkers[i];

        // Find the perspective transformation that brings current marker to rectangular form
        cv::Mat markerTransform = cv::getPerspectiveTransform(marker.points, m_markerCorners2d);

        // Transform image to get a canonical marker image
        cv::warpPerspective(grayscale, canonicalMarkerImage,  markerTransform, markerSize);

#ifdef SHOW_DEBUG_IMAGES
        {
            cv::Mat markerImage = grayscale.clone();
            marker.drawContour(markerImage);
            cv::Mat markerSubImage = markerImage(cv::boundingRect(marker.points));

            cv::showAndSave("Source marker" + ToString(i),           markerSubImage);
            cv::showAndSave("Marker " + ToString(i) + " after warp", canonicalMarkerImage);
        }
#endif

        int nRotations;
        int id = Marker::getMarkerId(canonicalMarkerImage, nRotations);
        if (id !=- 1)
        {
            marker.id = id;
            //sort the points so that they are always in the same order no matter the camera orientation
            std::rotate(marker.points.begin(), marker.points.begin() + 4 - nRotations, marker.points.end());

            goodMarkers.push_back(marker);
        }
    }  

    // Refine marker corners using sub pixel accuracy
    if (goodMarkers.size() > 0)
    {
        std::vector<cv::Point2f> preciseCorners(4 * goodMarkers.size());

        for (size_t i=0; i<goodMarkers.size(); i++)
        {  
            const Marker& marker = goodMarkers[i];      

            for (int c = 0; c <4; c++)
            {
                preciseCorners[i*4 + c] = marker.points[c];
            }
        }

        cv::TermCriteria termCriteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.01);
        cv::cornerSubPix(grayscale, preciseCorners, cvSize(5,5), cvSize(-1,-1), termCriteria);

        // Copy refined corners position back to markers
        for (size_t i=0; i<goodMarkers.size(); i++)
        {
            Marker& marker = goodMarkers[i];      

            for (int c=0;c<4;c++) 
            {
                marker.points[c] = preciseCorners[i*4 + c];
            }      
        }
    }

#if SHOW_DEBUG_IMAGES
    {
        cv::Mat markerCornersMat(grayscale.size(), grayscale.type());
        markerCornersMat = cv::Scalar(0);

        for (size_t i=0; i<goodMarkers.size(); i++)
        {
            goodMarkers[i].drawContour(markerCornersMat, cv::Scalar(255));    
        }

        cv::showAndSave("Markers refined edges", grayscale * 0.5 + markerCornersMat);
    }
#endif

    detectedMarkers = goodMarkers;
}


void MarkerDetector::estimatePosition(std::vector<Marker>& detectedMarkers)
{
    for (size_t i=0; i<detectedMarkers.size(); i++)
    {					
        Marker& m = detectedMarkers[i];

        cv::Mat Rvec;
        cv::Mat_<float> Tvec;
        cv::Mat raux,taux;
        cv::solvePnP(m_markerCorners3d, m.points, camMatrix, distCoeff,raux,taux);
        raux.convertTo(Rvec,CV_32F);
        taux.convertTo(Tvec ,CV_32F);

        cv::Mat_<float> rotMat(3,3); 
        cv::Rodrigues(Rvec, rotMat);

        // Copy to transformation matrix
        for (int col=0; col<3; col++)
        {
            for (int row=0; row<3; row++)
            {        
                m.transformation.r().mat[row][col] = rotMat(row,col); // Copy rotation component
            }
            m.transformation.t().data[col] = Tvec(col); // Copy translation component
        }

        // Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
        m.transformation = m.transformation.getInverted();
    }
}
