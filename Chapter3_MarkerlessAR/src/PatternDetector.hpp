/*****************************************************************************
*   Markerless AR desktop application.
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef EXAMPLE_MARKERLESS_AR_PATTERNDETECTOR_HPP
#define EXAMPLE_MARKERLESS_AR_PATTERNDETECTOR_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "Pattern.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

class PatternDetector
{
public:
    /**
     * Initialize a pattern detector with specified feature detector, descriptor extraction and matching algorithm
     */
    PatternDetector
        (
        cv::Ptr<cv::FeatureDetector>     detector  = new cv::ORB(1000), 
        cv::Ptr<cv::DescriptorExtractor> extractor = new cv::FREAK(false, false), 
        cv::Ptr<cv::DescriptorMatcher>   matcher   = new cv::BFMatcher(cv::NORM_HAMMING, true),
        bool enableRatioTest                       = false
        );

    /**
    * 
    */
    void train(const Pattern& pattern);

    /**
    * Initialize Pattern structure from the input image.
    * This function finds the feature points and extract descriptors for them.
    */
    void buildPatternFromImage(const cv::Mat& image, Pattern& pattern) const;

    /**
    * Tries to find a @pattern object on given @image. 
    * The function returns true if succeeded and store the result (pattern 2d location, homography) in @info.
    */
    bool findPattern(const cv::Mat& image, PatternTrackingInfo& info);

    bool enableRatioTest;
    bool enableHomographyRefinement;
    float homographyReprojectionThreshold;

protected:

    bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

    void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches);

    /**
    * Get the gray image from the input image.
    * Function performs necessary color conversion if necessary
    * Supported input images types - 1 channel (no conversion is done), 3 channels (assuming BGR) and 4 channels (assuming BGRA).
    */
    static void getGray(const cv::Mat& image, cv::Mat& gray);

    /**
    * 
    */
    static bool refineMatchesWithHomography(
        const std::vector<cv::KeyPoint>& queryKeypoints, 
        const std::vector<cv::KeyPoint>& trainKeypoints, 
        float reprojectionThreshold,
        std::vector<cv::DMatch>& matches, 
        cv::Mat& homography);

private:
    std::vector<cv::KeyPoint> m_queryKeypoints;
    cv::Mat                   m_queryDescriptors;
    std::vector<cv::DMatch>   m_matches;
    std::vector< std::vector<cv::DMatch> > m_knnMatches;

    cv::Mat                   m_grayImg;
    cv::Mat                   m_warpedImg;
    cv::Mat                   m_roughHomography;
    cv::Mat                   m_refinedHomography;

    Pattern                          m_pattern;
    cv::Ptr<cv::FeatureDetector>     m_detector;
    cv::Ptr<cv::DescriptorExtractor> m_extractor;
    cv::Ptr<cv::DescriptorMatcher>   m_matcher;
};

#endif