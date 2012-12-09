/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      KinectController.h
 * @author    Naureen Mahmood 
 * @copyright 2011 Austin Hines, Naureen Mahmood, and Texas A&M Dept. of Visualization
 * @version   1.0.1
 * 
 * This file is part of Fluid Wall. You can redistribute it and/or modify            
 * it under the terms of the GNU Lesser General Public License as published  
 * by the Free Software Foundation, either version 3 of the License, or     
 * (at your option) any later version.                                       
 *                                                                             
 * Fluid Wall is distributed in the hope that it will be useful,                 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              
 * GNU Lesser General Public License for more details.                       
 *                                                                            
 * You should have received a copy of the GNU Lesser General Public License  
 * along with Fluid Wall. If not, see <http://www.gnu.org/licenses/>.            
 *
 * Version History:
 * 1.0.0
 *   - Initial Release
 */

#ifndef KINECT_CONTROLLER_H
#define KINECT_CONTROLLER_H

//OpenNI includes
#include <XnOpenNI.h>
#include <XnCppWrapper.h>

//OpenCV includes
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>

#define HEIGHT			XN_VGA_Y_RES
#define WIDTH			XN_VGA_X_RES
#define SAMPLE_XML_PATH "Data/SamplesConfig.xml"
#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return xnRetVal;											\
	}

#define MAX_USERS                6
#define MAX_DEPTH	             3000

#define ITERATIONS_BEFORE_RESET  10000
using namespace std;
using namespace cv;

//! KinectController Class
/*!
	KinectController Class initializes and runs all the modules 
	for controlling the kinect camera devices.
*/
class KinectController
{
public:
	/*
	* (Default) Constructor with initialization list.
	* @param	_maxDepth		initialize depth threshold for Kinect depth data stream(less than 6000)
	* @param	_maxUsers		initialize maximum users to be detected (between 1-6)
	* @param	_depthMatrix	initialize an empty cvMatrix to store the kinect depth-map
	* @param	_usersMatrix	initialize an empty cvMatrix to store the kinect userID-map
	*/
	/*!	Constructor with initialization lists	*/
	KinectController () :	_maxDepth( MAX_DEPTH ),
							_maxUsers( MAX_USERS ), 
							_depthMatrix( Mat::zeros(480,640,CV_8UC1) ),
							_usersMatrix( Mat::zeros(480,640,CV_8UC1) )
							{	init();		}
	
	/*!	Destructor	*/
	~KinectController() {	kinectCleanupExit();	}	
	/*! Initialize all KinectController variables & modules	*/
	XnStatus init();
	/*! Depth & User Tracking Modules	*/
	XnStatus update();
	/*! Update the XnOpenNI Depth & User tracking data for each frame of video captured */
	XnStatus reset();
	/*! Set Depth Threshold		*/
	void setDepth(int depthDelta);	
	/*! Get depth matrix for current video frame 	*/
	void getDepthMat(Mat &depth)		{	_depthMatrix.copyTo(depth); }
	/*! Get matrix	of tracked users for current video frame */
	void getUsersMat(Mat &users)		{	_usersMatrix.copyTo(users); }
	/*! Get maximum number of users to be tracked */
	int getMaxUsers()		{	return _maxUsers;	 }

private: 
	// OPENNI DEPTH & USER TRACKING VARIABLES
	xn::Context			xnContext;			/*! context object that creates depth and user data nodes	*/
	xn::DepthGenerator	xnDepthGenerator;	/*! captures and returns depth values at each frame	*/
	xn::UserGenerator	xnUserGenerator;	/*! captures and returns user detection data at each frame	*/
	xn::SceneMetaData	xnSceneMD;			/*! scene metadata: gives access to IDs of detected users at each pixel of a captured frame	*/
	xn::DepthMetaData	xnDepthMD;			/*! depth metadata: gives access to depth data at each pixel of a captured frame	*/

	XnStatus xnRetVal;						/*! used to check the status of each call to an XNOpenNI function	*/
	int		_maxUsers;						/*! users to detect	*/
	int		_maxDepth;					/*! depth threshold for how far the Kinect should capture	*/
	int		_maxIterate;					/*! iterations to run before reset	*/
	int		_iterationCount;				/*! running iterations so far (goes up to maxIterate then resets to 0)	*/
	Mat		_depthMatrix;					/*! image-sized matrix containing the depth values at each pixel	*/
	Mat		_usersMatrix;					/*! image-sized matrix containing the userID's of detected people at		
											/*! each pixel (or 0 if no detected user at that pixel)	*/
	/*! Initialize XnOpenNI depth control & user tracking modules */
	XnStatus initDepthControl();
	/*! Destroy & shutdown XnOpenNI depth control & user tracking modules */
	void stopDepthControl()		{	xnContext.Shutdown();		}
	/*! Run Shutdown functions for Depth control 	*/
	void kinectCleanupExit();
};

#endif