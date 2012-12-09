/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      KinectController.cpp
 * @author    Naureen Mahmood 
 * @copyright 2011 Austin Hines, Naureen Mahmood, and Texas A&M Dept. of Visualization
 * @version	  1.0.1
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

#include "KinectController.h"

// XnOpenNI Callbacks when user is detected or lost
void XN_CALLBACK_TYPE User_NewUser_Cback  (xn::UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser_Cback (xn::UserGenerator& generator, XnUserID nId, void* pCookie);

// Initialize all KinectController variables & modules
XnStatus KinectController::init()
{	
	initDepthControl();			
	CHECK_RC(xnRetVal, "InitDepthControl");
	return xnRetVal;
}


/****************************************//**
*	Depth & User Tracking Modules
********************************************/
XnStatus KinectController::initDepthControl()
{	
	// Initializing maximum iterations of Kinect data stream capture
	// before resetting them (this is useful when running the application 
	// in crowded environments so that the kinect can purge tracking 
	// information of previous users and quickly continue tracking the most recent 
	// players)
	_maxIterate		= ITERATIONS_BEFORE_RESET; 	
	_iterationCount = 0;
	
	// Initialize Status variable and creating Context object
	xnRetVal = XN_STATUS_OK;	
	xnRetVal = xnContext.Init(); 	
	CHECK_RC(xnRetVal, "Context.Init");
	
 
	// DepthGenerator:	Create node
	xnRetVal = xnDepthGenerator.Create(xnContext); 
	CHECK_RC(xnRetVal, "DepthGenerator.Create");

	// DepthGenerator:	Set it to VGA maps at 30 FPS 
	XnMapOutputMode mapMode; 
	mapMode.nXRes = XN_VGA_X_RES; 
	mapMode.nYRes = XN_VGA_Y_RES; 
	mapMode.nFPS = 30; 

	// DepthGenerator:	Set MapMode 
	xnRetVal = xnDepthGenerator.SetMapOutputMode(mapMode); 
	CHECK_RC(xnRetVal, "DepthGenerator.SetOutputMode");
		
	
	// UserGenerator: Create node 
	xnRetVal = xnUserGenerator.Create(xnContext); 
	CHECK_RC(xnRetVal, "UserGenerator.Create");	

	// UserGenerator:  Set Callbacks Handles 
	XnCallbackHandle cbackHandle;
	xnUserGenerator.RegisterUserCallbacks (User_NewUser_Cback, User_LostUser_Cback, NULL, cbackHandle);
	
	xnDepthGenerator.GetMetaData(xnDepthMD);
	xnUserGenerator.GetUserPixels(0, xnSceneMD);
	

	// Generate all objects
	xnRetVal = xnContext.StartGeneratingAll();
	CHECK_RC(xnRetVal, "StartGenerating");	

	return xnRetVal;
}

// Update the XnOpenNI Depth & User tracking data for each frame of video captured
XnStatus KinectController::update()
{
	// Restart all kinect processes every once in a while
	if (_iterationCount > _maxIterate)
		reset();

	// Context:	Wait for new data to be available 
	xnRetVal = xnContext.WaitOneUpdateAll(xnDepthGenerator);
	CHECK_RC(xnRetVal, "UpdateAll");	
	
	// DepthGenerator:	Take current depth map 
	const XnDepthPixel* pDepthMap	= xnDepthGenerator.GetDepthMap(); 
	const XnDepthPixel* pDepth		= xnDepthMD.Data();
	const XnLabel*		pLabels		= xnSceneMD.Data();


	// UserGenerator:	Define number of users to be tracked
	XnUserID* userID	= new XnUserID [_maxUsers];	
	XnUInt16 nUsers		= _maxUsers;	 
	
	// UserGenerator:	Get Tracked Users' IDs
	xnUserGenerator.GetUsers(userID, nUsers);
	CHECK_RC(xnRetVal, "UserGenerator.GetUser");	
	

	// Create temp matrices to store depth & user detection values before flip
	Mat toflip_depthMatrix = Mat::zeros(480,640,CV_8UC1);
	Mat toflip_usersMatrix = Mat::zeros(480,640,CV_8UC1);

	// Iterate through the current frame and store depth & user detection values in above matrices
	for (int ind = 0; ind < HEIGHT * WIDTH; ind++)
	{	// check if current pixel is within depth threshold
		if (pDepthMap[ind] < _maxDepth)	
		{
			toflip_depthMatrix.data[ind] = pDepthMap[ind];
			toflip_usersMatrix.data[ind] = pLabels[ind];
		}
	}

	flip( toflip_depthMatrix, _depthMatrix, -1 );
	flip( toflip_usersMatrix, _usersMatrix, -1 ); 
		
	_iterationCount++;
	return xnRetVal;
}


// Shutdown and restart all Kinect modules
XnStatus KinectController::reset()
{
	kinectCleanupExit();
	_iterationCount		= 0;
	init   ();
	update ();
	return xnRetVal;
}

/*! Set Depth Threshold		*/
void KinectController::setDepth(int depthDelta)
{	
	_maxDepth += depthDelta;	
	cout<<"Depth Threshold: "<<_maxDepth<<endl;
}

// Shutdown function
void KinectController:: kinectCleanupExit()
{
	stopDepthControl();
}

/**
 * [Callback for when New User Detected]: This code is executed every time a new user is detected by UserGenerator node
 * @param generator	UserGenerator node
 * @param nId		ID of user detected (range: 1 to Maximum users allowed)
 * @param pCookie	any object can be passed through the register functions 
 *					(such as RegisterUserCallbacks() function) using this *pCookie 
 */
void XN_CALLBACK_TYPE User_NewUser_Cback(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{	cout << "New User: " << nId << endl;	}

/**
 * [Callback for when User Lost]: This code executed every time an existing 
 *	user is no longer visible to UserGenerator node for more than 10secs
 * @param generator	UserGenerator node
 * @param nId		ID of user (range: 1 to Maximum users allowed)
 * @param pCookie	any object can be passed through the register functions 
 *					(such as RegisterUserCallbacks() function) using this *pCookie 
 */
void XN_CALLBACK_TYPE User_LostUser_Cback (xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{	cout << "Lost user: " << nId << endl;	}
