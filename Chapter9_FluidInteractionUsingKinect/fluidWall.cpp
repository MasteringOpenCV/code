/*****************************************************************************
*   Ch9 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*   http://code.google.com/p/fluidwall/
*****************************************************************************/
/**
 * @file      fluidWall.cpp
 * @author    Austin Hines <futurelightstudios@gmail.com>
 * @copyright 2011 Austin Hines, Naureen Mahmood, and Texas A&M Dept. of Visualization
 * @version	  1.0.1
 * 
 * The main executable for Fluid Wall. Contains functions that define how the
 * KinectController class effects the FluidSolver classes. This defines the general
 * behavior of the simulation. 
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
 * 1.0.1
 *	 - Added fullscreen toggle mode, bound to 'q' key. 
 *   - Removed 'q' key as quit.
 *   - Added Version tag to GLUT window and command line output.
 * 1.0.0
 *   - Initial Release
 *
 */ 


#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <ctype.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>

#include <GL/glut.h>

#include "FluidSolver.h"
#include "FluidSolverMultiUser.h"
#include "KinectController.h"

static const char* VERSION = "1.0.1 BETA";

#ifndef GL_BGR //fix omission from windows OpenGL implementation
	#define GL_BGR GL_BGR_EXT
#endif
#ifndef GL_BGRA
	#define GL_BGRA GL_BGRA_EXT
#endif

#define DEBUG	0
#define USE_GPU 0

// macros 
#define ROW_WIDTH		N+2
#define IX(i, j)		((i) + (ROW_WIDTH) * (j))
#define FOR_EACH_CELL	for(i = 1; i <= N; i++) { for(j = 1; j <= N; j++) {
#define END_FOR			}}
#define SWAP(x0, x)		{ float* tmp = x0; x0 = x; x = tmp; }

///// constants
const static int	GRID_SIZE		= 128;
const static float	FLOW_SCALE		= 0.1;
const static int	SPLASH_ROWS		= 80;
const static float	BG_OFFSET		= 0.1;
const static int	DEF_WINDOW_SIZE	= 512;

using namespace std;
using namespace cv; 
using namespace cv::gpu;

typedef struct 
{ 
	Point2f center;
	Point2f vel; 
	int lifespan, lifeElapsed;
	int radius;
	int userID;
} Emitter;

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

static FluidSolver *solver; 
static FluidSolverMultiUser *userSolver;
static KinectController *kinect;
static bool useUserSolver = false;

static Mat depthMatrix;
static Mat usersMatrix;
static Mat resizedUsersMatrix;

GLfloat colors[][3] =                     // user colors for fluid emission
{
	//{0.02f,0.02f,0.02f},
	{0.0f,0.0f,0.0f},
	{0.0f,1.0f,1.0f},
	{0.5f,1.0f,0.0f},
	{0.5f,0.5f,0.0f},
	{0.0f,0.4f,0.6f},
	{0.0f,1.0f,0.0f},
	{1.0f,0.5f,0.0f},
	{1.0f,1.0f,0.0f},
	{1.0f,0.0f,0.0f},
	{0.0f,0.5f,1.0f},
	{1.0f,1.0f,0.5f},
	{1.0f,1.0f,1.0f}
};

GLfloat colorsWhiteBG[][3] =               // user colors for fluid emission
{
	{0.02f,0.02f,0.02f},
	{0.0f,1.0f,1.0f},
	{0.5f,1.0f,0.0f},
	{1.0f,0.5f,0.0f},
	{0.0f,0.0f,1.0f},
	{0.0f,1.0f,0.0f},
	{1.0f,1.0f,0.0f},
	{1.0f,0.0f,0.0f},
	{0.0f,0.5f,1.0f},
	{0.5f,0.0f,1.0f},
	{1.0f,1.0f,0.5f},
	{1.0f,1.0f,1.0f}
};


//particle system variables
static int N;
static float force  = 5.0f;
static float source = 20.0f;
static int MAX_EMITTERS = 200;

static bool useFlow;					//use optical flow
static vector<Emitter> emittersList(MAX_EMITTERS);

static Mat depthImage;
static Mat flow;					//optical flow matrix
static Mat currFlowImg, prevFlowImg;
static GpuMat gpuFlowX, gpuFlowY;	//gpu-enabled optical flow matrix 
static GpuMat gpuPrevFlowImg, gpuCurrFlowImg;

//OpenGL
static int winID;
static int winX, winY;
static int mouseDown[3];
static int omx, omy, mx, my;

//display flags
static int dvel, dbound, dusers;

//mode change variables
static bool autoChangeMode = false;
static bool useWhiteBackground = false;
static int mode = 0;
static int maxMode = 3;
static int iterations = 0;
static int iterationsPerMode = 500; //frames per mode

//forward method declarations
static void changeMode(int newMode);
static void openGlutWindow ( void );
static void initOpenGl();
static void toggleFullscreen();
/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/

/**
 * Clears all solver data.
 */
static void clearData(void)
{
	if(useUserSolver) 
		userSolver->reset();
	else
		solver->reset();

	emittersList.clear();
}

/**
 * Initializes all objects and defines constant variables used in main program.
 * TODO: relegate this code to a singleton class.
 */
static int allocateData ( void )
{
	solver = new FluidSolver(GRID_SIZE, 0.1f, 0.00f, 0.0f);
	kinect = new KinectController(); 
	userSolver = new FluidSolverMultiUser(kinect->getMaxUsers(), GRID_SIZE,0.1f, 0.00f, 0.0f);
	emittersList.reserve(MAX_EMITTERS);
	for(int i = 0; i < MAX_EMITTERS; i++)
	{
		Emitter newEmit = {Point(0, 0), Point2f(0, 0), 1, 2, 1, 0};
		emittersList.push_back(newEmit);
	}

	N = GRID_SIZE;
	flow = Mat::zeros(N, N, CV_32FC2);

	useFlow = true;

	#if DEBUG
		namedWindow("Users",1);
		namedWindow("flow", 1);
	#endif

	return ( 1 );
}


/**
 * Cleans up any allocated memory.
 */
void cleanupExit()
{
	if (glutGameModeGet(GLUT_GAME_MODE_ACTIVE))
		glutLeaveGameMode();
	exit(0);
}


/**
 * Used for debug and basic testing of fluid simulation. Drives fluid simulation based on
 * mouse input. 
 *
 * - Left Mouse Button + Drag adds velocity
 * - Middle Mouse Button adds boundaries, 
 * - Right Mouse Button adds density. 
 *
 */
static void getForcesFromMouse(FluidSolver* flSolver)
{
	int x, y;

	bool noButtonsPressed = !mouseDown[0] && !mouseDown[2] && !mouseDown[1];
	if (noButtonsPressed) return;

	// determine mouse position on the fluid grid by divide screenspace by N gridspaces
	x = (int)((         mx  / (float)winX) * N + 1);
	y = (int)(((winY - my) / (float)winY) * N + 1);

	bool isMouseOutsideFluidGrid = (x < 1) || (x > N) || (y < 1) || (y > N);
	if (isMouseOutsideFluidGrid) return;

	if (mouseDown[0]) {	//left mouse button
		flSolver->addHorzVelocityAt(x, y, force * (mx - omx));
		flSolver->addVertVelocityAt(x, y, force * (omy - my));
	}

	if (mouseDown[1])		// middle mouse button
		flSolver->setBoundAt(x, y, true);

	if (mouseDown[2])		// right mouse button
		if(useUserSolver)
			userSolver->addDensityAt(1, x, y, source);
		else
			flSolver->addDensityAt(x, y, source);

	omx = mx;
	omy = my;
	return;
}


/**
 * Loads kinect depth and users data into local matrices. 
 * Resizes kinect data to fluid simulation grid size:  
 * resizedDepthMatrix = NxN matrix
 * resizedUsersMatrix = NxN matrix
 */
int loadKinectData()
{
	Mat resizedDepthMatrix = Mat::zeros(N, N, CV_8UC1);
	Mat resizedUsersMatrix = Mat::zeros(N, N, CV_8UC1);

	// depth tracking
	kinect->update();
	kinect->getDepthMat(depthMatrix);
	if(depthMatrix.empty()) 
	{
		cout<<"ERROR: Cannot load depth frame"<<endl;
		return -1;
	}

	// Resize depth image to simulation size.
	resize(depthMatrix, resizedDepthMatrix, resizedDepthMatrix.size());
	depthImage = resizedDepthMatrix;

	// Copy resized depthImage for optical flow to use.
	resizedDepthMatrix.copyTo(currFlowImg);

	if(useUserSolver)
	{
		// Retrieve and resize UserID matrix.
		kinect->getUsersMat(usersMatrix);
		resize(usersMatrix, resizedUsersMatrix, resizedUsersMatrix.size());
	}
    return 0;
}

/**
 * Translates input depthImage or userID values into collision 
 * boundaries in the FluidSolver. Currently set to define 
 * a boundary at any pixel in the depthImage with a 
 * value greater than zero. 
 */
static void defineBoundsFromImage(FluidSolver* flSolver, Mat &bounds)
{
    for( int y = 0; y < bounds.rows; y++ )
        for( int x = 0; x < bounds.cols; x++ ) 
		{
			uchar &pixelVal = bounds.at<uchar>(y, x);
			//add + 1 to coordinates because fluid matrix indicies range from 1 - N
			if( pixelVal > 0)
				flSolver->setBoundAt(x, y, true);
			else
				flSolver->setBoundAt(x, y, false);
        }
}


/**
 * Draws and displays a graphical representation of the optical flow results using OpenCV.
 *
 * @param flow		- Matrix of type CV_32FC2 containing results of optical flow calculation.
 * @param cflowmap	- Color image representing the input of optical flow
 * @param step		- number of pixels to skip when drawing each vector. (drawing a vector for 
 *                    every pixel would be too dense and graphically noisy.
 * @param color		- CV_RGB scalar specifying color to draw the optical flow vectors.
 */
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const Point2f& fxy = flow.at<Point2f>(y, x);
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }
}



/**
 * Translates optical flow into velocity values. Flow values are 
 * rounded with cvRound to eliminate noise. Results are added directly into FluidSolver.
 */
static void computeOpticalFlow(FluidSolver* flSolver, Mat& flow)
{
	if(!prevFlowImg.empty()) 
	{
		#if USE_GPU
			GpuMat d_frameL(prevFlowImg), d_frameR(currFlowImg);
			GpuMat d_flowx, d_flowy;
			FarnebackOpticalFlow calcFlowFB;
			Mat flowx, flowy;
	
			calcFlowFB(d_frameL, d_frameR, d_flowx, d_flowy);
			d_flowx.download(flowx);
			d_flowy.download(flowy);  
			Mat planes[] = {flowx, flowy};
			planes->copyTo(flow);
		#else
			calcOpticalFlowFarneback(prevFlowImg, currFlowImg, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		#endif
		for(int y = 1; y < N; y++)
		{	for(int x = 1; x < N; x++)
			{
				const Point2f& fxy = flow.at<Point2f>(y, x);
				flSolver->addHorzVelocityAt(x, y, FLOW_SCALE * fxy.x);
				flSolver->addVertVelocityAt(x, y, FLOW_SCALE * fxy.y);
			}
		}
	}
	#if DEBUG 
		Mat cflow;
		cvtColor(prevFlowImg, cflow, CV_GRAY2BGR);
		drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
		imshow("flow", cflow);
	#endif
	std::swap(prevFlowImg, currFlowImg);
}


/**
 * Creates an emitter object with given properties.
 */
static void createEmitterAt(int center_x, int center_y, float force_u, float force_v, int lifespan, int radius, int userID = 1)
{
	Emitter newEmit = {Point(center_x, center_y), Point2f(force_u, force_v), lifespan, 0, radius, userID};
	emittersList.push_back(newEmit);
	#if DEBUG
		cout<<"Emitter created: "<<emittersList.size()<<endl;
	#endif
}

/**
 * Iterates through Emitters vector and adds forces to fluid simulation for each emitter.
 * Also kills emitters that have expired.
 *
 * @param e		- Reference to a vector containing Emitters to render.
 */
static void renderEmitters(FluidSolver* flSolver, vector<Emitter> &e)
{
	//TODO: convert emitters to fixed array. Bug being caught in loop right now.
	int i = 0;
	while (i < e.size())
	{
		bool emitterHasExpired = (e[i].lifespan - e[i].lifeElapsed == 0);
		if(emitterHasExpired) 
			e.erase(e.begin() + i);
		else 
		{
			if (useFlow)
			{
				Point lowerCoord, upperCoord;

				//calculate scalar for temporal falloff overlifespan
				float lifescalar = (e[i].lifespan - e[i].lifeElapsed) / e[i].lifespan;

				//prevent radius from referencing cells outside simulation matrix
				//TODO: clean up line breaks here
				lowerCoord.y = (e[i].center.y - e[i].radius) < 1 ? 1 : (e[i].center.y - e[i].radius);
				lowerCoord.x = (e[i].center.x - e[i].radius) < 1 ? 1 : (e[i].center.x - e[i].radius);
				upperCoord.y = (e[i].center.y + e[i].radius) > N ? N : (e[i].center.y + e[i].radius);
				upperCoord.x = (e[i].center.x + e[i].radius) > N ? N : (e[i].center.x + e[i].radius);

				for(int y = lowerCoord.y; y <= upperCoord.y; y++)
					for(int x = lowerCoord.x; x <= upperCoord.x; x++) 
					{
						//calculate falloff from center
						float vscalar = fabs(y - e[i].center.y) / e[i].radius;
						float uscalar = fabs(x - e[i].center.x) / e[i].radius;
						float dscalar = (vscalar+uscalar) / 2;
					
						float horzVel = e[i].vel.x * uscalar;
						float vertVel = e[i].vel.y * vscalar;
						float density = source * dscalar * lifescalar;

						flSolver->addHorzVelocityAt(x, y, horzVel);
						flSolver->addVertVelocityAt(x, y, vertVel);

						if(useUserSolver) 
							userSolver->addDensityAt(e[i].userID, x, y, density);
						else
							flSolver->addDensityAt(x, y, density);
					}
				e[i].lifeElapsed++;
				i++;
			}
			else
			{
				float fu, fv = 0.0;
				// emit splashes on either side of whole silhouette
				for (int j = 1; j <= N; j++)
				{
					for (int i = 1; i <= N; i++)
					{
						bool horzBoundChangesToYes = !flSolver->isBoundAt(i, j) && flSolver->isBoundAt(i+1, j);
						bool horzBoundChangesToNo  = flSolver->isBoundAt(i, j) && !flSolver->isBoundAt(i+1, j);

						if(horzBoundChangesToYes)
						{
							fu = -0.05; // emit velocity in negative direction
							fv = 0.1;
							flSolver->addHorzVelocityAt(i, j, force * fu);
							flSolver->addVertVelocityAt(i, j, force * fv);
						}
						else if(horzBoundChangesToNo)
						{ 
							fu = 0.05; // emit velocity in positive direction
							fv = 0.1;
							flSolver->addHorzVelocityAt(i+1, j, force * fu);
							flSolver->addVertVelocityAt(i+1, j, force * fv);
						}
					}
				}
			}
		}
	}
}


/**
 * Creates emitter objects based on optical flow velocity. 
 * If vertical velocity is negative at boundaries, an emitter 
 * is created. An emission threshold prevents negative velocities 
 * due to noise from creating emitters. In the call tree, we 
 * assume this function is called after computeOpticalFlow.
 *
 * @param flsolver	Fluid Solver to emit splashes into
 * @param flow		Reference to a matrix containing optical flow velocities.
 */
static void emitSplashes(FluidSolver* flSolver, Mat &flow)
{
	//precondition: optical flow has been calculated
	float fu, fv;
	fu = fv = 0.0;
	int velocityEmissionThreshold = -0.05; //creates emitters based on velocity emission

	if(useFlow)
	{
		// Only look for emitters in splash rows.
		for( int j = 1; j < SPLASH_ROWS; j++) 
		{	for(int i = 1; i <= N; i++) 
			{													
				bool vertBoundChangesToYes = !flSolver->isBoundAt(i, j) && flSolver->isBoundAt(i, j+1);
				if(vertBoundChangesToYes) 
				{ 
					const Point2f& opticalFlowVelocity = flow.at<Point2f>(i, j);
					fu = .8 *  opticalFlowVelocity.x;
					fv = .8 *  opticalFlowVelocity.y;

					if(opticalFlowVelocity.y < velocityEmissionThreshold) 
					{
						if(useUserSolver)
						{
							int userID = resizedUsersMatrix.at<uchar>(i, j+1);
							createEmitterAt(i, j-1, fu, fv, 6, 3, userID);
						}
						else
							createEmitterAt(i, j-1, fu, fv, 6, 3, 1);
					}
				}
			}
		}
	}
	renderEmitters(flSolver, emittersList);
}

/**
 * Changes various modes. Modes are given integer numbers to work with auto switcher function.
 *
 * @param newMode	Mode number to change to. Valid values 0-3.
 */
static void changeMode(int newMode)
{
	mode = newMode;
	//we have changed modes, initialize new modes
	clearData();
	switch(newMode)
	{
		case 0:
			source  = 20.0f;
			dvel    = false;
			dbound  = false;
			dusers  = false;
			useFlow = true;
			useUserSolver      = true;
			useWhiteBackground = false;
			cout<<"Changing to mode 0: Single color density"<<endl;
			break;
		case 1:
			source  = 20.0f;
			dvel    = true;
			dbound  = false;
			dusers  = false;
			useFlow = false;
			useUserSolver      = false;
			useWhiteBackground = false;
			cout<<"Changing to mode 1: Vectors without optical flow"<<endl;
			break;
		case 2:
			source  = 20.0f;
			dvel    = false;
			dbound  = false;
			dusers  = true;
			useFlow = true;
			useUserSolver      = true;
			useWhiteBackground = false;
			cout<<"Changing to mode 2: Multi-color user emission"<<endl;
			break;
		case 3:
			source  = 20.0f;
			dvel    = false;
			dbound  = false;
			dusers  = false;
			useFlow = true;
			useUserSolver      = true;
			useWhiteBackground = true;
			cout<<"Changing to mode 3: White background"<<endl;
			break;
	}
}



/**
 * Tries to change the mode if iterations have reached iterationsPerMode.
 */
static void tryChangeMode()
{
	if(autoChangeMode && (iterations > iterationsPerMode))
	{
		iterations = 0;
		mode++;
		if(mode > maxMode)
			mode = 0;

		changeMode(mode);
	}
	else
		iterations++;
}


////////////////////////////////////////////////////////////////////////
/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

/**
 * Draws fluid velocity vectors in OpenGL.
 * @param flSolver	FluidSolver containing the velocity to draw.
 *
 */
static void drawVelocity(FluidSolver* flSolver)
{
	int i, j;
	float x, y, h;
	h = 1.0f / N;
	glColor3f(1.0f, 1.0f, 1.0f);
	glLineWidth(1.0f);

	glBegin(GL_LINES);
		for (i = 1; i <= N; i++) 
		{
			x = (i - 0.5f) * h;
			for (j = 1; j <= N; j++)
			{
				y = (j - 0.5f) * h;
				glVertex2f(x, y);
				glVertex2f(x + flSolver->getHorzVelocityAt(i,j), 
					       y + flSolver->getVertVelocityAt(i,j));
			}
		}
	glEnd();
}


/**
 * Draws bounding cells in OpenGL.
 * @param flSolver	FluidSolver containing the bounds to draw.
 *
 */
static void drawBounds(FluidSolver* flSolver)
{
	int i, j;
	float x, y, h;
	h = 1.0f / N; //calculate unit length of each cell

	glBegin(GL_QUADS);
		for (i = 0; i <= N; i++) 
		{
			x = i * h;
			for (j = 0; j <= N; j++) 
			{
				y = j * h;

				if(flSolver->isBoundAt(i,j)) {
					glColor3f (0.30f, 0.30f, 0.30f); glVertex2f (x,   y);
					glColor3f (0.30f, 0.30f, 0.30f); glVertex2f (x+h, y);
					glColor3f (0.30f, 0.30f, 0.30f); glVertex2f (x+h, y+h);
					glColor3f (0.30f, 0.30f, 0.30f); glVertex2f (x,   y+h);
				}
			}
		}
	glEnd();
}


typedef struct {float R, G, B;} RGBType;
typedef struct {float H, S, V;} HSVType;

#define RETURN_RGB(r, g, b) {RGB.R = r; RGB.G = g; RGB.B = b; return RGB;}
#define RETURN_HSV(h, s, v) {HSV.H = h; HSV.S = s; HSV.V = v; return HSV;}
#define UNDEFINED -1

/**
 *  Converts an HSV color into RGB color space. 
 *
 *  @param HSV	HSV color variable to convert into RGB.
 */
RGBType HSV_to_RGB( HSVType HSV ) 
{
	// H is given on [0, 6] or UNDEFINED. S and V are given on [0, 1].
	// RGB are each returned on [0, 1].
	float h = HSV.H, s = HSV.S, v = HSV.V, m, n, f;
	int i;
	RGBType RGB;
	if (h == UNDEFINED) 
		RETURN_RGB(v, v, v);
	i = floor(h);
	f = h - i;
	if ( !(i&1) ) 
		f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);
	switch (i) 
	{
		case 6 :
		case 0 : RETURN_RGB(v, n, m);
		case 1 : RETURN_RGB(n, v, m);
		case 2 : RETURN_RGB(m, v, n)
		case 3 : RETURN_RGB(m, n, v);
		case 4 : RETURN_RGB(n, m, v);
		case 5 : RETURN_RGB(v, m, n);
	}
}

/**
 *  Computes the weighted color of all the user densities at a particular 
 *  pixel coordinate.
 *
 *  @param x	x-coordinate 
 *  @param y	y-coordinate
 */
RGBType getWeightedColor(int x, int y)
{
	RGBType RGB;
	float R, G, B;
	R=G=B=0;

	for(int i = 0; i < MAX_USERS; i++) 
	{
		float densityVal = userSolver->getDensityAt(i, x, y);		
		if(useWhiteBackground) 
		{
			R += colorsWhiteBG[i][0] * densityVal;
			G += colorsWhiteBG[i][1] * densityVal;
			B += colorsWhiteBG[i][2] * densityVal;
		}
		else
		{
			R += colors[i][0] * densityVal;
			G += colors[i][1] * densityVal;
			B += colors[i][2] * densityVal;
		}
	}
	RGB.R = R; RGB.B = B; RGB.G = G; 
	return RGB;
}



/**
 *  Render density grids as OpenGL quads with vertex shading.
 *
 *  @param flSolver	fluid solver 
 */
static void drawDensity ( FluidSolver* flSolver )
{
	int i, j;
	float x, y, h;
	RGBType rgb00, rgb10, rgb11, rgb01;
	float	d00,   d10,   d11,   d01;
	float hue = 3.25;
	float sat = 1.0;
	h = 1.0f/N;
	
	glBegin ( GL_QUADS );
		for ( i=1 ; i<=N ; i++ ) 
		{
			x = (i-0.5f)*h;
			for ( j=1 ; j<=N ; j++ ) 
			{
				y = (j-0.5f)*h;
				if(useUserSolver)
				{
					//render density color for each point based on blending user values
					rgb00 = getWeightedColor(i,j);
					rgb10 = getWeightedColor(i+1,j);
					rgb11 = getWeightedColor(i+1,j+1);
					rgb01 = getWeightedColor(i,j+1);
				}
				else 
				{
					//if a cell is a bounds cell, do not apply a density color
					d00 = flSolver->isBoundAt(i,j)     ? 0 : BG_OFFSET + flSolver->getDensityAt(i,j);
					d10 = flSolver->isBoundAt(i+1,j)   ? 0 : BG_OFFSET + flSolver->getDensityAt(i+1,j);
					d11 = flSolver->isBoundAt(i+1,j+1) ? 0 : BG_OFFSET + flSolver->getDensityAt(i+1,j+1); 
					d01 = flSolver->isBoundAt(i,j+1)   ? 0 : BG_OFFSET + flSolver->getDensityAt(i,j+1); 

					//hsv to rgb using the density in the cell
					HSVType hsv00 = {hue, sat, d00};
					rgb00 = HSV_to_RGB(hsv00);
					HSVType hsv10 = {hue, sat, d10};
					rgb10 = HSV_to_RGB(hsv10);
					HSVType hsv11 = {hue, sat, d11};
					rgb11 = HSV_to_RGB(hsv11);
					HSVType hsv01 = {hue, sat, d01};
					rgb01 = HSV_to_RGB(hsv01);
				}

				glColor3f (rgb00.R, rgb00.G, rgb00.B); glVertex2f ( x,   y   );
				glColor3f (rgb10.R, rgb10.G, rgb10.B); glVertex2f ( x+h, y   );
				glColor3f (rgb11.R, rgb11.G, rgb11.B); glVertex2f ( x+h, y+h );
				glColor3f (rgb01.R, rgb01.G, rgb01.B); glVertex2f ( x,   y+h );
			}
		}
	glEnd ();
}



/**
 *  Draws user silhouettes in unique colors per user in OpenGL. 
 *  Uses the usersMatrix from kinect.
 *
 */
static void drawUsers(void)
{
	int i, j;
	float x, y, h;
	int d00, index;
	h = 1.0f/N;

	glBegin ( GL_QUADS );
		for ( i=0 ; i<=N ; i++ ) 
		{
			x = i*h;
			for ( j=0 ; j<=N ; j++ ) 
			{
				y = j*h;
				index = j * N + i;
				d00 = resizedUsersMatrix.data[index];					
				if(d00 != 0) 
				{
					GLfloat R = colors[d00][0];
					GLfloat G = colors[d00][1];
					GLfloat B = colors[d00][2];

					//cout<<"found a color for user: "<<d00<<": ("<<R<<", "<<G<<", "<<B<<")"<<endl;
					glColor3f ( R, G, B ); glVertex2f ( x, y );
					glColor3f ( R, G, B ); glVertex2f ( x+h, y );
					glColor3f ( R, G, B ); glVertex2f ( x+h, y+h );
					glColor3f ( R, G, B ); glVertex2f ( x, y+h );
				}
			}
		}
	glEnd ();

}
////////////////////////////////////////////////////////////////////////

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

/**
 *  GLUT keyboard listener function.
 *
 *  @param key	key that is pressed
 *  @param x	x-coordinate of mouse at time of key press
 *  @param y	y-coordinate of mouse at time of keey press
 */
static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 'c':
		case 'C':
			clearData();
			break;
		case 27 : //escape key
			cleanupExit();
			break;
		case 'f':
		case 'F':
			//toggle optical flow
			useFlow = !useFlow;
			cout<<"Optical Flow: "<<useFlow<<endl;
			break;
		case 'v':
		case 'V':
			dvel = !dvel;
			cout<<"Display Velocity"<<endl;
			break;
		case 'b':
		case 'B':
			dbound = !dbound;
			break;
		case '1': //single color fluid
			changeMode(0);
			break;
		case '2': //vectors without optical flow
			changeMode(1);
			break;
		case '3': //multi color fluid
			changeMode(2);
			break;
		case '4': //white bg
			changeMode(3);
			break;
		case '0': //toggle auto mode change
			autoChangeMode = !autoChangeMode;
			cout<<"Auto Change Mode: "<<autoChangeMode<<endl;
			break;
		case 'u':
		case 'U':
			dusers = !dusers;
			cout<<"Draw Users: "<<dusers<<endl;
			break;
		case '+':
			kinect->reset();
			break;
		case 'o':
		case 'O':
			kinect->setDepth(+200);
			break;
		case 'k':
		case 'K':
			kinect->setDepth (-200);
			break;
		case 'Q' :
		case 'q' :
			toggleFullscreen();
			break;
	}
}



/**
 *  GLUT mouse listener function. Called when mouse button is pressed or 
 *  released.
 *
 *  @param button ID of button event that is presssed.
 *  @param state  A GLUT constant indicating mouse down or up
 *  @param x	  x-coordinate of mouse at time of button event
 *  @param y	  y-coordinate of mouse at time of button event
 */
static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;
	mouseDown[button] = state == GLUT_DOWN;
}



/**
 *  GLUT mouse movement function. Called when mouse is moved
 *
 *  @param x	  x-coordinate of mouse 
 *  @param y	  y-coordinate of mouse
 */
static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}



/**
 *  GLUT window reshaping function. Called whenever a window is resized.
 *
 *  @param width	New width of window
 *  @param height	New height of window.
 */
static void reshape_func ( int width, int height )
{
	glutReshapeWindow ( width, height );
	winX = width;
	winY = height;
}


/**
 *  Called when OpenGL is not drawing. Calls for another draw frame
 */
static void idle_func ( void )
{
	bool fullscreen = glutGameModeGet(GLUT_GAME_MODE_ACTIVE);
	if(!fullscreen)   glutSetWindow(winID);
	glutPostRedisplay();
}



/**
 *  Prepares OpenGL canvas to draw a new frame.
 */
static void pre_display ( void )
{		
//	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glViewport ( 0, 0, winX, winY );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}



/**
 *  Posts the drawn frame to the screen.
 */
static void post_display ( void )
{
	glutSwapBuffers ();
}
////////////////////////////////////////////////////////////////////////////

/**
 *  Draws OpenGL polygons that represent the fluid simulation. Also calles
 *  methods that trigger simulation steps. Most of the behavior of the 
 *  program is defined in this method. 
 *
 */
static void drawFunction ( void )
{
	FluidSolver* flSolver;
	bool dispUsr = useUserSolver && dusers;

	if(useUserSolver)	flSolver = userSolver;
	else				flSolver = solver;

	tryChangeMode();
	pre_display();
		loadKinectData();

		defineBoundsFromImage          (flSolver, depthImage);
		getForcesFromMouse             (flSolver);
		if(useFlow)  computeOpticalFlow(flSolver, flow);
		emitSplashes                   (flSolver, flow);

		if(useUserSolver)	userSolver->update();
		else				solver->update();

		if(dvel)     		drawVelocity(flSolver);
		else				drawDensity(flSolver);

		if(dbound)			drawBounds(flSolver);
		if(dispUsr)			drawUsers();
	post_display();
}


/**
 *  Sets OpenGL Callbacks
 *
 */
static void initOpenGl() 
{
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	pre_display ();

	glutKeyboardFunc (key_func     );
	glutMouseFunc    (mouse_func   );
	glutMotionFunc   (motion_func  );
	glutReshapeFunc  (reshape_func );
	glutIdleFunc     (idle_func    );
	glutDisplayFunc  (drawFunction );
}


/**
 * Switches between fullscreen and windowed mode.
 */
static void toggleFullscreen()
{
	bool fullscreen = glutGameModeGet(GLUT_GAME_MODE_ACTIVE);
	if(fullscreen) 
	{
		winX = winY = DEF_WINDOW_SIZE;
		glutLeaveGameMode();
		openGlutWindow();
	}
	else 
	{
		glutGameModeString("640x480:16@60");
		if (glutGameModeGet(GLUT_GAME_MODE_POSSIBLE)) 
		{
			glutEnterGameMode();
			//glutDestroyWindow(winID);
			winX = glutGameModeGet(GLUT_GAME_MODE_WIDTH);
			winY = glutGameModeGet(GLUT_GAME_MODE_HEIGHT);
			initOpenGl();
		}
		else 
			printf("The select fullscreen mode is not available\n");
	}
}



/**
 *  Opens a GLUT compatible window and sets OpenGL callbacks.
 *
 */
static void openGlutWindow ( void )
{
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( winX, winY );

	char* windowName;
	sprintf(windowName, "Fluid Wall %s", VERSION);	
	winID = glutCreateWindow(windowName);

	//register callbacks
	initOpenGl();
}



//////////////////////////////////////////////////////////////////////////
/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/
int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv);
	if ( argc != 1 && argc != 6 ) 
	{
		fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
		fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
		exit ( 1 );
	}

	printf ( "\n\n ==== Fluid Wall %s ==== \n", VERSION);
	printf ( " SIMULATION:\n");
	printf ( "\t Add densities with the right mouse button\n" );
	printf ( "\t Add bounds with the middle mouse button\n" );
	printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
	printf ( "\t Toggle use of optical flow with the 'f' key.\n" );
	printf ( "\t Clear the simulation with the 'c' key\n" );
	printf ( " DISPLAY:\n");
	printf ( "\t Toggle fullscreen mode with the 'q' key.\n" );
	printf ( "\t Toggle density/velocity display with the 'v' key.\n" );
	printf ( "\t Toggle bounds display with the 'b' key.\n" );
	printf ( "\t Toggle users display with the 'u' key.\n" );
	printf ( " MODES:\n");
	printf ( "\t '0' key: Toggle Automatic Mode Change.\n" );
	printf ( "\t '1' key: Switch to mode 1: Single user, blue fluid.\n" );
	printf ( "\t '2' key: Switch to mode 2: Velocity Vector, no optical flow.\n" );
	printf ( "\t '3' key: Switch to mode 3: Multi-user, multicolor fluid.\n" );
	printf ( "\t '4' key: Switch to mode 4: Multi-user, white background.\n" );
	printf ( " KINECT:\n");
	printf ( "\t Increase Kinect depth thrshold angle with the 'o' key.\n" );
	printf ( "\t Decrease Kinect depth thrshold angle with the 'k' key.\n" );
	printf ( "\t Reset the Kinect with the + key \n\n" );
	printf ( " Quit with the 'ESC' key.\n" );

	dvel = false;
	dusers = false;
	dbound = false;

	if ( !allocateData() ) 
		exit ( 1 );
	
	clearData();

	winX = DEF_WINDOW_SIZE;
	winY = DEF_WINDOW_SIZE;

	openGlutWindow();
	glutMainLoop();
}