/*****************************************************************************
*   VisualizationController.h
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
#import <Foundation/Foundation.h>
#import <vector>

////////////////////////////////////////////////////////////////////
// File includes:
#import "VideoSource.h"
#import "GeometryTypes.hpp"
#import "EAGLView.h"

@protocol VisualizationController <NSObject>
-(void) drawFrame;
-(void) updateBackground:(BGRAVideoFrame) frame;
-(void) setTransformationList:(const std::vector<Transformation>&) transformations;
@end

id<VisualizationController> createVisualizationController(EAGLView * view, CameraCalibration calibration, CGSize size);