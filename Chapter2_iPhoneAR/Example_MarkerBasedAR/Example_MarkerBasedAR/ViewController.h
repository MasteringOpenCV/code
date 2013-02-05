/*****************************************************************************
*   ViewController.h
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#import <UIKit/UIKit.h>

////////////////////////////////////////////////////////////////////
// File includes:
#import "EAGLView.h"

@interface ViewController : UIViewController
{
}

@property (weak, nonatomic) IBOutlet EAGLView *glview;
@end
