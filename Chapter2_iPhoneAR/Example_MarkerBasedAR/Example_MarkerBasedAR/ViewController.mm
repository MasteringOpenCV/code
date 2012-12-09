/*****************************************************************************
*   ViewController.mm
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

// File includes:
#import "ViewController.h"

@implementation ViewController
@synthesize m_glview;

#pragma mark - View lifecycle

- (void)viewDidLoad
{
  [super viewDidLoad];
  
    // Do any additional setup after loading the view, typically from a nib.
  m_videoSource = [[VideoSource alloc]init];
  m_videoSource.delegate = self;
  
  m_pipeline = createMarkerDetection([m_videoSource getCalibration]);
  [m_videoSource startWithDevicePosition:AVCaptureDevicePositionBack];
}

- (void)viewDidUnload
{
  [self setM_glview:nil];
  [super viewDidUnload];
  // Release any retained subviews of the main view.
  // e.g. self.myOutlet = nil;
}

- (void)viewWillAppear:(BOOL)animated
{
  [m_glview initContext];
  
  CGSize frameSize = [m_videoSource getFrameSize];
  m_renderer = createVisualizationController(m_glview, [m_videoSource getCalibration], frameSize);  

  [super viewWillAppear:animated];
}

- (BOOL)shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
  return interfaceOrientation == UIInterfaceOrientationLandscapeRight;
  //return interfaceOrientation == UIInterfaceOrientationPortrait;
}

#pragma mark - VideoSourceDelegate
-(void)frameReady:(BGRAVideoFrame) frame
{
  // Start upload new frame to video memory in main thread
  dispatch_async( dispatch_get_main_queue(), ^{ 
    [m_renderer updateBackground:frame];
  });
  
  // And perform processing in current thread
  m_pipeline->processFrame(frame);
  
  // When it's done we query rendering from main thread
  dispatch_async( dispatch_get_main_queue(), ^{ 
    [m_renderer setTransformationList:(m_pipeline->getTransformations)()];
    [m_renderer drawFrame];
  });
}

@end
