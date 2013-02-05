/*****************************************************************************
*   VideoSource.mm
*   Example_MarkerBasedAR
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch2 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#import <ImageIO/ImageIO.h>
#import <CoreVideo/CoreVideo.h>
#import <UIKit/UIKit.h>

////////////////////////////////////////////////////////////////////
// File includes:
#import "VideoSource.h"

@implementation VideoSource

@synthesize captureSession;
@synthesize delegate;
@synthesize deviceInput;

#pragma mark - Memory management

- (id)init
{
    if ((self = [super init]))
    {
        AVCaptureSession * capSession = [[AVCaptureSession alloc] init];
        
        if ([capSession canSetSessionPreset:AVCaptureSessionPreset640x480])
        {
            [capSession setSessionPreset:AVCaptureSessionPreset640x480];
            NSLog(@"Set capture session preset AVCaptureSessionPreset640x480");
        }
        else if ([capSession canSetSessionPreset:AVCaptureSessionPresetLow])
        {
            [capSession setSessionPreset:AVCaptureSessionPresetLow];
            NSLog(@"Set capture session preset AVCaptureSessionPresetLow");
        }
        
        self.captureSession = capSession;
    }
    return self;
}

- (CameraCalibration) getCalibration
{
    // Returns calibratoin data for iPad 2
    // Todo: Add parameters for the rest
    return CameraCalibration(6.24860291e+02 * (640./352.), 6.24860291e+02 * (480./288.), 640 * 0.5f, 480 * 0.5f);
}

- (CGSize) getFrameSize
{
    if (![captureSession isRunning])
        NSLog(@"Capture session is not running, getFrameSize will return invalid valies");
    
    NSArray *ports = [deviceInput ports];
    AVCaptureInputPort *usePort = nil;
    for ( AVCaptureInputPort *port in ports )
    {
        if ( usePort == nil || [port.mediaType isEqualToString:AVMediaTypeVideo] )
        {
            usePort = port;
        }
    }
    
    if ( usePort == nil ) return CGSizeZero;
    
    CMFormatDescriptionRef format = [usePort formatDescription];
    CMVideoDimensions dim = CMVideoFormatDescriptionGetDimensions(format);
    
    CGSize cameraSize = CGSizeMake(dim.width, dim.height);
    
    return cameraSize;
}

- (void)dealloc
{
    [self.captureSession stopRunning];
    
    self.captureSession = nil;
}

#pragma mark Capture Session Configuration

- (AVCaptureDevice *) cameraWithPosition:(AVCaptureDevicePosition) position
{
    NSArray *devices = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
    for (AVCaptureDevice *device in devices)
    {
        if ([device position] == position)
        {
            return device;
        }
    }
    return nil;
}

- (void) addRawViewOutput
{
	/*We setupt the output*/
	AVCaptureVideoDataOutput *captureOutput = [[AVCaptureVideoDataOutput alloc] init];
	
    /*While a frame is processes in -captureOutput:didOutputSampleBuffer:fromConnection: delegate methods no other frames are added in the queue.
	 If you don't want this behaviour set the property to NO */
	captureOutput.alwaysDiscardsLateVideoFrames = YES;
	
    /*We specify a minimum duration for each frame (play with this settings to avoid having too many frames waiting
	 in the queue because it can cause memory issues). It is similar to the inverse of the maximum framerate.
	 In this example we set a min frame duration of 1/10 seconds so a maximum framerate of 10fps. We say that
	 we are not able to process more than 10 frames per second.*/
	//captureOutput.minFrameDuration = CMTimeMake(1, 10);
    
	/*We create a serial queue to handle the processing of our frames*/
	dispatch_queue_t queue;
	queue = dispatch_queue_create("com.Example_MarkerBasedAR.cameraQueue", NULL);
	[captureOutput setSampleBufferDelegate:self queue:queue];
	dispatch_release(queue);
    
	// Set the video output to store frame in BGRA (It is supposed to be faster)
	NSString* key = (NSString*)kCVPixelBufferPixelFormatTypeKey;
	NSNumber* value = [NSNumber numberWithUnsignedInt:kCVPixelFormatType_32BGRA];
	NSDictionary* videoSettings = [NSDictionary dictionaryWithObject:value forKey:key];
	[captureOutput setVideoSettings:videoSettings];
    
    // Register an output
	[self.captureSession addOutput:captureOutput];
}

- (bool) startWithDevicePosition:(AVCaptureDevicePosition)devicePosition
{
    AVCaptureDevice *videoDevice = [self cameraWithPosition:devicePosition];
    
    if (!videoDevice)
        return FALSE;
    
    {
        NSError *error;
        
        AVCaptureDeviceInput *videoIn = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
        self.deviceInput = videoIn;
        
        if (!error)
        {
            if ([[self captureSession] canAddInput:videoIn])
            {
                [[self captureSession] addInput:videoIn];
            }
            else
            {
                NSLog(@"Couldn't add video input");
                return FALSE;
            }
        }
        else
        {
            NSLog(@"Couldn't create video input");
            return FALSE;
        }
    }
    
    [self addRawViewOutput];
    [captureSession startRunning];
    return TRUE;
}

#pragma mark -
#pragma mark AVCaptureSession delegate
- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    
    /*Lock the image buffer*/
    CVPixelBufferLockBaseAddress(imageBuffer,0);
    
    /*Get information about the image*/
    uint8_t *baseAddress = (uint8_t *)CVPixelBufferGetBaseAddress(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    size_t stride = CVPixelBufferGetBytesPerRow(imageBuffer);
    
    BGRAVideoFrame frame = {width, height, stride, baseAddress};
    [delegate frameReady:frame];
    
	/*We unlock the  image buffer*/
	CVPixelBufferUnlockBaseAddress(imageBuffer,0);
} 

@end