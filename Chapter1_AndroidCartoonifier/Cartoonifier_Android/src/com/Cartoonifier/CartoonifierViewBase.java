/*****************************************************************************
 *  Cartoonifier, for Desktop or Android.
 *  by Shervin Emami, 2012 (shervin.emami@gmail.com)
 *  http://www.shervinemami.info/
 *  Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
 *  Copyright Packt Publishing 2012.
 *****************************************************************************/

package com.Cartoonifier;

import java.io.IOException;
import java.util.List;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.os.Build;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

public abstract class CartoonifierViewBase extends SurfaceView implements SurfaceHolder.Callback, Runnable {
    private static final String TAG = "Cartoonifier::SurfaceView";

    private Camera              mCamera;
    private SurfaceHolder       mHolder;
    private int                 mFrameWidth;
    private int                 mFrameHeight;
    private byte[]              mFrame;
    private boolean             mThreadRun;
    private byte[]              mBuffer;

    // Signal that a camera frame is ready, without blocking the main thread.
    private boolean mCameraIsInitialized = false;

    
    public CartoonifierViewBase(Context context) {
        super(context);
        mHolder = getHolder();
        mHolder.addCallback(this);
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    public int getFrameWidth() {
        return mFrameWidth;
    }

    public int getFrameHeight() {
        return mFrameHeight;
    }

    public void setPreview() throws IOException {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
            mCamera.setPreviewTexture( new SurfaceTexture(10) );
        else
            mCamera.setPreviewDisplay(null);
    }

    public boolean openCamera() {
        Log.i(TAG, "openCamera");
        releaseCamera();
        mCamera = Camera.open();
        if(mCamera == null) {
            Log.e(TAG, "Can't open camera!");
            return false;
        }

        mCamera.setPreviewCallbackWithBuffer(new PreviewCallback() {
            public void onPreviewFrame(byte[] data, Camera camera) {
                // Whenever a camera preview frame is ready, just copy it straight to our mFrame,
                // and don't worry about blocking the main UI thread until it is safe.
                System.arraycopy(data, 0, mFrame, 0, data.length);
                camera.addCallbackBuffer(mBuffer);
                
                // Signal that a camera frame is ready, without blocking the main thread.
                mCameraIsInitialized = true;
            }
        });
        return true;
    }
    
    public void releaseCamera() {
        Log.i(TAG, "releaseCamera");
        mThreadRun = false;
        synchronized (this) {
            if (mCamera != null) {
                mCamera.stopPreview();
                mCamera.setPreviewCallback(null);
                mCamera.release();
                mCamera = null;
                
                // If this app was paused and restarted, it should wait for camera initialization again.
                mCameraIsInitialized = false;
            }
        }
        onPreviewStopped();
    }
    
    public void setupCamera(int width, int height) {
        Log.i(TAG, "setupCamera(" + width + "x" + height + ")");
        synchronized (this) {
            if (mCamera != null) {
                Camera.Parameters params = mCamera.getParameters();
                List<Camera.Size> sizes = params.getSupportedPreviewSizes();
                mFrameWidth = width;
                mFrameHeight = height;

                // selecting optimal camera preview size that is most similar to the screen height.
                {
                    int  minDiff = Integer.MAX_VALUE;
                    for (Camera.Size size : sizes) {
                        Log.i(TAG, "Found Camera Resolution " + size.width + "x" + size.height);
                        if (Math.abs(size.height - height) < minDiff) {
                            mFrameWidth = size.width;
                            mFrameHeight = size.height;
                            minDiff = Math.abs(size.height - height);
                        }
                    }
                }

                params.setPreviewSize(getFrameWidth(), getFrameHeight());
                
                List<String> FocusModes = params.getSupportedFocusModes();
                if (FocusModes.contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO))
                {
                    params.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
                }            
                
                mCamera.setParameters(params);
                
                /* Now allocate the buffer */
                params = mCamera.getParameters();
                Log.i(TAG, "Chosen Camera Preview Size: " + params.getPreviewSize().width + "x" + params.getPreviewSize().height);
                int size = params.getPreviewSize().width * params.getPreviewSize().height;
                size = size * ImageFormat.getBitsPerPixel(params.getPreviewFormat()) / 8;
                mBuffer = new byte[size];
                /* The buffer where the current frame will be copied */
                mFrame = new byte [size];
                mCamera.addCallbackBuffer(mBuffer);

                try {
                    setPreview();
                } catch (IOException e) {
                    Log.e(TAG, "mCamera.setPreviewDisplay/setPreviewTexture fails: " + e);
                }

                /* Notify that the preview is about to be started and deliver preview size */
                onPreviewStarted(params.getPreviewSize().width, params.getPreviewSize().height);

                /* Now we can start a preview */
                mCamera.startPreview();
            }
        }
    }
    
    public void surfaceChanged(SurfaceHolder _holder, int format, int width, int height) {
        Log.i(TAG, "surfaceChanged(). Window size: " + width + "x" + height);
        setupCamera(width, height);
    }

    public void surfaceCreated(SurfaceHolder holder) {
        Log.i(TAG, "surfaceCreated");
        (new Thread(this)).start();
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "surfaceDestroyed");
        releaseCamera();
    }


    /* The bitmap returned by this method shall be owned by the child and released in onPreviewStopped() */
    protected abstract Bitmap processFrame(byte[] data);

    /**
     * This method is called when the preview process is being started. It is called before the first frame delivered and processFrame is called
     * It is called with the width and height parameters of the preview process. It can be used to prepare the data needed during the frame processing.
     * @param previewWidth - the width of the preview frames that will be delivered via processFrame
     * @param previewHeight - the height of the preview frames that will be delivered via processFrame
     */
    protected abstract void onPreviewStarted(int previewWidtd, int previewHeight);

    /**
     * This method is called when preview is stopped. When this method is called the preview stopped and all the processing of frames already completed.
     * If the Bitmap object returned via processFrame is cached - it is a good time to recycle it.
     * Any other resources used during the preview can be released.
     */
    protected abstract void onPreviewStopped();

    public void run() {
        mThreadRun = true;
        Log.i(TAG, "Starting processing thread");

        // Wait until the first camera frame is ready.
        try {
            while (mThreadRun && !mCameraIsInitialized) {
                synchronized (this) {
                    wait(100);  // wait 100 milliseconds before trying again.
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        // Background processing loop.
        // Each iteration of this loop will process the latest camera image and render it to the
        // screen. Camera frames will be dropped, if it is not processed fast enough. The user
        // interface runs on the "main" thread, so intensive operations here won't delay the user
        // interface or cause "Application Not Responding".
        while (mThreadRun) {
            Bitmap bmp = null;

            // Process this frame.
            synchronized (this) {            
                bmp = processFrame(mFrame);
            }

            // Display this frame (on the "main" UI thread).
            if (bmp != null) {
                Canvas canvas = mHolder.lockCanvas();
                if (canvas != null) {
                    canvas.drawBitmap(bmp, (canvas.getWidth() - getFrameWidth()) / 2, (canvas.getHeight() - getFrameHeight()) / 2, null);
                    mHolder.unlockCanvasAndPost(canvas);
                }
            }
        }//end of background processing loop.

    }
}
