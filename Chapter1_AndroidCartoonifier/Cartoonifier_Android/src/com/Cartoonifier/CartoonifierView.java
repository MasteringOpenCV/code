/*****************************************************************************
 *  Cartoonifier, for Desktop or Android.
 *  by Shervin Emami, 2012 (shervin.emami@gmail.com)
 *  http://www.shervinemami.info/
 *  Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
 *  Copyright Packt Publishing 2012.
 *****************************************************************************/

 package com.Cartoonifier;

import android.content.Context;
import android.graphics.Bitmap;

// For saving Bitmaps to file and the Android picture gallery.
import android.graphics.Bitmap.CompressFormat;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;
import android.provider.MediaStore.Images;
import android.text.format.DateFormat;
import android.util.Log;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;

// For showing a Notification message when saving a file.
import android.app.Notification;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.ContentValues;
import android.content.Intent;


class CartoonifierView extends CartoonifierViewBase {
    private static final String TAG = "CartoonifierView";
    
    private int mFrameSize;
    private Bitmap mBitmap;
    private int[] mRGBA;

    // Set to true if you want to see line drawings instead of paintings.
    private boolean m_sketchMode = false;
    // Set to true if you want to change the skin color of the character to an alien color.
    private boolean m_alienMode = false;
    // Set to true if you want an evil "bad" character instead of a "good" character.
    private boolean m_evilMode = false;
    // Set to true if you want to see many windows created, showing various debug info. Set to 0 otherwise.
    private boolean m_debugMode = false;
    
    // Whether to cartoonify the next camera frame and save it, instead of just showing the sketch.
    private boolean bSaveThisFrame = false;
    // After processing, don't update the screen for a while, so the user can see the cartoonifier output.
    private boolean bFreezeOutput = false;
    // Set the delay of showing the processed image before displaying the next camera frames.
    private static final int FREEZE_OUTPUT_MSECS = 3000;

    private Context mContext;  // Activity Context, so we can store to the Android Gallery.
    
    private int mNotificationID = 0;    // Notification ID, so we can show a status notification message whenever an image is saved.
    
    
    
    public CartoonifierView(Context context) {
        super(context);
    }

    @Override
    protected void onPreviewStarted(int previewWidth, int previewHeight) {
        mFrameSize = previewWidth * previewHeight;
        mRGBA = new int[mFrameSize];
        mBitmap = Bitmap.createBitmap(previewWidth, previewHeight, Bitmap.Config.ARGB_8888);
    }

    @Override
    protected void onPreviewStopped() {
        if(mBitmap != null) {
            mBitmap.recycle();
            mBitmap = null;
        }
        mRGBA = null;
    }

    // Show a notification message, saying that we've saved another image to the Gallery.
    protected void showNotificationMessage(Context context, String filename)
    {
        // Popup a notification message in the Android status bar. To make sure a notification
        // is shown for each image but only 1 is kept in the statusbar at a time, use a
        // different ID each time but delete previous messages before creating it.
        
        final NotificationManager mgr = (NotificationManager)context.getSystemService(Context.NOTIFICATION_SERVICE);
    
        // Close the previous popup message, so we only have 1 at a time, but it still shows a popup message for each one.
        if (mNotificationID > 0)
            mgr.cancel(mNotificationID);
        mNotificationID++;
    
        Notification notification = new Notification(R.drawable.icon, "Saving to gallery (image " + mNotificationID + ") ...", System.currentTimeMillis());
        Intent intent = new Intent(context, CartoonifierView.class);
        notification.flags |= Notification.FLAG_AUTO_CANCEL;    // Close it if the user clicks on it.
        PendingIntent pendingIntent = PendingIntent.getActivity(context, 0, intent, 0);
        notification.setLatestEventInfo(context, "Cartoonifier saved " + mNotificationID + " images to Gallery", "Saved as '" + filename + "'", pendingIntent);
        mgr.notify(mNotificationID, notification);
    }

    // Save the processed image as a PNG file on the SD card and shown in the Android Gallery.
    protected void savePNGImageToGallery(Bitmap bmp, Context context, String baseFilename)
    {
        try {
            // Get the file path to the SD card.
            String baseFolder = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES).getAbsolutePath() + "/";
            File file = new File(baseFolder + baseFilename);
            Log.i(TAG, "Saving the processed image to file [" + file.getAbsolutePath() + "]");
        
            // Open the file.
            OutputStream out = new BufferedOutputStream(new FileOutputStream(file));
            // Save the image file as PNG.
            bmp.compress(CompressFormat.PNG, 100, out);
            out.flush();    // Make sure it is saved to file soon, because we are about to add it to the Gallery.
            out.close();
        
            // Add the PNG file to the Android Gallery.
            ContentValues image = new ContentValues();
            image.put(Images.Media.TITLE, baseFilename);
            image.put(Images.Media.DISPLAY_NAME, baseFilename);
            image.put(Images.Media.DESCRIPTION, "Processed by the Cartoonifier App");
            image.put(Images.Media.DATE_TAKEN, System.currentTimeMillis()); // Milliseconds since 1970 UTC.
            image.put(Images.Media.MIME_TYPE, "image/png");
            image.put(Images.Media.ORIENTATION, 0);
            image.put(Images.Media.DATA, file.getAbsolutePath());
            Uri result = context.getContentResolver().insert(MediaStore.Images.Media.EXTERNAL_CONTENT_URI, image);
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    
    @Override
    protected Bitmap processFrame(byte[] data) {
        int[] rgba = mRGBA;

        // Only process the camera or update the screen if we aren’t supposed
        // to just show the cartoon image.
        if (bFreezeOutput) {
            // Only needs to be triggered here once.
            bFreezeOutput = false;

            // Wait for several seconds, doing nothing!
            try {
                wait(FREEZE_OUTPUT_MSECS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            return null;
        }

        String baseFilename = "";
        if (!bSaveThisFrame) {
            // Quick preview (show either "sketch" mode or unmodified camera preview).
            if (m_sketchMode) {
                // Process the image using C/C++ code through NDK (JNI).
                CartoonifyImage(getFrameWidth(), getFrameHeight(), data, rgba, m_sketchMode, m_alienMode, m_evilMode, m_debugMode);
            }
            else {
                // Just prepare the camera image for display without any modification. Uses C/C++ code through NDK (JNI).
                ShowPreview(getFrameWidth(), getFrameHeight(), data, rgba);
            }
        }
        else {
            // Just do it once.
            bSaveThisFrame = false;
            // Don't update the screen for a while, so the user can see the cartoonifier output.
            bFreezeOutput = true;

            // Generate the filename that we will store it as, so we can display a notification message while it is processing.
            // Get the current date & time
            SimpleDateFormat s = new SimpleDateFormat("yyyy-MM-dd,HH-mm-ss");
            String timestamp = s.format(new Date());
            baseFilename = "Cartoon" + timestamp + ".png";
            
            // Show a notification message, saying that we've saved another image to the Gallery.
            showNotificationMessage(mContext, baseFilename);

            // Process the image using C/C++ code through NDK (JNI).
            CartoonifyImage(getFrameWidth(), getFrameHeight(), data, rgba, m_sketchMode, m_alienMode, m_evilMode, m_debugMode);
        }
        
        // Put the processed image into the Bitmap object that will be returned for display on the screen.
        Bitmap bmp = mBitmap;
        bmp.setPixels(rgba, 0/* offset */, getFrameWidth() /* stride */, 0, 0, getFrameWidth(), getFrameHeight());
        
        if (bFreezeOutput) {
            // Save the processed image as a PNG file on the SD card and shown in the Android Gallery.
            savePNGImageToGallery(bmp, mContext, baseFilename);
        }
        
        return bmp;
    }
    
    protected void toggleSketchMode() {
        m_sketchMode = !m_sketchMode;
    }
    protected void toggleAlienMode() {
        m_alienMode = !m_alienMode;
    }
    protected void toggleEvilMode() {
        m_evilMode = !m_evilMode;
    }
    protected void toggleDebugMode() {
        m_debugMode = !m_debugMode;
    }

    // Cartoonify the next camera frame and save it, instead of just showing the sketch.
    protected void nextFrameShouldBeSaved(Context context) {
        bSaveThisFrame = true;
        mContext = context;
    }
    
    // Declare the function prototypes of the C/C++ code using NDK (JNI):

    // Just show the camera image, without any modification. Converts from YUV 420 planar to BGRA packed format.
    public native void ShowPreview(int width, int height, byte[] yuv, int[] rgba);

    // Modify the camera image using the Cartoonifier filter.
    public native void CartoonifyImage(int width, int height, byte[] yuv, int[] rgba, boolean sketchMode, boolean alienMode, boolean evilMode, boolean debugMode);
    
    // Load (dynamically at runtime) the C/C++ code in "libcartoonifier.so" using NDK (JNI).
    static {
        System.loadLibrary("cartoonifier");
    }
}
