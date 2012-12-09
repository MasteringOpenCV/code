/*****************************************************************************
 *  Cartoonifier, for Desktop or Android.
 *  by Shervin Emami, 2012 (shervin.emami@gmail.com)
 *  http://www.shervinemami.info/
 *  Ch1 of the book "Mastering OpenCV with Practical Computer Vision Projects"
 *  Copyright Packt Publishing 2012.
 *****************************************************************************/

package com.Cartoonifier;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.Window;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.MotionEvent;

public class CartoonifierApp extends Activity implements OnTouchListener {
    private static final String TAG = "CartoonifierApp";
    private CartoonifierView mView;

    // Items for the Android menubar:
    private MenuItem mMenuSketch;
    private MenuItem mMenuAlien;
    private MenuItem mMenuEvil;
    private MenuItem mMenuDebug;

    public CartoonifierApp() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    @Override
    protected void onPause() {
        Log.i(TAG, "onPause");
        super.onPause();
        mView.releaseCamera();
    }

    @Override
    protected void onResume() {
        Log.i(TAG, "onResume");
        super.onResume();
        if( !mView.openCamera() ) {
            AlertDialog ad = new AlertDialog.Builder(this).create();  
            ad.setCancelable(false); // This blocks the 'BACK' button  
            ad.setMessage("Fatal error: can't open camera!");  
            ad.setButton("OK", new DialogInterface.OnClickListener() {  
                public void onClick(DialogInterface dialog, int which) {  
                    dialog.dismiss();                      
                    finish();
                }  
            });  
            ad.show();
        }
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "onCreate");
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        mView = new CartoonifierView(this);
        setContentView(mView);

        // Call our "onTouch()" callback function whenever the user touches the screen.
        mView.setOnTouchListener(this);
    }

    /** Called when the menubar is being created by Android. */
    public boolean onCreateOptionsMenu(Menu menu) {
        Log.i(TAG, "onCreateOptionsMenu");
        mMenuSketch = menu.add("Sketch or Painting");
        mMenuAlien = menu.add("Alien or Human");
        mMenuEvil = menu.add("Evil or Good");
        mMenuDebug = menu.add("[Debug mode]");
        return true;
    }

    /** Called whenever the user pressed a menu item in the menubar. */
    public boolean onOptionsItemSelected(MenuItem item) {
        Log.i(TAG, "Menu Item selected: " + item);
        if (item == mMenuSketch)
            mView.toggleSketchMode();
        else if (item == mMenuAlien)
            mView.toggleAlienMode();
        else if (item == mMenuEvil)
            mView.toggleEvilMode();
        else if (item == mMenuDebug)
            mView.toggleDebugMode();
        return true;
    }
    
    /** Called whenever the user touches the touchscreen */
//    @Override
    public boolean onTouch(View v, MotionEvent m) {
        // Ignore finger movement event, we just care about when the finger first touches the screen.
        if (m.getAction() != MotionEvent.ACTION_DOWN) {
            return false;   // We didn't do anything with this touch movement event.
        }
        
        Log.i(TAG, "onTouch down event");
        
        // Signal that we should cartoonify the next camera frame and save it, instead of just showing the sketch.
        mView.nextFrameShouldBeSaved(getBaseContext());

        return false;
    }
}
