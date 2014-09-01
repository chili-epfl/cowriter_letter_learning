/*
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.shape_learner;

import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;

import org.ros.address.InetAddressFactory;
import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageButton;


/**
 * The main Activity for the shape_learning app, which starts the relevant ROS nodes and configures
 * various callback methods.
 * @author deanna.m.hood@gmail.com (Deanna Hood).
 */


public class MainActivity extends RosActivity {
    private InteractionManager interactionManager;
    private static final java.lang.String TAG = "shapeLearner";
    private DisplayManager displayManager;
    private UserDrawingView userDrawingsView;
    private UserDrawingView userGestureView;
    private Button buttonClear;
    private ImageButton buttonSend;
    private ArrayList< ArrayList<double[]> > userDrawnMessage = new ArrayList<ArrayList<double[]>>();
    private GestureDetector gestureDetector;
    private boolean longClicked = true;
    private int timeBetweenWatchdogClears_ms = 100;
    private boolean replayingUserShapes = false;
    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("Shape learner", "Shape learner");
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE); //remove title bar with app's icon and name
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
        WindowManager.LayoutParams.FLAG_FULLSCREEN); //remove bar with notifications and battery level etc
        Log.e(TAG,"Should be fullscreen now");

        setContentView(R.layout.main);
        buttonClear = (Button)findViewById(R.id.buttonClear);
        buttonClear.setOnClickListener(clearListener); // Register the onClick listener with the implementation below
        buttonSend = (ImageButton)findViewById(R.id.buttonSend);
        buttonSend.setOnClickListener(sendListener); // Register the onClick listener with the implementation below

        startWatchdogClearer();

        //for collecting user demonstrations (with stylus)
        userDrawingsView = (UserDrawingView)findViewById(R.id.signature);
        userDrawingsView.setRespondToFinger(false);
        userDrawingsView.setRespondToStylus(true);
        userDrawingsView.setStylusStrokeFinishedCallable(new MessageCallable<Integer, ArrayList<double[]>>() {
            @Override
            public Integer call(ArrayList<double[]> message) {
                onStylusStrokeDrawingFinished(message);
                return 1;
            }
        });

        //for collecting user gestures (with fingertip)
        userGestureView = (UserDrawingView)findViewById(R.id.gestureView);
        userGestureView.setRespondToFinger(true);
        userGestureView.setRespondToStylus(false);
        userGestureView.setColor(Color.RED);
        userGestureView.setFingerStrokeFinishedCallable(new MessageCallable<Integer, ArrayList<double[]>>() {
            @Override
            public Integer call(ArrayList<double[]> message) {
                onFingerStrokeDrawingFinished(message);
                return 1;
            }
        });

        displayManager = (DisplayManager) findViewById(R.id.image);
        displayManager.setTopicName("write_traj");
        displayManager.setMessageType(nav_msgs.Path._TYPE);


        displayManager.setClearScreenCallable(new MessageCallable<Integer, Integer>() {
            @Override
            public Integer call(Integer message) {
                onClearScreen();
                return 1;
            }
        });

        gestureDetector = new GestureDetector(this, new GestureDetector.SimpleOnGestureListener() {
            @Override
            public void onLongPress(MotionEvent e) {
                float x = e.getX();
                float y = e.getY();
                Log.e(TAG, "Double tap at: ["+String.valueOf(x)+", "+String.valueOf(y)+"]");
                //publish touch event in world coordinates instead of tablet coordinates
                interactionManager.publishGestureInfoMessage(DisplayMethods.PX2M(x), DisplayMethods.PX2M(DisplayMethods.getTabletResolution()[1] - y));
                longClicked = true;
            }
        });
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        interactionManager = new InteractionManager();
        interactionManager.setTouchInfoTopicName("touch_info");
        interactionManager.setGestureInfoTopicName("gesture_info");
        interactionManager.setClearScreenTopicName("clear_screen");
        displayManager.setClearScreenTopicName("clear_screen");
        displayManager.setClearWatchdogTopicName("watchdog_clear/tablet");
        displayManager.setFinishedShapeTopicName("shape_finished");
        interactionManager.setUserDrawnShapeTopicName("user_drawn_shapes");

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        // At this point, the user has already been prompted to either enter the URI
        // of a master to use or to start a master locally.
        nodeConfiguration.setMasterUri(getMasterUri());
        String hostIp = getMasterUri().getHost();
        Log.d(TAG, "Host's IP address: "+hostIp);

        NtpTimeProvider ntpTimeProvider = new NtpTimeProvider(InetAddressFactory.newFromHostString(hostIp),nodeMainExecutor.getScheduledExecutorService());
        ntpTimeProvider.startPeriodicUpdates(1, TimeUnit.MINUTES);
        nodeConfiguration.setTimeProvider(ntpTimeProvider);

        Log.e(TAG, "Ready to execute");
        if(replayingUserShapes){

            nodeMainExecutor.execute(displayManager, nodeConfiguration.setNodeName("android_gingerbread2/display_manager"));
            nodeMainExecutor.execute(interactionManager, nodeConfiguration.setNodeName("android_gingerbread2/interaction_manager"));
        }
        else{
            nodeMainExecutor.execute(displayManager, nodeConfiguration.setNodeName("android_gingerbread/display_manager"));
            nodeMainExecutor.execute(interactionManager, nodeConfiguration.setNodeName("android_gingerbread/interaction_manager"));
        }

        DisplayMethods displayMethods = new DisplayMethods();
        displayMethods.setOnAnimationFinishCallable(new MessageCallable<Integer, Integer>() {
            @Override
            public Integer call(Integer message) {
                onShapeDrawingFinish();
                return 1;
            }
        });

        displayManager.setMessageToDrawableCallable(displayMethods.getTurnPathIntoAnimation());
        displayMethods.setDisplayHeight(displayManager.getHeight());
        displayMethods.setDisplayWidth(displayManager.getWidth());
        displayMethods.setDisplayRate(displayManager.getDisplayRate()); //read value that node got from rosparam server
    }

    private void onStylusStrokeDrawingFinished(ArrayList<double[]> points){
        //convert from pixels in 'tablet frame' to metres in 'robot frame'
        for(double[] point : points){
            point[0] = DisplayMethods.PX2M(point[0]);                        //x coordinate
            point[1] = DisplayMethods.PX2M(DisplayMethods.getTabletResolution()[1] - point[1]); //y coordinate
        }
        Log.e(TAG, "Adding stroke to message");
        userDrawnMessage.add(points);
    }
    //When a finger-drawn stoke is finished in the SignatureView, publish its centre to the gesture topic
    private void onFingerStrokeDrawingFinished(ArrayList<double[]> points){
        if(points.size()>15){
            //convert from pixels in 'tablet frame' to metres in 'robot frame'
            double xMax = Double.NEGATIVE_INFINITY;
            double yMax = Double.NEGATIVE_INFINITY;
            double xMin = Double.POSITIVE_INFINITY;
            double yMin = Double.POSITIVE_INFINITY;
            for(double[] point : points){
                point[0] = DisplayMethods.PX2M(point[0]);                        //x coordinate
                point[1] = DisplayMethods.PX2M(DisplayMethods.getTabletResolution()[1] - point[1]); //y coordinate
                //update the max and min values of the stroke
                if(point[0]>xMax){
                    xMax = point[0];
                }
                if(point[1]>yMax){
                    yMax = point[1];
                }
                if(point[0]<xMin){
                    xMin = point[0];
                }
                if(point[1]<yMin){
                    yMin = point[1];
                }
            }
            double xRange = xMax - xMin;
            double yRange = yMax - yMin;
            double xCentre = xMax - xRange/2.0;
            double yCentre = yMax - yRange/2.0;
            Log.e(TAG, "Publishing finger stroke");
            interactionManager.publishGestureInfoMessage(xCentre, yCentre);
        }
        userGestureView.requestClear();
    }

    private void onClearScreen(){
        userDrawingsView.requestClear(); //clear display of user-drawn shapes
    }

    private View.OnClickListener sendListener = new View.OnClickListener() {
        public void onClick(View v) {
            Log.e(TAG, "onClick() called - send button");
            interactionManager.publishUserDrawnMessageMessage(userDrawnMessage);
            userDrawnMessage.clear(); //empty/reinitialise message
        }
    };

    private View.OnClickListener clearListener = new View.OnClickListener() {
        public void onClick(View v) {
            Log.e(TAG, "onClick() called - clear button");
            //interactionManager.publishClearScreenMessage();  //clear display of robot-drawn message
            userDrawnMessage.clear(); //empty/reinitialise message
            userDrawingsView.clear(); //clear display of user-drawn shapes (would have liked to have
                // done this with a callback upon receipt of clearScreenMessage, but that thread isn't allowed to 'touch' signatureView)
        }
    };

    private void onShapeDrawingFinish(){
        Log.e(TAG,"Animation finished!");
        displayManager.publishShapeFinishedMessage();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        gestureDetector.onTouchEvent(event);
        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                longClicked = false;
                break;
            case MotionEvent.ACTION_MOVE:
                break;
            case MotionEvent.ACTION_UP:
                if(!longClicked){
                    int x = (int)event.getX();
                    int y = (int)event.getY();
                    Log.e(TAG, "Touch at: ["+String.valueOf(x)+", "+String.valueOf(y)+"]");
                    //publish touch event in world coordinates instead of tablet coordinates
                    interactionManager.publishTouchInfoMessage(DisplayMethods.PX2M(x), DisplayMethods.PX2M(DisplayMethods.getTabletResolution()[1] - y));
                }
                break;
        }

        return true;
    }

    public void startWatchdogClearer() {
        final Handler handler = new Handler();
        Timer timer = new Timer();
        TimerTask clearWatchdog = new TimerTask() {
            @Override
            public void run() {
                handler.post(new Runnable() {
                    public void run() {
                        try {
                            displayManager.publishWatchdogClearMessage();
                        } catch (Exception e) {
                            // TODO Auto-generated catch block
                        }
                    }
                });
            }
        };
        timer.schedule(clearWatchdog, 0, timeBetweenWatchdogClears_ms);
    }

}





