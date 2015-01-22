package org.ros.android.shape_learner;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

import java.util.ArrayList;

import nav_msgs.Path;
import std_msgs.Empty;
import std_msgs.Float32MultiArray;
import std_msgs.Float64MultiArray;

/**
 * Created by lemaigna on 20/01/15.
 */
public class BoxesViewNode extends View implements NodeMain {
    private static final java.lang.String TAG = "BoxesView";
    private Paint currentPaint;
    private ArrayList<RectF> boxes;

    private String topicName;
    private String clearScreenTopicName;

    public BoxesViewNode(Context context, AttributeSet attrs) {
        super(context, attrs);

        setLayerType(View.LAYER_TYPE_SOFTWARE, null);
        currentPaint = new Paint();
        currentPaint.setColor(Color.GRAY);
        currentPaint.setStyle(Paint.Style.STROKE);
        currentPaint.setStrokeJoin(Paint.Join.ROUND);
        currentPaint.setStrokeCap(Paint.Cap.ROUND);
        currentPaint.setStrokeWidth(10);

        boxes = new ArrayList<RectF>();
    }


    public void newBox(double left, double top, double right, double bottom) {
        boxes.add(new RectF((float)left, (float)top, (float)right, (float)bottom));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        for (RectF box : boxes) {
            canvas.drawRect(box, currentPaint);
        }
    }

    public void setTopicName(String topicName) {
        this.topicName = topicName;
    }

    public void setClearScreenTopicName(String topicName) {
        this.clearScreenTopicName = topicName;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        Subscriber<Float64MultiArray> subscriber = connectedNode.newSubscriber(topicName, Float64MultiArray._TYPE);
        subscriber.addMessageListener(new MessageListener<Float64MultiArray>() {
            @Override
            public void onNewMessage(final Float64MultiArray message) {

                // if the label contains 'select', then display a red filled box
                String type = message.getLayout().getDim().get(0).getLabel();
                if (type.contains("select")) {
                    currentPaint.setStyle(Paint.Style.FILL_AND_STROKE);
                    currentPaint.setColor(Color.argb(128,125,110,164));
                }
                else {
                    currentPaint.setStyle(Paint.Style.STROKE);
                    currentPaint.setColor(Color.argb(100,100,95,150));
                }

                double left = DisplayMethods.M2PX(message.getData()[0]);
                double top = getHeight() - DisplayMethods.M2PX(message.getData()[1]);
                double right = DisplayMethods.M2PX(message.getData()[2]);
                double bottom = getHeight() - DisplayMethods.M2PX(message.getData()[3]);
                Log.e(TAG, "Got new box to display: " + Double.toString(left) + ", " + Double.toString(top) + ", " + Double.toString(right) + ", " + Double.toString(bottom));
                newBox(left, top, right, bottom);
                postInvalidate();
            }
        });

        Subscriber<Empty> clearScreenSubscriber = connectedNode.newSubscriber(clearScreenTopicName, Empty._TYPE);
        clearScreenSubscriber.addMessageListener(new MessageListener<Empty>() {
            @Override
            public void onNewMessage(final Empty message) {
                boxes.clear();
                postInvalidate();
            }
        });
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

}
