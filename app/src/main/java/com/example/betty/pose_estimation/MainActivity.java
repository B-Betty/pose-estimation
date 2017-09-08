package com.example.betty.pose_estimation;

import android.content.BroadcastReceiver;
import android.content.pm.ActivityInfo;
import android.os.Handler;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;
import android.widget.TextView;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat.*;
import org.opencv.core.*;

import static org.opencv.imgproc.Imgproc.COLOR_BGR2GRAY;
import static org.opencv.imgproc.Imgproc.cvtColor;


public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("native-lib");
        System.loadLibrary("pose_estimation");
        System.loadLibrary("opencv_java3");
    }
    private   TextView tv;
    Mat mRgb1;
    Mat mGray;
    Mat first_frame;
    //opencv相机借口
    private CameraBridgeViewBase cameraBridgeViewBase;
    final String TAG = "pose-estimation";
    private int numFrame = 0;
    private BaseLoaderCallback baseLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case BaseLoaderCallback.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    //开启摄像头
                    cameraBridgeViewBase.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }

        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_main);
        cameraBridgeViewBase = (CameraBridgeViewBase) findViewById(R.id.CameraView);
        cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        cameraBridgeViewBase.setCvCameraViewListener(this);
        this.setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
    }
    @Override
    protected void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found.Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, baseLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package.Using it !");
            baseLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mRgb1 = new Mat();
        first_frame = new Mat();
    }

    @Override
    public void onCameraViewStopped() {
        mRgb1.release();
        first_frame.release();
    }

    double[] r_value = {0,0,0,0,0,0,0,0,0,0,0,0};

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        mRgb1 = inputFrame.rgba();
        mGray = inputFrame.gray();
        if (numFrame == 0 && !mGray.empty()) {
            mGray.copyTo(first_frame);
//            cvtColor(first_frame, first_frame, COLOR_BGR2GRAY);
        } else {
            r_value = floatMatrixFromJNI(mGray.getNativeObjAddr(), first_frame.getNativeObjAddr());
            if (r_value == null){
                return mGray;
            }else {
                for (int i = 0; i<r_value.length; i++){
                    Log.i("旋转矩阵、平移矩阵", "\n"+r_value[i]);
                }
            }
               }
        numFrame++;
        return mGray;
    }
    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     *
     * @param nativeObjAddr
     * @param nativeObjAddr1
     */
    public native double[] floatMatrixFromJNI(long nativeObjAddr, long nativeObjAddr1);
    public native String stringFromJNI();
}