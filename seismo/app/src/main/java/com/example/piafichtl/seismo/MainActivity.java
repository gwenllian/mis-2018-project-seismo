package com.example.piafichtl.seismo;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.*;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.CountDownTimer;
import android.os.Handler;
import android.os.HandlerThread;
import android.renderscript.Allocation;
import android.renderscript.Element;
import android.renderscript.RenderScript;
import android.renderscript.ScriptIntrinsicYuvToRGB;
import android.renderscript.Type;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.GridLabelRenderer;
import com.jjoe64.graphview.series.DataPoint;
import com.jjoe64.graphview.series.LineGraphSeries;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import static android.support.v4.math.MathUtils.clamp;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "MainActivity";

    Vector<Pair> redValues = new Vector<>();
    Vector<Pair> greenValues = new Vector<>();
    Vector<Pair> accelValues = new Vector<>();
    Vector<Pair> accelSmoothValues = new Vector<>();


    public void calculateBloodPressure() {
        // iterate red values and check if the value rises or not
        Vector<Long> fingerPulses = new Vector<>();
        long sumPulseDifference = 0;
        long lastValue = 0;
        long lastTime = 0;
        for (Pair<Long,Long> r : redValues) {
            if (r.second > lastValue) {
                // save only times where value increases
                fingerPulses.add(r.first);
                if (lastValue != 0) {
                    sumPulseDifference += r.first - lastTime;
                }
                lastValue = r.second;
                lastTime = r.first;
            } else if (r.second < lastValue) {
                lastValue = r.second;
            }
        }
        long PPG = sumPulseDifference / fingerPulses.size();

        // get approximate peaks
        Vector<Long> possibleHeartBeats = new Vector<>();
        Pair<Long,Float> current = accelSmoothValues.firstElement();
        Pair<Long,Float> possiblePeak = current;
        for (Pair<Long,Float> a : accelSmoothValues) {
            if (current.second <= a.second) {
                possiblePeak = a;
                current = a;
            } else {
                possibleHeartBeats.add(possiblePeak.first);
                current = a;
            }
        }

        // limiting signals to the closest acceleration signal that came before the actual pulse
        Vector<Long> closestSignals = new Vector<>();
        long sumBeatDifference = 0;
        lastTime = 0;
        for (long f : fingerPulses) {
            long difference = Math.abs(possibleHeartBeats.firstElement() - f);
            long possibleMatch = 0;
            for (long p : possibleHeartBeats) {
                if (p < f) {
                    long cdifference = Math.abs(p - f);
                    if (cdifference < difference) {
                        possibleMatch = p;
                        difference = cdifference;
                    }
                }
            }
            if (lastTime != 0) {
                sumBeatDifference += f-lastTime;
            }
            lastTime = f;
            closestSignals.add(possibleMatch);
        }

        long SCG = sumBeatDifference / closestSignals.size();

        // smoothed z-score algorithm https://stackoverflow.com/a/48772305


        // get acceleration maximums
        // (the acceleration shows a jump from a minimum to maximum with a heart beat)
        // compare values to acceleration maximums and stabilize result
        // calculate time interval from one maximum to its corresponding finger pulse
        Log.i(TAG,"FINGER PULSE SIGNALS " + fingerPulses.toString());
        Log.i(TAG,"\nPOSSIBLE HEARTBEATS " + possibleHeartBeats.toString());
        Log.i(TAG,"\nMATCHING HEARTBEAT SIGNALS " + closestSignals.toString());
        Log.i(TAG,"\n\nPPG " + PPG);
        Log.i(TAG,"\nSCG " + SCG);

    }

    public class CountDownTimerMeasurement extends CountDownTimer {
        long timer;
        public CountDownTimerMeasurement(long startTime, long interval) {
            super(startTime, interval);
            timer = startTime;
        }

        @Override
        public void onFinish() {
            Toast.makeText(MainActivity.this, "Time's up!", Toast.LENGTH_LONG).show();
            Log.i(TAG, "RED VALUE MEASUREMENT [" + redValues.size() + "]\n" + redValues.toString());
            Log.i(TAG, "\nGREEN VALUE MEASUREMENT [" + greenValues.size() + "]\n" + greenValues.toString());
            Log.i(TAG, "\nACCELERATION VALUE MEASUREMENT [" + accelValues.size() + "]\n" + accelValues.toString());
            Log.i(TAG, "\nSMOOTHED ACCELERATION VALUE MEASUREMENT [" + accelSmoothValues.size() + "]\n" + accelSmoothValues.toString());
            calculateBloodPressure();

        }

        @Override
        public void onTick(long millisUntilFinished) {
            redValues.add(new Pair<>(timer - millisUntilFinished, redMean));
            greenValues.add(new Pair<>(timer - millisUntilFinished, greenMean));
            accelValues.add(new Pair<>(timer - millisUntilFinished, currentAcceleration));
            accelSmoothValues.add(new Pair<>(timer - millisUntilFinished, currentSmoothAcceleration));
        }
    }

    ///////////////////////////////////////
    // ACCELEROMETER
    // https://stackoverflow.com/a/8101144
    // https://stackoverflow.com/a/8323572
    private SensorManager sensorManager;
    private LineGraphSeries<DataPoint> rawAccelSeries;
    private LineGraphSeries<DataPoint> filteredAccelSeries;
    private LineGraphSeries<DataPoint> smoothedAccelSeries;
    private static double currentAccelX;
    double ax,ay,az;
    float currentAcceleration, currentSmoothAcceleration;
    float lastAcceleration[] = new float[3];
    float accelFilterA[] = new float[3];
    float accelFilterB[] = new float[3];
    float gravityA[] = new float[3];
    float gravityB[] = new float[3];


    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor.getType()==Sensor.TYPE_ACCELEROMETER){
            ax=sensorEvent.values[0];
            ay=sensorEvent.values[1];
            az=sensorEvent.values[2];

            final float alpha = 0.15f;
            final float beta = 0.005f;

            gravityA[0] = alpha * gravityA[0] + (1 - alpha) * sensorEvent.values[0];
            gravityA[1] = alpha * gravityA[1] + (1 - alpha) * sensorEvent.values[1];
            gravityA[2] = alpha * gravityA[2] + (1 - alpha) * sensorEvent.values[2];

            accelFilterA[0] = sensorEvent.values[0] - gravityA[0];
            accelFilterA[1] = sensorEvent.values[1] - gravityA[1];
            accelFilterA[2] = sensorEvent.values[2] - gravityA[2];

            gravityB[0] = beta * gravityB[0] + (1 - beta) * sensorEvent.values[0];
            gravityB[1] = beta * gravityB[1] + (1 - beta) * sensorEvent.values[1];
            gravityB[2] = beta * gravityB[2] + (1 - beta) * sensorEvent.values[2];

            accelFilterB[0] = sensorEvent.values[0] - gravityB[0];
            accelFilterB[1] = sensorEvent.values[1] - gravityB[1];
            accelFilterB[2] = sensorEvent.values[2] - gravityB[2];
        }
        currentAcceleration = accelFilterA[1];
        currentSmoothAcceleration = accelFilterB[1];
        rawAccelSeries.appendData(new DataPoint(currentAccelX,(float)ay), true, 100);
        filteredAccelSeries.appendData(new DataPoint(currentAccelX,currentAcceleration), true, 100);
        smoothedAccelSeries.appendData(new DataPoint(currentAccelX,currentSmoothAcceleration), true, 100);
        currentAccelX++;
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
    ///////////////////////////////////////
    // CAMERA
    // https://inducesmile.com/android/android-camera2-api-example-tutorial/
    // http://werner-dittmann.blogspot.com/2016/03/using-androids-imagereader-with-camera2.html

    private CameraManager mCameraManager;
    private String mCameraId;
    // Session for every camera frame
    private CameraCaptureSession mPreviewSession;
    private CameraDevice mCameraDevice;
    private Size mPreviewSize;

    private TextureView mTextureView;
    private static final SparseIntArray ORIENTATIONS = new SparseIntArray();
    static {
        ORIENTATIONS.append(Surface.ROTATION_0, 90);
        ORIENTATIONS.append(Surface.ROTATION_90, 0);
        ORIENTATIONS.append(Surface.ROTATION_180, 270);
        ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }

    protected CaptureRequest.Builder mPreviewBuilder;
    private ImageReader mImageReader;
    private File mFile;
    private static final int REQUEST_CAMERA_PERMISSION = 200;
    private boolean mFlashSupported;
    private Handler mBackgroundHandler;
    private HandlerThread mBackgroundThread;

    RenderScript rs;
    ScriptIntrinsicYuvToRGB si;

    long redMean;
    long greenMean;
    private static double currentCamX;
    private GraphView accelGraph;
    private GraphView camGraph;
    private LineGraphSeries<DataPoint> redSeries;
    private LineGraphSeries<DataPoint> greenSeries;

    private Button mStartButton;
    boolean measuring = false;
    long startingTime;

    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Boolean isFlashAvailable = getApplicationContext().getPackageManager()
                .hasSystemFeature(PackageManager.FEATURE_CAMERA_FLASH);

        if (!isFlashAvailable) {

            Toast.makeText(MainActivity.this, "Sorry this app needs to use the flash", Toast.LENGTH_LONG).show();
            finish();
            return;
        }

        rs = RenderScript.create(this);
        si = ScriptIntrinsicYuvToRGB.create(rs, Element.U8_4(rs));

        mStartButton = (Button) findViewById(R.id.start);
        //mStartButton.setVisibility(View.VISIBLE);
        mStartButton.setOnClickListener(new View.OnClickListener() {
                                            public void onClick(View v) {
                                                //mStartButton.setVisibility(View.GONE);
                                                startingTime = System.currentTimeMillis();
                                                measuring = true;
                                                CountDownTimerMeasurement timer = new CountDownTimerMeasurement(10000,100);
                                                timer.start();
                                                Toast.makeText(MainActivity.this, "Starting measurement", Toast.LENGTH_LONG).show();
                                            }});
        
        mCameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            mCameraId = mCameraManager.getCameraIdList()[0];
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }

        sensorManager=(SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
        accelGraph = (GraphView) findViewById(R.id.graph);
        camGraph = (GraphView) findViewById(R.id.camgraph);

        redSeries = new LineGraphSeries<>();
        redSeries.setColor(Color.RED);
        greenSeries = new LineGraphSeries<>();
        greenSeries.setColor(Color.GREEN);
        camGraph.addSeries(redSeries);
        camGraph.addSeries(greenSeries);

        camGraph.getGridLabelRenderer().setGridStyle(GridLabelRenderer.GridStyle.NONE);
        camGraph.getGridLabelRenderer().setHorizontalLabelsVisible(false);
        camGraph.getGridLabelRenderer().setVerticalLabelsVisible(false);

        camGraph.getViewport().setScalable(false);
        camGraph.getViewport().setScrollable(false);
        camGraph.getViewport().setXAxisBoundsManual(true);
        camGraph.getViewport().setMinX(0);
        camGraph.getViewport().setMaxX(10);
        camGraph.getViewport().setYAxisBoundsManual(true);
        camGraph.getViewport().setMinY(0);
        camGraph.getViewport().setMaxY(255);

        currentCamX = 0.0;

        rawAccelSeries = new LineGraphSeries<>();
        rawAccelSeries.setColor(Color.GRAY);
        filteredAccelSeries = new LineGraphSeries<>();
        filteredAccelSeries.setColor(Color.MAGENTA);
        smoothedAccelSeries = new LineGraphSeries<>();
        smoothedAccelSeries.setColor(Color.LTGRAY);

        accelGraph.addSeries(rawAccelSeries);
        accelGraph.addSeries(filteredAccelSeries);
        accelGraph.addSeries(smoothedAccelSeries);

        // https://stackoverflow.com/a/36400198
        accelGraph.getGridLabelRenderer().setGridStyle(GridLabelRenderer.GridStyle.NONE);
        accelGraph.getGridLabelRenderer().setHorizontalLabelsVisible(false);
        accelGraph.getGridLabelRenderer().setVerticalLabelsVisible(false);

        accelGraph.getViewport().setScalable(false);
        accelGraph.getViewport().setScrollable(false);
        accelGraph.getViewport().setXAxisBoundsManual(true);
        accelGraph.getViewport().setMinX(0);
        accelGraph.getViewport().setMaxX(100);
        accelGraph.getViewport().setYAxisBoundsManual(true);
        accelGraph.getViewport().setMinY(-0.01);
        accelGraph.getViewport().setMaxY(0.01);

        currentAccelX = 0.0;

        // finding Views
        mTextureView = (TextureView) findViewById(R.id.texture);
        assert mTextureView != null;
        mTextureView.setSurfaceTextureListener(mTextureListener);
    }

    TextureView.SurfaceTextureListener mTextureListener = new TextureView.SurfaceTextureListener() {
        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
            openCamera();
        }
        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
            // Transform your image captured size according to the surface width and height
        }
        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
            return false;
        }
        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        }
    };

    private final CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice camera) {
            //This is called when the camera is open
            Log.e(TAG, "onOpened");
            mCameraDevice = camera;
            createCameraPreview();
        }
        @Override
        public void onDisconnected(CameraDevice camera) {
            mCameraDevice.close();
            mCameraDevice = null;

        }
        @Override
        public void onError(CameraDevice camera, int error) {
            mCameraDevice.close();
            mCameraDevice = null;
        }
    };

    final CameraCaptureSession.CaptureCallback captureCallbackListener = new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request, TotalCaptureResult result) {
            super.onCaptureCompleted(session, request, result);
            Toast.makeText(MainActivity.this, "Saved:" + mFile, Toast.LENGTH_SHORT).show();
            createCameraPreview();
        }
    };

    protected void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("Camera Background");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    protected void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    protected void createCameraPreview() {
        if (mCameraDevice == null ||!mTextureView.isAvailable() ||mPreviewSize == null) {
            return;
        }
        try {
            SurfaceTexture texture = mTextureView.getSurfaceTexture();
            assert texture != null;
            texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());
            mPreviewBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            mPreviewBuilder.set(CaptureRequest.FLASH_MODE, CameraMetadata.FLASH_MODE_TORCH);
            List surfaces = new ArrayList<>();

            Surface previewSurface = new Surface(texture);
            surfaces.add(previewSurface);
            mPreviewBuilder.addTarget(previewSurface);

            Surface readerSurface = mImageReader.getSurface();
            surfaces.add(readerSurface);
            mPreviewBuilder.addTarget(readerSurface);

            mCameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback(){
                @Override
                public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                    //The camera is already closed
                    if (null == mCameraDevice) {
                        return;
                    }
                    // When the session is ready, we start displaying the preview.
                    mPreviewSession = cameraCaptureSession;
                    updatePreview();
                }
                @Override
                public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {
                    Toast.makeText(MainActivity.this, "Configuration change", Toast.LENGTH_SHORT).show();
                }
            }, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }


    private void openCamera() {
        Log.e(TAG, "is camera open");
        // Added ImageReader handling to capture every single frame

        try {
            ImageReader.OnImageAvailableListener mImageAvailable = new ImageReader.OnImageAvailableListener() {
                @Override
                public void onImageAvailable(ImageReader reader) {
                    // FOR EVERY FRAME DO:
                    Image image = reader.acquireLatestImage();
                    if (image == null)
                        return;
                    Image.Plane Y = image.getPlanes()[0];
                    Image.Plane U = image.getPlanes()[1];
                    Image.Plane V = image.getPlanes()[2];

                    int Yb = Y.getBuffer().remaining();
                    int Ub = U.getBuffer().remaining();
                    int Vb = V.getBuffer().remaining();

                    byte[] buffer = new byte[Yb + Ub + Vb];


                    Y.getBuffer().get(buffer, 0, Yb);
                    U.getBuffer().get(buffer, Yb + Vb, Ub);
                    V.getBuffer().get(buffer, Yb, Vb);

                    Bitmap bmp = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.ARGB_8888);
                    Allocation bmData = renderScriptNV21ToRGBA888(
                            getApplicationContext(),
                            image.getWidth(),
                            image.getHeight(),
                            buffer);
                    bmData.copyTo(bmp);

                    int rgba[] = new int[mPreviewSize.getWidth()*mPreviewSize.getHeight()];   // the rgba[] array

                    bmp.getPixels(rgba, 0, mPreviewSize.getWidth(), 0, 0, mPreviewSize.getWidth(), mPreviewSize.getHeight());

                    long sumR = 0;
                    long sumG = 0;
                    for (int p : rgba){
                        int red = Color.red(p);
                        int green = Color.green(p);

                        sumR += red;
                        sumG += green;
                    }

                    redMean = sumR/rgba.length;
                    greenMean = sumG/rgba.length;

                    redSeries.appendData(new DataPoint(currentCamX,redMean), true, 10);
                    greenSeries.appendData(new DataPoint(currentCamX,greenMean), true, 10);


                    currentCamX++;
                    image.close();
                }
            };

            CameraCharacteristics characteristics = mCameraManager.getCameraCharacteristics(mCameraId);
            StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            assert map != null;
            // Assigning appropriate sizes and YUV image format
            // YUV ist supported by every device; others should be used with caution
            mPreviewSize = map.getOutputSizes(SurfaceTexture.class)[0];
            mImageReader = ImageReader.newInstance(mPreviewSize.getWidth(),
                    mPreviewSize.getHeight(),
                    ImageFormat.YUV_420_888, 3);
            mImageReader.setOnImageAvailableListener(mImageAvailable,mBackgroundHandler);

            // Add permission for camera and let user grant the permission
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE}, REQUEST_CAMERA_PERMISSION);
                return;
            }
            mCameraManager.openCamera(mCameraId, stateCallback, null);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }

        Log.e(TAG, "openCamera X");
    }

    protected void updatePreview() {
        if(null == mCameraDevice) {
            Log.e(TAG, "updatePreview error, return");
        }
        mPreviewBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);

        try {
            mPreviewSession.setRepeatingRequest(mPreviewBuilder.build(), null, mBackgroundHandler);

        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private void closeCamera() {
        if (null != mCameraDevice) {
            mCameraDevice.close();
            mCameraDevice = null;
        }
        if (null != mImageReader) {
            mImageReader.close();
            mImageReader = null;
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        if (requestCode == REQUEST_CAMERA_PERMISSION) {
            if (grantResults[0] == PackageManager.PERMISSION_DENIED) {
                // close the app
                Toast.makeText(MainActivity.this, "Sorry!!!, you can't use this app without granting permission", Toast.LENGTH_LONG).show();
                finish();
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.e(TAG, "onResume");
        startBackgroundThread();
        if (mTextureView.isAvailable()) {
            openCamera();
        } else {
            mTextureView.setSurfaceTextureListener(mTextureListener);
        }

    }

    public void onDestroy() {
        super.onDestroy();
    }


    // IMAGE PROCESSING
    // https://stackoverflow.com/questions/11761147/get-rgb-data-from-the-android-camera-in-stream
    public Allocation renderScriptNV21ToRGBA888(Context context, int width, int height, byte[] nv21) {

        Type.Builder yuvType = new Type.Builder(rs, Element.U8(rs)).setX(nv21.length);
        Allocation in = Allocation.createTyped(rs, yuvType.create(), Allocation.USAGE_SCRIPT);

        Type.Builder rgbaType = new Type.Builder(rs, Element.RGBA_8888(rs)).setX(width).setY(height);
        Allocation out = Allocation.createTyped(rs, rgbaType.create(), Allocation.USAGE_SCRIPT);

        in.copyFrom(nv21);

        si.setInput(in);
        si.forEach(out);
        return out;
    }

    public void turnOnFlash() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                mCameraManager.setTorchMode(mCameraId, true);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void turnOffFlash() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                mCameraManager.setTorchMode(mCameraId, false);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


}
