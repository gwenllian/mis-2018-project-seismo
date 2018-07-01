package com.example.piafichtl.seismo;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
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
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
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
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import static android.support.v4.math.MathUtils.clamp;
import static java.lang.Math.pow;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "MainActivity";

    ///////////////////////////////////////
    // ACCELEROMETER
    // https://stackoverflow.com/a/8101144
    // https://stackoverflow.com/a/8323572
    private SensorManager sensorManager;
    private LineGraphSeries<DataPoint> series;
    private static double currentX;
    double ax,ay,az;
    float lastAcceleration[] = new float[3];
    float accelFilter[] = new float[3];
    float gravity[] = new float[3];


    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        if (sensorEvent.sensor.getType()==Sensor.TYPE_ACCELEROMETER){
            ax=sensorEvent.values[0];
            ay=sensorEvent.values[1];
            az=sensorEvent.values[2];

            final float alpha = 0.5f;

            gravity[0] = alpha * gravity[0] + (1 - alpha) * sensorEvent.values[0];
            gravity[1] = alpha * gravity[1] + (1 - alpha) * sensorEvent.values[1];
            gravity[2] = alpha * gravity[2] + (1 - alpha) * sensorEvent.values[2];

            accelFilter[0] = sensorEvent.values[0] - gravity[0];
            accelFilter[1] = sensorEvent.values[1] - gravity[1];
            accelFilter[2] = sensorEvent.values[2] - gravity[2];

//            float updateFreq = 30;
//            float cutOffFreq = 0.9f;
//            float RC = 1.0f / cutOffFreq;
//            float dt = 1.0f / updateFreq;
//            float filterConstant = RC / (dt + RC);
//            float alpha = filterConstant;
//            float kAccelerometerMinStep = 0.033f;
//            float kAccelerometerNoiseAttenuation = 3.0f;

//
//            float d = clamp(Math.abs(norm(accelFilter[0], accelFilter[1], accelFilter[2]) - norm(ax, ay, az)) / kAccelerometerMinStep - 1.0f, 0.0f, 1.0f);
//            alpha = d * filterConstant / kAccelerometerNoiseAttenuation + (1.0f - d) * filterConstant;
//
//            accelFilter[0] = (float) (alpha * (accelFilter[0] + ax - lastAcceleration[0]));
//            accelFilter[1] = (float) (alpha * (accelFilter[1] + ay - lastAcceleration[1]));
//            accelFilter[2] = (float) (alpha * (accelFilter[2] + az - lastAcceleration[2]));
//
//            lastAcceleration[0] = (float)ax;
//            lastAcceleration[1] = (float)ay;
//            lastAcceleration[2] = (float)az;
        }
        series.appendData(new DataPoint(currentX,accelFilter[1]), true, 200);
        currentX++;
    }

    public float norm(double a, double b, double c) {
        return (float)Math.sqrt(pow(a,2)+pow(b,2)+pow(c,2));
    }


    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
    ///////////////////////////////////////
    // CAMERA
    // https://inducesmile.com/android/android-camera2-api-example-tutorial/
    // http://werner-dittmann.blogspot.com/2016/03/using-androids-imagereader-with-camera2.html

    private String mCameraId;
    // Session for every camera frame
    private CameraCaptureSession mPreviewSession;
    private CameraDevice mCameraDevice;
    private Size mPreviewSize;

    private Button mTakePictureButton;
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

    public MainActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        sensorManager=(SensorManager) getSystemService(SENSOR_SERVICE);
        sensorManager.registerListener(this, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_NORMAL);
        GraphView graph = (GraphView) findViewById(R.id.graph);

        series = new LineGraphSeries<>();
        series.setColor(Color.RED);
        graph.addSeries(series);

        // https://stackoverflow.com/a/36400198
        graph.getGridLabelRenderer().setGridStyle(GridLabelRenderer.GridStyle.NONE);
        graph.getGridLabelRenderer().setHorizontalLabelsVisible(false);
        graph.getGridLabelRenderer().setVerticalLabelsVisible(false);

        graph.getViewport().setScalable(false);
        graph.getViewport().setScrollable(false);
        graph.getViewport().setXAxisBoundsManual(true);
        graph.getViewport().setMinX(0);
        graph.getViewport().setMaxX(200);
        graph.getViewport().setYAxisBoundsManual(true);
        graph.getViewport().setMinY(-0.05);
        graph.getViewport().setMaxY(0.05);

        currentX = 0.0;

        // finding Views
        mTextureView = (TextureView) findViewById(R.id.texture);
        assert mTextureView != null;
        mTextureView.setSurfaceTextureListener(mTextureListener);
        mTakePictureButton = (Button) findViewById(R.id.btn_takepicture);
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
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
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

                    double avg = decodeYUV420toRGBAvg(buffer.clone(),mPreviewSize.getHeight(),mPreviewSize.getWidth(),1);

                    Log.e(TAG,""+avg);
                    image.close();
                }
            };

            mCameraId = manager.getCameraIdList()[0];
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(mCameraId);
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
            manager.openCamera(mCameraId, stateCallback, null);
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
    // temporary: https://github.com/YahyaOdeh/HealthWatcher
    private static int decodeYUV420toRGBSum(byte[] yuv420sp, int width, int height, int type) {

        final int frameSize = width * height;

        int sum=0;
        int sumr = 0;
        int sumg = 0;
        int sumb = 0;
        for (int j = 0, yp = 0; j < height; j++) {
            int uvp = frameSize + (j >> 1) * width, u = 0, v = 0;
            for (int i = 0; i < width; i++, yp++) {
                int y = (0xff & yuv420sp[yp]) - 16;
                if (y < 0) y = 0;
                if ((i & 1) == 0) {
                    v = (0xff & yuv420sp[uvp++]) - 128;
                    u = (0xff & yuv420sp[uvp++]) - 128;
                }
                int y1192 = 1192 * y;
                int r = (y1192 + 1634 * v);
                int g = (y1192 - 833 * v - 400 * u);
                int b = (y1192 + 2066 * u);

                if (r < 0) r = 0;
                else if (r > 262143) r = 262143;
                if (g < 0) g = 0;
                else if (g > 262143) g = 262143;
                if (b < 0) b = 0;
                else if (b > 262143) b = 262143;

                int pixel = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00) | ((b >> 10) & 0xff);
                int red = (pixel >> 16) & 0xff;
                int green = (pixel >> 8) & 0xff;
                int blue = pixel&0xff;
                sumr += red;
                sumg += green;
                sumb += blue;
            }
        }
        switch(type){
            case (1): sum = sumr;
                break;
            case (2): sum = sumb;
                break;
            case (3): sum = sumg;
                break;
        }
        return sum;
    }


    public static double decodeYUV420toRGBAvg(byte[] yuv420sp, int width, int height, int type) {
        final int frameSize = width * height;

        int sum = decodeYUV420toRGBSum(yuv420sp, width, height, type);
        int mean = (sum / frameSize);

        return mean;
    }

}
