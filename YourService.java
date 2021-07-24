package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;
// astrobee library
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
// android library
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.qrcode.decoder.QRCodeDecoderMetaData;
import com.google.zxing.qrcode.encoder.QRCode;

// zxing library
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.objdetect.QRCodeDetector;

import static org.opencv.android.Utils.matToBitmap;
// opencv library
import java.util.ArrayList;
import java.util.List;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    String MODE = "sim"; // mode setting ("sim" or "iss")
    int NAV_MAX_COL = 1280;
    int NAV_MAX_ROW =  960;
    int PointCloud_COL = 224;
    int PointCloud_ROW = 171;
    // carmera constant value
    int max_count = 5, center_range = 6, P1 = 0, P2 = 1;
    // limit value
    float AR_diagonal = 0.07071067812f;
    float ARtoTarget = 0.1414f, y_shift = 0.1328f;
    @Override
    protected void runPlan1(){
        // astrobee is undocked and the mission starts
        api.startMission();
// move to Point-A
        Point point1 = new Point(11.21, -9.8, 4.79);
        Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        api.moveTo(point1, quaternion1, true);
        Bitmap image = api.getBitmapNavCam();
        String contents = QR_event(image);
        api.sendDiscoveredQR(contents);

        api.laserControl(true);


        api.takeSnapshot();
        // turn the laser off
        api.laserControl(false);
// move to the rear of Bay7
        Point point2 = new Point(10.6, -8.0, 4.5);
        Quaternion quaternion2 = new Quaternion(0f, -0.707f, 0f, 0.707f);
        api.moveTo(point2, quaternion2, true);
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while(!result.hasSucceeded() || loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }
    public Mat undistord(Mat src)
    {
        Mat dst = new Mat(1280, 960, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        int row = 0, col = 0;

        double cameraMatrix_sim[] =
                {
                        344.173397, 0.000000, 630.793795,
                        0.000000, 344.277922, 487.033834,
                        0.000000, 0.000000, 1.000000
                };
        double distCoeffs_sim[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};

        double cameraMatrix_orbit[] =
                {
                        692.827528, 0.000000, 571.399891,
                        0.000000, 691.919547, 504.956891,
                        0.000000, 0.000000, 1.000000
                };
        double distCoeffs_orbit[] = {-0.312191, 0.073843, -0.000918, 0.001890, 0.000000};

        if(MODE == "sim")
        {
            cameraMatrix.put(row, col, cameraMatrix_sim);
            distCoeffs.put(row, col, distCoeffs_sim);
            Log.d("Mode[camera]:"," sim");
        }
        else if(MODE == "iss")
        {
            cameraMatrix.put(row, col, cameraMatrix_orbit);
            distCoeffs.put(row, col, distCoeffs_orbit);
            Log.d("Mode[camera]:"," iss");
        }

        cameraMatrix.put(row, col, cameraMatrix_orbit);
        distCoeffs.put(row, col, distCoeffs_orbit);

        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
        return dst;
    }

    public Rect cropImage(int percent_crop)
    {
        double ratio = NAV_MAX_COL / NAV_MAX_ROW;

        double percent_row = percent_crop/2;
        double percent_col = percent_row * ratio;

        int offset_row = (int) percent_row * NAV_MAX_ROW / 100;
        int offset_col = (int) percent_col * NAV_MAX_COL / 100;
        double rows = NAV_MAX_ROW - (offset_row * 2);
        double cols = NAV_MAX_COL - (offset_col * 2);

        return new Rect(offset_col, offset_row, (int) cols, (int) rows);
    }
    public Bitmap resizeImage(Mat src, int width, int height)
    {
        Size size = new Size(width, height);
        Imgproc.resize(src, src, size);

        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        matToBitmap(src, bitmap, false);
        return bitmap;
    }
    public double[] Intersection(double p[][])
    {
        double[] center = new double[3];

        double a = (p[1][0] - p[0][0]) * (p[3][0] - p[2][0]);
        double b = (p[1][0] - p[0][0]) * (p[3][1] - p[2][1]);
        double c = (p[3][0] - p[2][0]) * (p[1][1] - p[0][1]);

        center[0] = (a * p[0][1] + b * p[2][0] - a * p[2][1] - c * p[0][0]) / (b - c);
        center[1] = ((p[1][1] - p[0][1]) * (center[0] - p[0][0]) / (p[1][0] - p[0][0])) + p[0][1];

        double x_l1 = Math.pow(p[0][0] - p[1][0], 2);
        double y_l1 = Math.pow(p[0][1] - p[1][1], 2);
        double x_l2 = Math.pow(p[3][0] - p[2][0], 2);
        double y_l2 = Math.pow(p[3][1] - p[2][1], 2);
        double avg = (Math.sqrt(x_l1 + y_l1) + Math.sqrt(x_l2 + y_l2)) / 2;

        center[2] = avg / AR_diagonal;
        Log.d("AR[info]: ", center[0] + ", " + center[1] + ", " + center[2]);
        return center;
    }
    public double getPointCloud(int center_range)
    {
        double depth = 0;
        int count = 0;

        Log.d("PointCloud[status]:", " start");
        PointCloud hazCam = api.getPointCloudHazCam();
        Point[] point = hazCam.getPointArray();
        int width  = hazCam.getWidth();
        int height = hazCam.getHeight();
        int row_max = height/2 + center_range/2;
        int row_min = height/2 - center_range/2;
        int col_max = width/2  + center_range/2;
        int col_min = width/2  - center_range/2;
        Log.d("PointCloud[status]:", " stop");


        //////////////////////////////////////////////////////////////////////////////////////////////////////
        for (int row = row_min; row < row_max; row++)
        {
            for (int col = col_min; col < col_max; col++)
            {
                depth += point[(row * width) + col].getZ();
                count++;
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////


        depth /= count;
        Log.d("PointCloud[value]:", "z[" + depth + "]");
        return depth;
    }
    public void flash_control(boolean status)
    {
        if(status)
        {
            api.flashlightControlFront(0.025f);

            try
            {
                Thread.sleep(1000); // wait a few seconds
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        else api.flashlightControlFront(0);
    }
    /*public void judgeSendFinish(boolean laser_control)
    {
        api.laserControl(laser_control);

        if(MODE == "sim")
        {
            Log.d("Mode[judge]:"," sim");
            api.judgeSendFinishSimulation();
        }
        else if(MODE == "iss")
        {
            Log.d("Mode[judge]:"," iss");
            api.judgeSendFinishISS();
        }
    }*/
    public String QR_event(Bitmap img)
    {
        String contents = null;
        int count = 0;
        double final_x = 0, final_y = 0, final_z = 0, final_w = 0;

        while (contents == null && count < 2)
        {
            Log.d("QR[status]:", " start");
            long start_time = SystemClock.elapsedRealtime();
            //                                            //
            Log.d("QR[NO]:"," "+1);

            flash_control(true);
            Mat src_mat = new Mat(undistord(api.getMatNavCam()), cropImage(40));
            img = resizeImage(src_mat, 2000, 1500);



            //////////////////////////////////////////////////////////////////////////////////////////////////////
            int[] intArray = new int[img.getWidth() * img.getHeight()];
            img.getPixels(intArray, 0, img.getWidth(), 0, 0, img.getWidth(), img.getHeight());

            LuminanceSource source = new RGBLuminanceSource(img.getWidth(), img.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

            try
            {
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                contents = result.getText();
                Log.d("QR[status]:", " Detected");

                String[] multi_contents = contents.split(", ");
                final_x = Double.parseDouble(multi_contents[1]);
                final_y = Double.parseDouble(multi_contents[3]);
                final_z = Double.parseDouble(multi_contents[5]);
                //if(no == 1) final_w = Math.sqrt(1 - final_x*final_x - final_y*final_y - final_z*final_z);
            }
            catch (Exception e)
            {
                Log.d("QR[status]:", " Not detected");
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            Log.d("QR[status]:", " stop");
            long stop_time = SystemClock.elapsedRealtime();



            Log.d("QR[count]:", " " + count);
            Log.d("QR[total_time]:"," "+ (stop_time-start_time)/1000);
            count++;
        }


        flash_control(false);
        //api.sendDiscoveredQR(contents);
        //return new double[] {final_x, final_y, final_z, final_w};
        return contents;
    }
    public double[] AR_event(float pos_x, float pos_y, float pos_z, float qua_x, float qua_y, float qua_z, float qua_w, int count_max, boolean sent_AR)
    {
        int contents = 0, count = 0;
        double result[] = new double[3];

        while (contents == 0 && count < count_max)
        {
            Log.d("AR[status]:", " start");
            long start_time = SystemClock.elapsedRealtime();
            //                                            //
            moveToWrapper(pos_x, pos_y, pos_z, qua_x, qua_y, qua_z, qua_w);



            //////////////////////////////////////////////////////////////////////////////////////////////////////
            Mat source = undistord(api.getMatNavCam());
            //   Kinematics robot = api.getTrustedRobotKinematics(5);
            Mat ids = new Mat();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();

            try
            {
                Aruco.detectMarkers(source, dictionary, corners, ids);
                contents = (int) ids.get(0, 0)[0];

                // if(sent_AR) api.judgeSendDiscoveredAR(Integer.toString(contents));
                // Log.d("AR[status]:", " Detected");


                double[][] AR_corners =
                        {
                                {(int) corners.get(0).get(0, 0)[0], (int) corners.get(0).get(0, 0)[1]},
                                {(int) corners.get(0).get(0, 2)[0], (int) corners.get(0).get(0, 2)[1]},
                                {(int) corners.get(0).get(0, 1)[0], (int) corners.get(0).get(0, 1)[1]},
                                {(int) corners.get(0).get(0, 3)[0], (int) corners.get(0).get(0, 3)[1]}
                        };
                double[] AR_info = Intersection(AR_corners);


                Point point = new Point(pos_x, pos_y, pos_z);
                if(robot != null)
                {
//                    point = robot.getPosition();
                    Log.d("getKinematics[status]:"," Finished");
                }
                result[0] = point.getX() + (AR_info[0]- NAV_MAX_COL/2) / AR_info[2];
                result[1] = point.getY();
                result[2] = point.getZ() + (AR_info[1]- NAV_MAX_ROW/2) / AR_info[2];
            }
            catch (Exception e)
            {
                Log.d("AR[status]:", " Not detected");
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            Log.d("AR[status]:", " stop");
            long stop_time = SystemClock.elapsedRealtime();



            Log.d("AR[count]:", " " + count);
            Log.d("AR[total_time]:"," "+ (stop_time-start_time)/1000);
            count++;
        }
        return result;
    }

}

