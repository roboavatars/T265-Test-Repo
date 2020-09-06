package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.FrameGrabber;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * This class processes camera frames from {@linkplain FrameGrabber}
 * to determine the location of stones in autonomous
 */

@SuppressWarnings({"FieldCanBeLocal"}) @SuppressLint({"DefaultLocale","SdCardPath"})
public class stoneLocator2 extends Thread {

    // Variables
    private final static String basePath = "/sdcard/FIRST/procFiles2/";
    private final static String inputPath = basePath + "input";
    private final static String filteredPath = basePath + "filtered";
    private final static String contoursPath = basePath + "contours";
    private final static String circlePath = basePath + "circle";
    private final static String ellipsePath = basePath + "ellipse";
    private final static String testPath = "/sdcard/FIRST/input/";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true; // <<<----------------------
    private final boolean debug = true;
    private boolean active = false;

    private final static double hpg = 8.5;
    private final static double phi = Math.toRadians(15);
    private final static double verticalFOV = Math.toRadians(57);
    private final static double horizontalFOV = Math.toRadians(71);
    private final static double minimumYDistance = 6;
    private final static double maximumYDistance = 40;
    private final static double contourMinimumArea = 1000;
    private final static double numberOfFrames = 2500;

    private int frameNum = 1;
    private double[] sPos = {-1, -1, -1};
    private double time = -1;
    private double contourArea = -1;
    private ElapsedTime timer = new ElapsedTime();

    private LinearOpMode op;
    public stoneLocator2(LinearOpMode opMode) {op = opMode;}

    // Phone Position-
    // 7in up, side closest to camera is 7.5in from left of robot (aligned to depot), slight tilt forward

    // difference between 2 skystones is 170 px
    
    /**
     * Enables the camera view
     */
    public void initializeCamera() {
//        telemetry("Initializing OpenCV", "v" + OpenCVLoader.OPENCV_VERSION);
        if (usingCamera) FtcRobotControllerActivity.enableCameraView();
        telemetry("Status", "Ready");
    }
    
    /**
     * Runs OpenCV Thread
     */
    @Override public void run() {
        setName("OpenCV2");

        // Clear Image Folder
        File dir = new File(basePath);
        String[] children = dir.list();
        if (children != null) {for (String child : children) {new File(dir, child).delete();}}

        // Use and Process Camera Image, or Use and Process Hardcoded Image
        if (usingCamera) {
            frameGrabber = FtcRobotControllerActivity.frameGrabber;

            while (active) {
//                Mat input = frameGrabber.getNextMat();
                Mat input = new Mat();
                if (input != null) {
                    log("Frame " + frameNum + " ----------------------------------------");
                    sPos = detectSkyStone(input);
                    frameNum++;
                } else sPos = new double[]{-1, -1, -1};
            }
            FtcRobotControllerActivity.disableCameraView();
        } else {
            for (int i = 0; i < numberOfFrames; i++) {
                Mat input = Imgcodecs.imread(testPath + "input" + i + ".jpg", Imgcodecs.IMREAD_COLOR);
                Imgproc.resize(input, input, new Size(200, 150));
                sPos = detectSkyStone(input);
                frameNum++;
            }
        }
        log("");
    }
    
    /**
     * Finds position value of stone by processing an input frame
     * @param input the camera frame that will be used to detect skystones
     * @return x, y, and theta of stone
     */
    private double[] detectSkyStone (Mat input) {
        // Log Input Image and Reset Variables and Timer
        timer.reset();
        double xpix = -1;
        double ypix = -1;
        double stoneX = -1;
        double stoneY = -1;
        double stoneTheta = -1;
        contourArea = -1;
        new File(inputPath + (frameNum % numberOfFrames) + ".jpg").delete();
        if (debug) {Imgcodecs.imwrite(inputPath + (frameNum % numberOfFrames) + ".jpg", input);}

        // Process Image
        Mat filtered = new Mat(input.rows(), input.cols(), CvType.CV_8UC1, new Scalar(255));
        Imgproc.cvtColor(input, filtered, Imgproc.COLOR_RGB2HSV);
        Core.inRange(filtered, new Scalar(85, 95, 95), new Scalar(115, 255, 255), filtered);
        Imgproc.morphologyEx(filtered, filtered, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(filtered, filtered, Imgproc.MORPH_CLOSE, new Mat());
        new File(filteredPath + (frameNum % numberOfFrames) + ".jpg").delete();
        if (debug) Imgcodecs.imwrite(filteredPath + (frameNum % numberOfFrames) + ".jpg", filtered);

        // Further Process Image
        Imgproc.Canny(filtered, filtered, 0, 0, 3, false);
        new File(contoursPath + (frameNum % numberOfFrames) + ".jpg").delete();
        if (debug) Imgcodecs.imwrite(contoursPath + (frameNum % numberOfFrames) + ".jpg", filtered);

        // Find Contours
        Mat heirarchyMat = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(filtered, contours, heirarchyMat, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);

        if (contours.size() != 0) {
            // Find Bottommost Point
            int contourIndex = 0;
            for (int i = 0; i < contours.size(); i++) {
                for (int j = 0; j < contours.get(i).rows(); j++) {
                    if (contours.get(i).get(j, 0)[1] >= ypix && Imgproc.contourArea(contours.get(i)) >= contourMinimumArea) {
                        xpix = contours.get(i).get(j, 0)[0];
                        ypix = contours.get(i).get(j, 0)[1];
                        contourArea = Imgproc.contourArea(contours.get(i));
                        contourIndex = i;
                    }
                }
            }
            Imgproc.circle(input, new Point(xpix, ypix), 2, new Scalar(0, 0, 255), 2);
            new File(circlePath + (frameNum % numberOfFrames) + ".jpg").delete();
            if (debug) Imgcodecs.imwrite(circlePath + (frameNum % numberOfFrames) + ".jpg", input);

            // Find Ellipse Using Contour Index
            Mat ellipseOnly = input.clone();
            RotatedRect ellipse;
            ellipse = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(contourIndex).toArray()));
            stoneTheta = Math.toRadians(ellipse.angle);
            Imgproc.ellipse(ellipseOnly, ellipse, new Scalar(0), 1);
            new File(ellipsePath + (frameNum % numberOfFrames) + ".jpg").delete();
            if (debug) Imgcodecs.imwrite(ellipsePath + (frameNum % numberOfFrames) + ".jpg", ellipseOnly);

            // Convert Local Coordinates to Field Coordinates
            xpix = xpix / (input.cols() / 2) - 1;
            ypix = 1 - ypix / input.rows();
            stoneY = -hpg * (Math.cos(-phi - verticalFOV / 2) / (2 * Math.sin(verticalFOV / 2)) + ypix * Math.sin(phi)) / (Math.sin(-phi - verticalFOV / 2) / (2 * Math.sin(verticalFOV / 2)) + ypix * Math.cos(phi));
            stoneX = Math.tan(horizontalFOV / 2) * xpix * Math.sqrt(Math.pow(hpg, 2) + Math.pow(stoneY, 2));

            // Throw Out Any Values Too Far or Too Close
            if (stoneY <= minimumYDistance || stoneY >= maximumYDistance || stoneY<0 || contourArea<contourMinimumArea) {
                stoneX = -1;
                stoneY = -1;
                stoneTheta = -1;
            }
        }

        // Log and Return Data
        time = timer.milliseconds();
        log("x: " + stoneX);
        log("y: " + (stoneY + 8));
        log("theta: " + stoneTheta);
        log("ms: " + time);

        return new double[] {stoneX, stoneY, stoneTheta};
    }
    
    /**
     * Gets current stone position value (x, y, theta)
     * @return current stone position value (x, y, theta)
     */
    public double[] getLocation() {return sPos;}

    /**
     * Gets current frame process time (ms)
     * @return current frame process time (ms)
     */
    public double getTime() {return time;}

    public double getArea() {return contourArea;}
    
    /**
     * Sets whether the stone locator is actively processing camera frames to locate stone
     * @param active true = processing; false = not processing
     */
    public void setActive(boolean active) {this.active = active;}

    private void telemetry(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }
    
    private void log(String message) {Log.w("opencv-sl", message);}
}