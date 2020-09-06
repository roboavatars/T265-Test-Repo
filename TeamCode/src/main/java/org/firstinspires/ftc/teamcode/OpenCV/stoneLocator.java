package org.firstinspires.ftc.teamcode.OpenCV;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This class processes camera frames from {@linkplain FrameGrabber}
 * to determine the location of stones in teleop
 */

@SuppressWarnings({"FieldCanBeLocal"}) @SuppressLint({"DefaultLocale","SdCardPath"})
public class stoneLocator extends Thread {

    // Variables
    private final static String basePath = "/sdcard/FIRST/procFiles2/";
    private final static String inputPath = basePath + "input";
    private final static String filteredPath = basePath + "filtered";
    private final static String ellipsePath = basePath + "ellipse";
    private final static String linesPath = basePath + "lines";
    private final static String intersectionsPath = basePath + "intersections";
    private final static String testPath = "/sdcard/FIRST/testFiles2/";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true; // <<<----------------------
    private final boolean debug = true;

    private int frameNum = 1;
    private double[] sPos = {-1, -1, -1};

    private boolean active = false;

    private LinearOpMode op;
    public stoneLocator(LinearOpMode opMode) {op = opMode;}

    // Phone Position-
    // 7in up, side closest to camera is 7.5in from left of robot (aligned to depot), slight tilt forward

    // difference between 2 skystones is 170
    
    /**
     * Enables the camera view
     */
    public void initializeCamera() {
//        telemetry2("Initializing OpenCV", "v" + OpenCVLoader.OPENCV_VERSION);
        if (usingCamera) FtcRobotControllerActivity.enableCameraView();
        telemetry2("Status", "Ready");
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
            Mat input = Imgcodecs.imread(testPath + "test5.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(240, 180));
            sPos = detectSkyStone(input);
            frameNum++;

            input = Imgcodecs.imread(testPath + "test6.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(240, 180));
            sPos = detectSkyStone(input);
            frameNum++;

            input = Imgcodecs.imread(testPath + "test7.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(input, input, new Size(240, 180));
            sPos = detectSkyStone(input);
        }
        log("");
    }
    
    /**
     * Finds position value of stone by processing an input frame
     * @param input the camera frame that will be used to detect skystones
     * @return x, y, and theta of stone
     */
    private double[] detectSkyStone (Mat input) {
        if (debug) {Imgcodecs.imwrite(inputPath + (frameNum % 100) + ".jpg", input);}

        // Process Image
        Mat filtered = new Mat(180, 240, CvType.CV_8UC1, new Scalar(255));
        Imgproc.cvtColor(input, filtered, Imgproc.COLOR_RGB2HSV);
        Core.inRange(filtered, new Scalar(85, 90, 90), new Scalar(115, 255, 255), filtered);
        Imgproc.morphologyEx(filtered, filtered, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(filtered, filtered, Imgproc.MORPH_CLOSE, new Mat());
        Imgproc.Canny(filtered, filtered, 0, 0, 3, false);
        if (debug) Imgcodecs.imwrite(filteredPath + (frameNum % 100) + ".jpg", filtered);

        // Define Stuff for Finding Contours
        Mat heirarchyMat = new Mat();
        Mat ellipseOnly = input.clone();
        RotatedRect ellipse = new RotatedRect();
        List<MatOfPoint> contours = new ArrayList<>();
        double max = 0;
        int maxIndex = 0;
        //Mat largestContour = new Mat();
        //Mat largestContour2 = new Mat(180, 240, CvType.CV_8UC3);

        // Find Largest Contour By Area and Draw Ellipse
        Imgproc.findContours(filtered, contours, heirarchyMat, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > max) {
                max = Imgproc.contourArea(contours.get(i));
                maxIndex = i;
            }
        }
        if (contours.size() > 0) {ellipse = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(maxIndex).toArray()));}
        else {return new double[] {-1, -1, -1};}
        Imgproc.ellipse(ellipseOnly, ellipse, new Scalar(255, 0, 0), 1);
        if (debug) Imgcodecs.imwrite(ellipsePath + (frameNum % 100) + ".jpg", ellipseOnly);

        /*Imgproc.drawContours(largestContour2, contours, -1, new Scalar(0, 255, 0));
        if (debug) Imgcodecs.imwrite(ellipsePath + (frameNum + 7 % 100) + ".jpg", largestContour2);

        if (debug) Imgcodecs.imwrite(ellipsePath + (frameNum % 100) + ".jpg", ellipseOnly);
        largestContour = contours.get(maxIndex);
        for (int i = 0; i < largestContour.cols(); i++) {
            for (int j = 0; j < largestContour.rows(); j++) {
                log(new Scalar(largestContour.get(j, i)) + "");
            }
        }
        log("contour: " + largestContour);*/
        //Imgcodecs.imwrite("contours6.jpg", largestContour);

        // Get and Process Lines
        Mat lines = new Mat();
        Imgproc.HoughLines(filtered, lines, 1, Math.PI/180, 20, 0, 0, 0, Math.PI);

        ArrayList<Double> radii1 = new ArrayList<>();
        ArrayList<Double> thetas1 = new ArrayList<>();
        ArrayList<Double> radii2 = new ArrayList<>();
        ArrayList<Double> thetas2 = new ArrayList<>();
        for (int i = 0; i < lines.rows(); i++) {
            radii1.add(lines.get(i, 0)[0]);
            thetas1.add(lines.get(i, 0)[1]);
        }
        for (int i = 0; i < lines.rows() - 1; i++) {
            if ((Math.abs(radii1.get(1) - radii1.get(i + 1)) < 5) || (Math.abs(1 / Math.tan(thetas1.get(i + 1)) - 1 / Math.tan(thetas1.get(i))) < 0.3)) {
                radii2.add(radii1.get(i));
                thetas2.add(thetas1.get(i));
            }
        }

        log("Radii: " + radii2);
        log("Thetas: " + thetas2);

        // Draw Lines
        //Mat edges = new Mat(180, 240, CvType.CV_8UC3, new Scalar(255, 255, 255));
        Mat edges = input.clone();

        for (int i = 0; i < radii2.size(); i++) {
            Point pt1 = new Point();
            Point pt2 = new Point();

            double radius = radii2.get(i);
            double theta = thetas2.get(i);

            double x = radius * Math.cos(theta);
            double y = radius * Math.sin(theta);
            pt1.x = Math.round(x - 300 * Math.sin(theta));
            pt1.y = Math.round(y + 300 * Math.cos(theta));
            pt2.x = Math.round(x + 300 * Math.sin(theta));
            pt2.y = Math.round(y - 300 * Math.cos(theta));

            Imgproc.line(edges, pt1, pt2, new Scalar(0, 255, 0), 1);
        }

        if (debug) Imgcodecs.imwrite(linesPath + (frameNum % 100) + ".jpg", edges);

        // Generate Index Combinations to Find Intersection Points
        int[] indexes = new int[radii2.size()];
        ArrayList<ArrayList<Integer>> combinationsList = new ArrayList<>();
        for (int i = 0; i < radii2.size(); i++) {indexes[i] = i;}
        combinations(indexes, indexes.length, 0, new int[2], 0, combinationsList);

        // Find Intersection Points
        ArrayList<Double> xf1 = new ArrayList<>();
        ArrayList<Double> yf1 = new ArrayList<>();

        for (int i = 0; i < combinationsList.size(); i++) {
            int index1 = combinationsList.get(i).get(0);
            int index2 = combinationsList.get(i).get(1);

            double x1 = radii2.get(index1) * Math.cos(thetas2.get(index1));
            double y1 = radii2.get(index1) * Math.sin(thetas2.get(index1));
            double m1 = -1 / Math.tan(thetas2.get(index1));
            double b1 = y1 - m1 * x1;

            double x2 = radii2.get(index2) * Math.cos(thetas2.get(index2));
            double y2 = radii2.get(index2) * Math.sin(thetas2.get(index2));
            double m2 = -1 / Math.tan(thetas2.get(index2));
            double b2 = y2 - m2 * x2;

            double xf = (b2 - b1) / (m1 - m2);
            double yf = m1 * xf + b1;

            xf1.add(xf);
            yf1.add(yf);
        }

        // Find Block Corner
        ArrayList<Double> xf2 = new ArrayList<>();
        ArrayList<Double> yf2 = new ArrayList<>();
        max = 0;
        maxIndex = 0;

        for (int i = 0; i < xf1.size(); i++) {
            if ((xf1.get(i) >= 0) && (xf1.get(i) <= 240) && (yf1.get(i) >= 0) && (yf1.get(i) <= 180)) {
                xf2.add(xf1.get(i));
                yf2.add(yf1.get(i));
                Imgproc.circle(edges, new Point(xf1.get(i), yf1.get(i)), 2, new Scalar(0, 255, 0), 2);
            }

            if (yf1.get(i) > max) {
                max = yf1.get(i);
                maxIndex = i;
            }
        }

        log("xf2: " + xf2);
        log("yf2: " + yf2);
        log(maxIndex + ": " + xf1.get(maxIndex) + ", " + yf1.get(maxIndex));
        Imgproc.circle(edges, new Point(xf1.get(maxIndex), max), 3, new Scalar(0, 0, 255), 3);
        if (debug) Imgcodecs.imwrite(intersectionsPath + (frameNum % 100) + ".jpg", edges);

        return new double[] {0,0,0};
    }
    
    /**
     * Gets current stone position value (x, y, theta)
     * @return current stone position value
     */
    public double[] getLocation() {return sPos;}
    
    /**
     * Sets whether the skystone detector is actively processing camera frames to locate skystones
     * @param active true = processing; false = not processing
     */
    public void setActive(boolean active) {this.active = active;}

    private void telemetry2(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }
    
    private void log(String message) {Log.w("opencv-sl", message);}

    private static void combinations(int indexes[], int n, int index, int data[], int i, ArrayList<ArrayList<Integer>> finalList) {
        if (index == 2) {
            finalList.add(new ArrayList<>(Arrays.asList(data[0], data[1])));
            return;
        }

        if (i >= n) return;

        data[index] = indexes[i];
        combinations(indexes, n,index + 1, data, i + 1, finalList);
        combinations(indexes, n, index, data, i + 1, finalList);
    }
}