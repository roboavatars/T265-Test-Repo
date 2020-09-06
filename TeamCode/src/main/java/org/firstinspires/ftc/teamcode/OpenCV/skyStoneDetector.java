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
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * This class processes camera frames from {@linkplain FrameGrabber}
 * to determine the position of skystones in autonomous
 */

@SuppressWarnings({"FieldCanBeLocal"}) @SuppressLint({"DefaultLocale","SdCardPath"})
public class skyStoneDetector extends Thread {

    // Variables
    private final static String basePath = "/sdcard/FIRST/procFiles/";
    private final static String inputPath = "/sdcard/FIRST/input/";
    private final static String satFilteredPath = basePath + "sFiltered";
    private final static String openClosePath = basePath + "openClose";
    private final static String croppedPath = basePath + "croppedImage";
    private final static String verViewPath = basePath + "verticalAvg";
    private final static String testPath = "/sdcard/FIRST/testFiles/";

    private FrameGrabber frameGrabber;
    private final boolean usingCamera = true; // <<<----------------------
    private final boolean debug = true;
    private boolean active = false;

    private final static double numberOfFrames = 200;

    private final int horThreshold = 90;
    private final int verThreshold = 215;
    private final int magnificationFactor = 10;
    private final int binaryValue = 255;

    private int frameNum = 1;
    private double ssPos = -1;
    private double ssXPos = -1;
    //private double stoneSum = 0;
    private double curStoneCount;
    private double stoneLength;
    private boolean isRed = true;

    private LinearOpMode op;
    public skyStoneDetector(LinearOpMode opMode) {op = opMode;}

    // Phone Position-
    // 7in up, side closest to camera is 7.5in from left of robot (aligned to depot), slight tilt forward
    
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
    @Override
    public void run() {
        setName("OpenCV");

        // Clear Folder of Images
        File dir = new File(basePath);
        String[] children = dir.list();
        if (children != null) {
            for (String child : children) {
                new File(dir, child).delete();
            }
        }

        if (usingCamera) {
            frameGrabber = FtcRobotControllerActivity.frameGrabber;

            while (active) {
//                Mat input = frameGrabber.getNextMat();
                Mat input = new Mat();

                if (input != null) {
                    log("Frame " + frameNum + " ----------------------------------------");
                    ssPos = detectSkyStone(input);
                    log("SkyStone Position #: " + ssPos); log("SkyStone X Position: " + ssXPos);
                    frameNum++;
                } else ssPos = -1;
            }
            interrupt();
            //log("Avg stone is view: " + String.format("%.2f", stoneSum / frameNum));
        } else {
            Mat in = Imgcodecs.imread(inputPath + "input74.jpg", Imgcodecs.IMREAD_COLOR);
            Imgproc.resize(in, in, new Size(240, 180));
            ssPos = detectSkyStone(in);
        }
        log(" ");
    }

    @Override
    public void interrupt() {
        FtcRobotControllerActivity.disableCameraView();
        super.interrupt();
    }

    /**
     * Finds position value of skystone by processing an input frame
     * <p>Return Values: 1 = left, 2 = middle, 3 = right)
     * @param input the camera frame that will be used to detect skystones
     * @return position value of skystone
     */
    private double detectSkyStone (Mat input) {
        double ssPosValue = -1;
        //if (debug) {Imgcodecs.imwrite(inputPath + (frameNum % numberOfFrames) + ".jpg", input);}

        // Convert to HSV (Saturation)
        Mat HSV = new Mat();
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        List<Mat> hsvTypes = new ArrayList<>(3);
        Core.split(HSV, hsvTypes);
        Mat satUnfiltered = hsvTypes.get(1);

        // Filter Saturation Image
        Mat satFiltered = new Mat();
        Core.inRange(satUnfiltered, new Scalar(190, 120, 0), new Scalar(255, 135, 10), satFiltered);
        //if (debug) {Imgcodecs.imwrite(satFilteredPath + (frameNum % numberOfFrames) + ".jpg", satFiltered);}

        // Remove extra noise in image
        Mat openClose = new Mat();
        Imgproc.morphologyEx(satFiltered, openClose, Imgproc.MORPH_OPEN, new Mat());
        Imgproc.morphologyEx(openClose, openClose, Imgproc.MORPH_CLOSE, new Mat());
        //if (debug) {Imgcodecs.imwrite(openClosePath + (frameNum % numberOfFrames) + ".jpg", openClose);}

        // Crop Image to where quarry row is
        double horAvg;
        Mat SCropped = new Mat();
        for (int row = 0; row < openClose.rows(); row++) {
            horAvg = Core.mean(openClose.row(row)).val[0];
            if (horAvg > horThreshold) SCropped.push_back(openClose.row(row));
        }

        // Vertical Analysis
        if (!(SCropped.cols() == 0)) {
            Imgcodecs.imwrite(croppedPath + (frameNum % numberOfFrames) + ".jpg", SCropped);

            // Makes image black(skystone) and white(stone)
            //String preList = ""; // for printing pre-binary vertical averages
            //String postList = ""; // for printing post-binary vertical values
            double verAvg;
            Mat verImage = new Mat(10, SCropped.cols(), CvType.CV_8UC1);
            for (int col = 0; col < SCropped.cols(); col++) {
                verAvg = Core.mean(openClose.col(col)).val[0] * magnificationFactor; //preList += verAvg + " ";
                if (verAvg <= verThreshold) verAvg = 0;
                else verAvg = binaryValue;
                verImage.col(col).setTo(new Scalar(verAvg)); //postList += verAvg + " ";
            }
            Imgproc.morphologyEx(verImage, verImage, Imgproc.MORPH_OPEN, new Mat());
            if (debug) {Imgcodecs.imwrite(verViewPath + (frameNum % numberOfFrames) + ".jpg", verImage);}

            //log("pre-binary ver avg: " + preList);
            //log("post-binary ver val: " + postList);

            // Image Analyzing
            ArrayList<Double> darkAreas = new ArrayList<>();
            double left = 0, right = 0;
            double curIntensity;
            double nextIntensity = new Scalar(verImage.get(0, 0)).val[0];
            for (int c = 0; c < verImage.cols()-1; c++) {
                curIntensity = nextIntensity;
                nextIntensity = new Scalar(verImage.get(0, c+1)).val[0];

                if (curIntensity == binaryValue && nextIntensity == 0) {
                    left = c;
                }
                else if (curIntensity == 0 && nextIntensity == binaryValue) {
                    right = c;
                    double stoneCenter = (left + right) / 2;
                    stoneLength = right - left;
                    darkAreas.add(stoneCenter);
                    left = 0; right = 0;
                }
            }
            //log(darkAreas.size() + " Dark Areas: " + darkAreas);

            // Disregarding Small Columns
            double prevArea = 0;
            boolean firstStone = true;
            for (int a = 0; a < darkAreas.size(); a++) {
                double curArea = darkAreas.get(a);

                double areaDiff = Math.abs(curArea - prevArea);
                if (areaDiff < stoneLength && !firstStone) {
                    darkAreas.remove(a);
                    a--;
                }
                firstStone = false;

                prevArea = curArea;
            }
            log(darkAreas.size() + " Dark Areas: " + darkAreas); log("SkyStones Detected: " + darkAreas.size());
            curStoneCount = darkAreas.size(); //stoneSum += curStoneCount;

            // Converting X Coordinates to Positions
            if (!(darkAreas.size() == 0)) {

                if (isRed) {
                    ssXPos = darkAreas.get(0);
                    if (ssXPos > 50 && ssXPos < 105) {ssPosValue = 1;} // left
                    else if((ssXPos > 105 && ssXPos < 165)) {ssPosValue = 2;} // middle
                    else if ((ssXPos > 10 && ssXPos < 50) || (ssXPos > 165 && ssXPos < 230)) {ssPosValue = 3;} // right
                } else {
                    if (darkAreas.size() > 1) {ssXPos = darkAreas.get(1);}
                    else {ssXPos = darkAreas.get(0);}

                    if ((ssXPos > 25 && ssXPos < 50) || (ssXPos > 175 && ssXPos < 240)) {ssPosValue = 3;} // left
                    else if((ssXPos > 60 && ssXPos < 120)) {ssPosValue = 2;} // middle
                    else if (ssXPos > 120 && ssXPos < 175) {ssPosValue = 1;} // right
                }

            } else log("Cannot Determine SkyStone Position :-(");
        } else log("Quarry Row Not Detected :-(");

        //log("Stone Length: " + stoneLength);
        return ssPosValue;
    }
    
    /**
     * Gets current skystone position value (1 = left, 2 = middle, 3 = right)
     * @return current skystone position value
     */
    public double getPosition() {return ssPos;}
    
    /**
     * Gets current skystone x coordinate value of the first skystone in view (0-240)
     * @return current skystone x coordinate
     */
    public double getSSPosX() {return ssXPos;}
    
    /**
     * Gets total amount of skystones in the camera view
     * @return number of skystones
     */
    public double getNumberOfStones() {return curStoneCount;}
    
    /**
     * Sets whether the skystone detector is actively processing camera frames to locate skystones
     * @param active true = processing; false = not processing
     */
    public void setActive(boolean active) {this.active = active;}
    
    /**
     * Sets the alliance color
     * @param isRed true = red alliance; false = blue alliance
     */
    public void isAllianceRed(boolean isRed) {this.isRed = isRed;}
    
    private void telemetry2(String caption, String value) {
        op.telemetry.addData(caption, value);
        op.telemetry.update();
    }
    
    private void log(String message) {Log.w("opencv-ssd", message);}
}