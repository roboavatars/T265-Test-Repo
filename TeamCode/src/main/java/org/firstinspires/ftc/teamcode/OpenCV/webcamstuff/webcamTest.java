package org.firstinspires.ftc.teamcode.OpenCV.webcamstuff;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Webcam")
public class webcamTest extends LinearOpMode {

    private final static String basePath = "/sdcard/FIRST/input/";

    private final int width = 320;
    private final int height = 240;
    private final int horThreshold = 22;
    private final int verLowThreshold = 30;
    private final int verHighThreshold = 180;
    private final int widthThreshold = 80;

    private double position;
    private double stoneLength;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new AutoStackPipeline());
        webcam.startStreaming(width, height);
        telemetry.addData("Status", "Ready"); telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Position", position);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    class AutoStackPipeline extends OpenCvPipeline {
        // Variables
        Mat HSV = new Mat();
        Mat satUnfiltered = new Mat(width, height, CvType.CV_8UC1);
        Mat satFiltered = new Mat(width, height, CvType.CV_8UC1);
        Mat openClose = new Mat(width, height, CvType.CV_8UC1);
        List<Mat> hsvTypes = new ArrayList<>(3);
        double verAvg;
        Mat verImage = new Mat(width, height, CvType.CV_8UC1);
        double horAvg;
        Mat horImage = new Mat(width, height, CvType.CV_8UC1);
        ArrayList<Double> darkAreas = new ArrayList<>();
        double curIntensity;

        @Override
        public Mat processFrame(Mat input) {
            // Image Processing
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Core.split(HSV, hsvTypes);
            satUnfiltered = hsvTypes.get(1);
            Core.inRange(satUnfiltered, new Scalar(190, 120, 0), new Scalar(255, 135, 10), satFiltered);
            Imgproc.morphologyEx(satFiltered, openClose, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(openClose, openClose, Imgproc.MORPH_CLOSE, new Mat());

//            // Vertical Average
//            for (int col = 0; col < satFiltered.cols(); col++) {
//                verAvg = Core.mean(satFiltered.col(col)).val[0];
//                if (verAvg >= verLowThreshold && verAvg <= verHighThreshold){
//                    verImage.col(col).setTo(new Scalar(255));
//                } else{
//                    verImage.col(col).setTo(new Scalar(0));
//                }
//            }
//            Imgproc.morphologyEx(verImage, verImage, Imgproc.MORPH_OPEN, new Mat());

            // Horizontal Average
            for (int row = 0; row < satFiltered.rows(); row++) {
                horAvg = Core.mean(satFiltered.row(row)).val[0];
                if (horAvg >= horThreshold) {
                    horImage.row(row).setTo(new Scalar(255));
                } else {
                    horImage.row(row).setTo(new Scalar(0));
                }
            }
            Imgproc.morphologyEx(horImage, horImage, Imgproc.MORPH_OPEN, new Mat());

            // Find Left and Right Boundaries of Rows
            double left = 0, right = 0;
            double nextIntensity = new Scalar(horImage.get(0, 0)).val[0];
            for (int c = 0; c < horImage.cols() - 1; c++) {
                curIntensity = nextIntensity;
                nextIntensity = new Scalar(horImage.get(0, c+1)).val[0];

                if (curIntensity == 255 && nextIntensity == 0) {
                    left = c;
                } else if (curIntensity == 0 && nextIntensity == 255) {
                    right = c;
                    double stoneCenter = (left + right) / 2;
                    stoneLength = right - left;
                    darkAreas.add(stoneCenter);
                    left = 0; right = 0;
                }
            }

            // Find Row Relating to Stack
            double prevArea = 0;
            for (int a = 0; a < darkAreas.size(); a++) {
                double curArea = darkAreas.get(a);
                double areaDiff = Math.abs(curArea - prevArea);

                if (areaDiff < widthThreshold) {
                    darkAreas.remove(a);
                    a--;
                }
                prevArea = curArea;
            }

            // Save Image
            Mat output = new Mat(width, height, CvType.CV_8UC3);
            input.copyTo(output, horImage);
            //Imgproc.line(output, new Point(darkAreas.get(0), 0), new Point(darkAreas.get(0), height - 1), new Scalar(0, 255, 0), 3);
            if (!(darkAreas.size() == 0)) {
                output.row((int) Math.round(darkAreas.get(0)) - 1).setTo(new Scalar(0, 255, 0));
                output.row((int) Math.round(darkAreas.get(0))).setTo(new Scalar(0, 255, 0));
                output.row((int) Math.round(darkAreas.get(0)) + 1).setTo(new Scalar(0, 255, 0));

                position = darkAreas.get(0);
            } else {
                position = -1;
            }

            return output;
        }
    }
}