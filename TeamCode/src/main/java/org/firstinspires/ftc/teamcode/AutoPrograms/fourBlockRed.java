package org.firstinspires.ftc.teamcode.AutoPrograms;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.Waypoint;
import org.firstinspires.ftc.teamcode.iLQR.Point2D;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "4 block red")
public class fourBlockRed extends LinearOpMode {

    private Robot robot;
    private skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() {

        // Initialize Skystone Detector
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(true);

        // Initialize Robot
        robot = new Robot(this, 8.625, 110, 0, true);
        robot.logger.startLogging();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        // Segment Finished Variables
        boolean skystone1 = false;
        boolean toFoundation1 = false;
        boolean foundationPull = false;
        boolean skystone2 = false;
        boolean toFoundation2 = false;
        boolean stone3 = false;
        boolean toFoundation3 = false;
        boolean stone4 = false;
        boolean toFoundation4 = false;
        boolean toTape = false;

        // deposit position
        double depositX, depositY, depositTheta;

        // After Start
        waitForStart();

        // Skystone Position Variables
        int skystonePos = (int) detector.getPosition();
        double skystoneY;
        if (skystonePos == 1) {
            skystoneY = 128;
        }
        else if (skystonePos == 2) {
            skystoneY = 121;
        }
        else {
            skystoneY = 113;
            skystonePos = 3; // if pos is -1
        }

        // Path time variables
        // stone 1 times
        double skystone1Time = 1.5;
        double toFoundation1Time = 3.75;
        double foundationPullTime = 1.5;
        double skystone2Time = 1.5;
        double toFoundation2Time = 1.75;
        double stone3Time = 1.75;
        double toFoundation3Time = 1.5;
        double stone4Time = 1.75;
        double toFoundation4Time = 2.25;

        if (skystonePos == 2) {
            skystone1Time = 1.5;
            toFoundation1Time = 3;
            skystone2Time = 1.75;
            toFoundation2Time = 1.75;
            stone3Time = 1.65;
            toFoundation3Time = 1.75;
            stone4Time = 2;
            toFoundation4Time = 2.25;

        } else if(skystonePos == 3) {
            skystone1Time = 1.25;
            toFoundation1Time = 2.75;
            skystone2Time = 1.5;
            toFoundation2Time = 1.5;
            stone3Time = 1.75;
            toFoundation3Time = 2.25;
            stone4Time = 1.75;
            toFoundation4Time = 2.25;
        }

        // stone 3-5 locations
        Point2D[][] stoneLocations = {
                {new Point2D(56,97), new Point2D(57,118)},
                {new Point2D(55,112), new Point2D(47,129)},
                {new Point2D(47,110), new Point2D(47,129)}
        };

        // stop detector
        detector.setActive(false);

        // Defining paths
        Waypoint[] skystone1PathWaypoints = {
                new Waypoint(9,111,0,20,100,0,0),
                new Waypoint(47,skystoneY,Math.PI / 4 + 0.2, 15, -95, 0, skystone1Time)
        };
        Path skystone1Path = new Path(new ArrayList<>(Arrays.asList(skystone1PathWaypoints)));
        Spline skystone1ThetaSpline = new Spline(Math.PI/6 , 2, 8, Math.PI / 4 + 0.2, 0, 0, skystone1Time);

        Path toFoundation1Path = null;
        Path foundationPullPath = null;
        Path skystone2Path = null;
        Path toFoundation2Path = null;
        Path stone3Path = null;
        Path toFoundation3Path = null;
        Path stone4Path = null;
        Path toFoundation4Path = null;

        // Time Used to End Segments After a Certain Period of Time
        ElapsedTime time = new ElapsedTime();

        robot.intake.setControls(robot.intakePower);
        sleep(33);

        // Robot Move Loop
        while (opModeIsActive()) {

            // Update Robot's Location and States
            robot.update();

            // Get the First Skystone
            if (!skystone1) {
                double currentTime = Math.min(skystone1Time, time.seconds());
                Pose robotPose = skystone1Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(),robotPose.getY(),skystone1ThetaSpline.position(currentTime));

                if (time.seconds() > 0.5) {
                    robot.grabber.releaseFoundation();
                }
                if (robot.stoneInRobot || time.seconds() > (skystone1Time + 2)) {
                    //setting variable to move on from this segment
                    skystone1 = true;
                    time.reset();
                }
            }


            // Park Under Skybridge
            else if (!toTape) {
                robot.drivetrain.setTargetPointAuto(33, 72, Math.PI / 2);
            }
            else {
                robot.drivetrain.setControls(0,0,0);
            }

            //adding relevant information to telemetry
//            robot.addPacket("Battery Voltage", robot.getBatteryVoltage());
            robot.addPacket("Time", (System.currentTimeMillis()-robot.startTime)/1000);
            robot.addPacket("Skystone Position", skystonePos);
        }

        robot.update();
        robot.logger.stopLogging();
    }

    public void log(String message) {
        Log.w("auto", message);
    }
}