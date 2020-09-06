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

@Autonomous(name = "4 block blue NEW!!!!!!!!!!!!!!!")
public class fourBlockBlueNew extends LinearOpMode {

    private Robot robot;
    private skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() {

        // Initialize Skystone Detector
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(false);

        // Initialize Robot
        robot = new Robot(this, 133.375, 110, Math.PI, false);
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
            skystoneY = 111.5;
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
                {new Point2D(57,109), new Point2D(47,129)},
                {new Point2D(47,120), new Point2D(47,129)}
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

                    //defining the tofoundation path
                    Waypoint[] toFoundation1PathWaypoints = new Waypoint[] {
                            new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(31, skystoneY - 25, Math.PI / 2.1, -50, -50,0, 1),
                            new Waypoint(28, 35, Math.PI / 2, -30, -30,0, 2),
                            new Waypoint(45, 30, Math.PI, -25, 100,0, toFoundation1Time)
                    };
                    toFoundation1Path = new Path(new ArrayList<>(Arrays.asList(toFoundation1PathWaypoints)));

                    //reset time
                    time.reset();
                }
            }

            // Go to Foundation to Deposit First Skystone
            else if (!toFoundation1) {
                double currentTime = Math.min(toFoundation1Time, time.seconds());
                Pose robotPose = toFoundation1Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(),robotPose.getY(),robotPose.getTheta()+Math.PI);

//                if (robot.drivetrain.x>42 && time.seconds()>toFoundation1Time-0.8 && robot.drivetrain.y<40) {
//                    robot.grabber.grabFoundation();
//                }
                if (time.seconds() > toFoundation1Time && robot.drivetrain.x < 107) {
                    robot.grabber.grabFoundation();
                }
//                if(robot.drivetrain.y>70 && robot.drivetrain.y<80){
//                    robot.cheesemode = true;
//                }
                if (time.seconds() > (toFoundation1Time + 1)) {
                    toFoundation1 = true;
                    robot.depositAuto();
                    Waypoint[] foundationPullWaypoints = new Waypoint[] {
                            new Waypoint(142 - robot.drivetrain.x,robot.drivetrain.y,Math.PI - robot.drivetrain.currentheading, 1, 30,0, 0),
                            new Waypoint(32, 44, 3*Math.PI/4, 10, 80,0,0.3),
                            new Waypoint(28, 57, Math.PI/2 - 0.2, 50, 10,0, foundationPullTime)
                    };
                    foundationPullPath = new Path(new ArrayList<>(Arrays.asList(foundationPullWaypoints)));
//                    robot.drivetrain.zerostrafeCorrection = true;
                    time.reset();
                }
            }

            // Turn and Pull Foundation
            else if (!foundationPull) {
//                if (robot.drivetrain.x > 38 && robot.drivetrain.currentheading > 2*Math.PI/3 ) {
//                    robot.drivetrain.setControls(0.8, 0, 0);
//                } else if (robot.drivetrain.currentheading > 2*Math.PI/3) {
//
//                } else {
//                    robot.drivetrain.setTargetPointAuto(33,58,Math.PI/2, 0.2, 0.2, 0.4);
//                }
                double currentTime = Math.min(foundationPullTime, time.seconds());
                Pose robotPose = foundationPullPath.getRobotPose(currentTime);
                if (time.seconds() > 0.75 && time.seconds() < 1.5) {
                    robot.drivetrain.setTargetPointAuto(robotPose.getX(),robotPose.getY(),robotPose.getTheta(), 0.1,0.1, 2.8);
                }
//                else if(time.seconds()>1.5){
//                    robot.drivetrain.zerostrafeCorrection = false;
//                }
                else{
                    robot.drivetrain.setTargetPointAuto(robotPose.getX(),robotPose.getY(),robotPose.getTheta(), 0.3,0.3, 2.8);
                }

//                if(robot.drivetrain.currentheading > 2*Math.PI/3 ){
//                    robot.drivetrain.setControls(0.54,0,-0.38);
//                }
//                else{
//                    robot.drivetrain.setTargetPointAuto(33,58,Math.PI/2, 0.2, 0.2, 0.4);
//                }


                if (robot.stacker.isArmOut() && !robot.stacker.isArmMoving()) {
                    robot.letGo = true;
                }

                if (/*time.seconds() > foundationPullTime + 1*/robot.drivetrain.y > 50 && robot.stacker.getArmPosition() < 180) {
                    robot.grabber.releaseFoundation();
                    foundationPull = true;
                    depositX = robot.drivetrain.x; depositY = robot.drivetrain.y; depositTheta = robot.drivetrain.currentheading;
                    Log.w("auto", "Deposit Cor: " + depositX + " " + depositY + " " + depositTheta);

                    Waypoint[] skystone2PathWaypoints = new Waypoint[] {
                            new Waypoint(142 - robot.drivetrain.x,robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, 10, 75,0, 0),
                            new Waypoint(31, skystoneY - 33, Math.PI / 3, 30, 10,-3, 1.25),
                            new Waypoint(47, skystoneY - 24, Math.PI/4, 10, -100,0, skystone2Time)
                    };
                    skystone2Path = new Path(new ArrayList<>(Arrays.asList(skystone2PathWaypoints)));
                    robot.yeetmode = true;
                    robot.drivetrain.zerostrafeCorrection = false;
                    time.reset();
                }
            }

            // Get Second Skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystone2Time, time.seconds());

                if (time.seconds()< (skystone2Time + 0.5)) {
                    Pose robotPose = skystone2Path.getRobotPose(currentTime);
                    robot.drivetrain.setTargetPointAuto(robotPose.getX(),robotPose.getY(),robotPose.getTheta());
                }
                else {
                    robot.drivetrain.setTargetPointAuto(50,skystoneY-18, Math.PI/4 + 0.25);
                }

                if (time.seconds() > (skystone2Time + 2) || (robot.stoneInRobot && robot.drivetrain.y > 72)) {
                    skystone2 = true;
                    Waypoint[] toFoundation2PathWaypoints = new Waypoint[] {
                            new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(33, skystoneY - 35, Math.PI / 2, -50, -10,0, 0.75),
                            new Waypoint(33, 57, Math.PI / 2, -5, 100,0, toFoundation2Time)
                    };
                    toFoundation2Path = new Path(new ArrayList<>(Arrays.asList(toFoundation2PathWaypoints)));
                    time.reset();
                }
            }

            // Go to Foundation to Deposit Second Skystone
            else if (!toFoundation2) {
                double currentTime = Math.min(toFoundation2Time, time.seconds());
                Pose robotPose = toFoundation2Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(),robotPose.getY(),robotPose.getTheta()+Math.PI);

                if (robot.drivetrain.y < 72) {
                    robot.depositAuto();
                }
//                if(robot.drivetrain.y>80 && robot.drivetrain.y<120){
//                    robot.cheesemode = true;
//                }

                if (time.seconds() > (toFoundation2Time + 0.5) && robot.stacker.getArmPosition() < 180) {
                    toFoundation2 = true;
                    Waypoint[] stone3PathWaypoints;
                    if (skystonePos == 1) {
                        stone3PathWaypoints = new Waypoint[] {
                                new Waypoint(142 - robot.drivetrain.x,robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, 10, 100,0, 0),
                                new Waypoint(31, stoneLocations[skystonePos-1][0].getY() - 18, Math.PI / 3, 30, 10,-3, 1.33),
                                new Waypoint(stoneLocations[skystonePos-1][0].getX(), stoneLocations[skystonePos-1][0].getY(), Math.PI/4-0.2, 10, -100,0, stone3Time)
                        };
                    }
                    else if (skystonePos == 2) {
                        stone3PathWaypoints = new Waypoint[] {
                                new Waypoint(142 - robot.drivetrain.x,robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, 10, 100,0, 0),
                                new Waypoint(31, stoneLocations[skystonePos-1][0].getY() - 18, Math.PI / 3, 30, 10,-3, 0.75),
                                new Waypoint(stoneLocations[skystonePos-1][0].getX(), stoneLocations[skystonePos-1][0].getY(), Math.PI/4, 10, -100,0, stone3Time)
                        };
                    }
                    else { //case 3
                        stone3PathWaypoints = new Waypoint[]{
                                new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][1].getY() - 30, Math.PI / 3, 30, 10, -3, 1.0),
                                new Waypoint(stoneLocations[skystonePos - 1][1].getX(), stoneLocations[skystonePos - 1][1].getY(), Math.PI/2, 10, -100, 0, stone3Time)
                        };
                    }
                    stone3Path = new Path(new ArrayList<>(Arrays.asList(stone3PathWaypoints)));
                    time.reset();

                }
            }

            // Get Third stone
            else if (!stone3) {
                double currentTime = Math.min(stone3Time, time.seconds());
                Pose robotPose = stone3Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(), robotPose.getY(), robotPose.getTheta());

                if (time.seconds() > (stone3Time + 2) || (robot.stoneInRobot && robot.drivetrain.y > 72)) {
                    stone3 = true;
                    Waypoint[] toFoundation3PathWaypoints = new Waypoint[] {
                            new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(31, stoneLocations[skystonePos-1][0].getY() - 18, Math.PI / 2, -50, -10,0, 0.75),
                            new Waypoint(33, 57, Math.PI / 2, -5, 50,0, toFoundation3Time)
                    };
                    toFoundation3Path = new Path(new ArrayList<>(Arrays.asList(toFoundation3PathWaypoints)));
                    time.reset();
                }
            }

            // Go to Foundation to Deposit Third Stone
            else if (!toFoundation3) {
                double currentTime = Math.min(toFoundation3Time, time.seconds());

                Pose robotPose = toFoundation3Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(), robotPose.getY(), robotPose.getTheta()+Math.PI);
//                if(robot.drivetrain.y>80 && robot.drivetrain.y<120){
//                    robot.cheesemode = true;
//                }
                if (robot.drivetrain.y < 70) {
                    robot.depositAuto();
                }
                if (time.seconds() > (toFoundation3Time + 0.75) && robot.stacker.getArmPosition() < 180) {
                    toFoundation3 = true;
                    Waypoint[] stone4PathWaypoints;
                    if (skystonePos == 2 || skystonePos == 3) {
                        stone4PathWaypoints = new Waypoint[]{
                                new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][1].getY() - 26, Math.PI / 3, 30, 10, -3, 1.33),
                                new Waypoint(stoneLocations[skystonePos - 1][1].getX(), stoneLocations[skystonePos - 1][1].getY(), Math.PI / 2, 10, -100, 0, stone4Time)
                        };
                    }
                    else { //pos 1
                        stone4PathWaypoints = new Waypoint[]{
                                new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI -robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][1].getY() - 18, Math.PI / 3, 30, 10, -3, 1.33),
                                new Waypoint(stoneLocations[skystonePos - 1][1].getX(), stoneLocations[skystonePos - 1][1].getY(), Math.PI / 4, 10, -100, 0, stone4Time)
                        };
                    }
                    stone4Path = new Path(new ArrayList<>(Arrays.asList(stone4PathWaypoints)));
                    time.reset();
                }
            }

            // Get Fourth Stone
            else if (!stone4) {
                double currentTime = Math.min(stone4Time, time.seconds());
                Pose robotPose = stone4Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(), robotPose.getY(), robotPose.getTheta());

                if (time.seconds() > (stone4Time + 2) || (robot.stoneInRobot && robot.drivetrain.y > 72)) {
                    stone4 = true;
                    Waypoint[] toFoundation4PathWaypoints = new Waypoint[] {
                            new Waypoint(142 - robot.drivetrain.x, robot.drivetrain.y, Math.PI - robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(33, stoneLocations[skystonePos-1][1].getY() -3, Math.PI / 2, -50, -10,0, 0.7),
                            new Waypoint(33, 20, Math.PI / 2, -20, 100,0, toFoundation4Time)
                    };
                    toFoundation4Path = new Path(new ArrayList<>(Arrays.asList(toFoundation4PathWaypoints)));
                    time.reset();
                }
            }

            // Go to Foundation to Deposit Fourth Stone
            else if (!toFoundation4) {
                double currentTime = Math.min(toFoundation4Time, time.seconds());

                Pose robotPose = toFoundation4Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPointAuto(robotPose.getX(), robotPose.getY(), robotPose.getTheta()+Math.PI, 0.11, 0.11, 1.5);

                if (robot.drivetrain.y < 70) {
                    robot.depositAuto();
                }
//                if(robot.drivetrain.y>80 && robot.drivetrain.y<120){
//                    robot.cheesemode = true;
//                }
                if (time.seconds() > (toFoundation4Time + 2) && robot.stacker.getArmPosition() < 180 && !robot.stacker.stoneClamped) {
                    toFoundation4 = true;
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