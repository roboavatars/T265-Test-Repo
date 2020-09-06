package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp
public class MecanumTest extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this,8.625, 111, 0, true);
//        robot.intakeManual = true;
//        robot.intake.setControls(0);
        waitForStart();
        robot.yeetmode = true;
        double foundationPullTime = 2;
//        Waypoint[] foundationPullWaypoints = new Waypoint[] {
//                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, 5, 30,0, 0),
//                new Waypoint(33, 35, 5*Math.PI/6 , 30, 30,0,0.75),
////                            new Waypoint(27, 35, 3*Math.PI / 4, 40, 10,-2, 1.25),
//                new Waypoint(33, 60, Math.PI/2 - 0.2, 60, 10,0, foundationPullTime)
//        };
//        Path foundationPullPath = new Path(new ArrayList<>(Arrays.asList(foundationPullWaypoints)));
        ElapsedTime time = new ElapsedTime();
        while(opModeIsActive()){
//            double currentTime = Math.min(foundationPullTime, time.seconds());
//            Pose robotPose = foundationPullPath.getRobotPose(currentTime);
//            if(robot.drivetrain.y>45){
//                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta(), 0.3, 0.3, 2.4);
//            }else{
//                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta(), 0.3, 0.3, 2.4);
//            }
////            robot.drivetrain.setControls(0,0, -1.0);
//            robot.drivetrain.setTargetPoint(30, 78, 0);


            robot.update();

            Log.w("auto", String.format("%.5f", robot.drivetrain.x) + " " + String.format("%.5f", robot.drivetrain.y) + " " + String.format("%.5f", robot.drivetrain.currentheading));
        }
    }
}
