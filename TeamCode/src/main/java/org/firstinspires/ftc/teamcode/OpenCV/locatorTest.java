package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous(name = "OpenCV2") @Disabled
public class locatorTest extends LinearOpMode {

    private Robot robot;
    private double[] loc;
    private double time;
    private ElapsedTime timer = new ElapsedTime();

    private double x, y, theta, r;
    private double lastX, lastY, lastTheta;
    private double gotoX, gotoY, gotoTheta;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 0, 0, 0, true);
        stoneLocator2 locator = new stoneLocator2(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        locator.initializeCamera();
        locator.start();
        locator.setActive(true);
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        timer.reset();

        gotoX = 0;
        gotoY = 0;
        gotoTheta = 0;

        while (opModeIsActive() && !isStopRequested() && (!robot.stoneInRobot || timer.seconds() > 30)) {
            TelemetryPacket packet  = new TelemetryPacket();
            loc = locator.getLocation();
            time = locator.getTime();

            if (loc[0] != -1 && loc[1] != -1 && loc[2] != -1) {
                x = loc[0];
                y = loc[1] + 8;
                r = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
                theta = robot.drivetrain.currentheading;

                gotoX = robot.drivetrain.x + r*Math.cos(theta);
                gotoY = robot.drivetrain.y + r*Math.sin(theta);
                gotoTheta = theta;
            }

            lastX = x;
            lastY = y;
            lastTheta = theta;

            robot.drivetrain.setTargetPoint(gotoX, gotoY, gotoTheta);
            robot.update();

            packet.put("Area", locator.getArea());
            packet.put("Local X", x);
            packet.put("Local Y", y);
            packet.put("FPS", 1000 / time);
            packet.put("Target X", gotoX);
            packet.put("Target Y", gotoY);
            packet.put("Target Theta", gotoTheta);
            packet.put("robot x: ", robot.drivetrain.x);
            packet.put("robot y: ", robot.drivetrain.y);
            dashboard.sendTelemetryPacket(packet);

        }

        locator.setActive(false);
        locator.interrupt();
    }
}