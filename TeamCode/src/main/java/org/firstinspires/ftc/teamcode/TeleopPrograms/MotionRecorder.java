package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp @Disabled
public class MotionRecorder extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 9, 111, 0, false);
        robot.logger.startLogging();

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }

        robot.logger.stopLogging();
    }
}
