package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp @Disabled
public class normalFeeder extends LinearOpMode {
    private Robot robot;
    private boolean robotCentric = true;
    private boolean dpadUp = true;
    private boolean dpadDown = true;
    private boolean rightBumper = true;
    private boolean leftBumper = true;
    private boolean a = true;

    @Override
    public void runOpMode() {
        double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition)); telemetry.update();
        robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2], false);
        robot.logger.startLogging();

        waitForStart();
        robot.stacker.unClampStone();
        robot.stacker.goHome();
        ElapsedTime time = new ElapsedTime();
        robot.intakeManual = true;
        while (opModeIsActive()) {


            if (gamepad1.right_bumper) {
                robot.intake.setControls(0.6);
            }
            else if(gamepad1.left_bumper){
                robot.intake.setControls(-0.3);
            }
            else{
                robot.intake.setControls(0);
            }
            robot.drivetrain.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);


        }

    }
}
