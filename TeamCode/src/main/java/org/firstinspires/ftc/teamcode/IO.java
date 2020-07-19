package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Input/Output")
public class IO extends LinearOpMode {

    private DcMotor motor;
    private Servo servo;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class,"motor");
        servo = hardwareMap.get(Servo.class, "servo");

        motor.setPower(0);
        servo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
//            gamepad1.a;
//            gamepad1.b;
//            gamepad1.x;
//            gamepad1.y;
//
//            gamepad1.dpad_up;
//            gamepad1.dpad_down;
//            gamepad1.dpad_left;
//            gamepad1.dpad_right;
//
//            gamepad1.left_stick_x;
//            gamepad1.left_stick_y;
//            gamepad1.left_trigger;
//
//            gamepad1.right_stick_x;
//            gamepad1.right_stick_y;
//            gamepad1.right_trigger;
//
//            gamepad2.a;
//            ...

            telemetry.addData("Motor Power: ", motor.getPower());
            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.update();
        }
    }
}