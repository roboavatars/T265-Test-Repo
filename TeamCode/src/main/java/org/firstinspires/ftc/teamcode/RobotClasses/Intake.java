package org.firstinspires.ftc.teamcode.RobotClasses;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
//@Config
public class Intake {

    // Electronics
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private Servo stonePushServo;

    private final double stonePushPos = 0.9;
    private final double homePos = 0.27;
    private double lastIntakePower = 0;
    private double motorUpdateTolerance = 0.1;
    public boolean stonePushed;

    // OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Intake(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        stonePushServo = hardwareMap.get(Servo.class, "stonePushServo");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        stoneServoHome();

        op.telemetry.addData("Status", "Intake Initialized");
        op.telemetry.update();
    }

    public void setControls(double intakePower) {
        if (Math.abs(intakePower - lastIntakePower) > motorUpdateTolerance){
            leftIntake.setPower(-intakePower);
            rightIntake.setPower(-intakePower);
            lastIntakePower = intakePower;
        }
    }

    public void pushStoneIn() {
        stonePushServo.setPosition(stonePushPos);
        stonePushed = true;
    }

    public void stoneServoHome() {
        stonePushServo.setPosition(homePos);
        stonePushed = false;
    }
}