package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Stacker;

@TeleOp(name="Stacker Test") @SuppressWarnings("FieldCanBeLocal")
public class StackerTest extends LinearOpMode {

    private Stacker stacker;

    @Override
    public void runOpMode() {
        stacker = new Stacker(this);
//        stacker.unClampStone();
        telemetry.update();
        stacker.setDepositControls(0,0);

        waitForStart();
        boolean rightBumper = true;

        while(opModeIsActive()){

//            if(gamepad1.right_bumper && rightBumper){
//                rightBumper = false;
//            }
//            else if(!rightBumper && !gamepad1.right_bumper){
//                stacker.deposit();
//                rightBumper = true;
//            }
            stacker.update();
            telemetry.addData("ticks: " , stacker.getArmPosition());
            telemetry.update();
        }
    }
}