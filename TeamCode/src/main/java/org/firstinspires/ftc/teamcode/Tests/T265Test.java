package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.T265;

@TeleOp(name="T265 Test")
public class T265Test extends LinearOpMode {

    private T265 t265;

    @Override
    public void runOpMode() {
        t265 = new T265(this, 9, 111, Math.PI/2);

        waitForStart();
        t265.startCam();

        while(opModeIsActive()){
            t265.updateCamPose();
            telemetry.addData("X: " , t265.getCamX());
            telemetry.addData("Y: " , t265.getCamY());
            telemetry.addData("Theta: " , t265.getCamTheta());
            telemetry.update();
        }

        t265.stopCam();
    }
}