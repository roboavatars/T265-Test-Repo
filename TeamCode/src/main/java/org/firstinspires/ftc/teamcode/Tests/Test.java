package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp //@Config
public class Test extends LinearOpMode {

    private Servo extendoServo;
    public static double extendoExtendPos = 0;
    public static double extendoHomePos = 1;
    public static boolean home = true;

    @Override
    public void runOpMode() {

        extendoServo = hardwareMap.get(Servo.class, "extendoServo");
        extendoServo.setPosition(extendoHomePos);

        waitForStart();

        while(opModeIsActive()) {

            if (home) {extendoServo.setPosition(extendoHomePos);}
            else {extendoServo.setPosition(extendoHomePos);}
        }
    }
}