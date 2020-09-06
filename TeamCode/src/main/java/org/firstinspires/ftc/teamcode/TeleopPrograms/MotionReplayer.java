package org.firstinspires.ftc.teamcode.TeleopPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.ArrayList;

@TeleOp
@Config
public class MotionReplayer extends LinearOpMode {

    private Robot robot;
    private ArrayList<double[]> data;
    private int counter = 0;
    private double curtime = 0;
    public static double fileNumber = Logger.getLastFileNumber();

    @Override
    public void runOpMode() {
        robot = new Robot(this, 9, 111, 0, false);

        waitForStart();
        try {
            data = robot.logger.replay("/sdcard/FIRST/robotLogs/RobotData" + fileNumber + ".csv");
        } catch (Exception e) {
            data = new ArrayList<>();
        }

        while (opModeIsActive()) {
            if (data.size() > 0) {
                curtime = data.get(counter)[0];
                robot.drawRobot(data.get(counter)[1], data.get(counter)[2], data.get(counter)[3]);
                robot.sendPacket();
                sleep((long) curtime - (long) data.get(Math.max(0,counter-1))[0]);
                counter = Math.min(counter+1, data.size()-1);
            } else {
                stop();
            }
        }
    }
}