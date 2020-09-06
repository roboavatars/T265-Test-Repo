package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV")
public class detectorTest extends LinearOpMode {

    private final int ssAligned = 125;

    @Override public void runOpMode() {
        skyStoneDetector detector = new skyStoneDetector(this);
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(false);
        telemetry.addData("Status", "Ready"); telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            double curPos = detector.getPosition();
            if (Math.abs(ssAligned - curPos) < 5) {telemetry.addData("Status", "Ready to Get SkyStone");}

            telemetry.addData("SkyStone Position: ", curPos);
            telemetry.addData("SkyStone X Location", detector.getSSPosX());
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double curPos = detector.getPosition();
            if (Math.abs(ssAligned - curPos) < 5) {telemetry.addData("Status", "Ready to Get SkyStone");}

            telemetry.addData("SkyStone Position: ", curPos);
            telemetry.addData("SkyStone X Location", detector.getSSPosX());
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }
        detector.setActive(false);
        telemetry.addData("Status", "Done"); telemetry.update();
    }
}