package org.firstinspires.ftc.teamcode.OpenCV.copypastetest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;

import static org.firstinspires.ftc.teamcode.OpenCV.copypastetest.StoneWranglerUtils.log;

@TeleOp(name="Online OpenCV Test")
@Disabled
public class OpenCVCopy extends LinearOpMode {

    private StoneWrangler stoneWrangler;

    @Override
    public void runOpMode() {

        waitForStart();

        stoneWrangler = new StoneWrangler();
        stoneWrangler.analyze(Imgcodecs.imread("/sdcard/FIRST/testFiles2/test2.jpg", Imgcodecs.IMREAD_COLOR));
        Imgcodecs.imwrite("/sdcard/FIRST/procFiles2/output.jpg", stoneWrangler.getVisualization());
        log("done");
    }
}