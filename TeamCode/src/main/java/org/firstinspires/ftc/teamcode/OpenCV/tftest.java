package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.AnnotatedYuvRgbFrame;

import java.util.List;

@TeleOp(name = "TF Test") @Disabled
public class tftest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "ATUOrmn/////AAABmVLVlWBtWUpnh9+EekIwR4lmMDXtnMrh/37lRyh+1m4oZJv1ANDvpS7D/Es9GNQ0wAkJ4YOHVWFjjsE5ptAFY2NRCAAwEY4VtvXEvSr3j/a0WR54dNfoCHRsnEaL5oQu25MoyOo7VrmhkE3xb2J9cNbsJzeqNaZWdQQpHkrgzEotos4i2tf/z+IMQxQ5nwH7Daiar93yoFv6FKeTh9MfI3bxVKR0nF+vrMzmNPC6YLk3yjqAKLqSgAvV0t07MBz9BjT2r58njS6qCo2U1H3sQXBlUcMdeKi4iclQaM+Oac+mZrzrhMvSEW7gC9mDhoL8l3zf2yMLPV9oGtnirNWn7ov/mupDtDecOUI4MPDNi9dt";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    double stoneY;
    double stoneX;
    private final static double hpg = 8.5;
    private final static double phi = Math.toRadians(15);
    private final static double verticalFOV = Math.toRadians(57);
    private final static double horizontalFOV = Math.toRadians(71);


    @Override
    public void runOpMode() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {initTfod();}
        else {telemetry.addData("Sorry!", "This device is not compatible with TFOD");}

        if (tfod != null) {tfod.activate();}

        waitForStart();

        while (opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                  telemetry.addData("# Object Detected", updatedRecognitions.size());

                  int i = 0;
                  if(updatedRecognitions.size()>0){
                      Recognition recognition = updatedRecognitions.get(0);
                      double xpix = recognition.getBottom() / (recognition.getImageHeight()/2) - 1;
                      double ypix = recognition.getLeft() / recognition.getImageWidth();
                      stoneY = -hpg * (Math.cos(-phi - verticalFOV / 2) / (2 * Math.sin(verticalFOV / 2)) + ypix * Math.sin(phi)) / (Math.sin(-phi - verticalFOV / 2) / (2 * Math.sin(verticalFOV / 2)) + ypix * Math.cos(phi));
                      stoneX = Math.tan(horizontalFOV / 2) * xpix * Math.sqrt(Math.pow(hpg, 2) + Math.pow(stoneY, 2));
//                      telemetry.addData("xmax: ", recognition.getImageWidth());
//                      telemetry.addData("ymax: ", recognition.getImageHeight());
//                      telemetry.addData("x: ", recognition.getBottom());
//                      telemetry.addData("y: ", recognition.getRight());
//                      telemetry.addData("xpix: ", xpix);
//                      telemetry.addData("ypix: ", ypix);
                      telemetry.addData("stonex: ", stoneX);
                      telemetry.addData("stoney: ", stoneY);

                  }


                  telemetry.update();
                }
            }
        }

        if (tfod != null) {tfod.shutdown();}
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
