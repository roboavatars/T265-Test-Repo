//package org.firstinspires.ftc.teamcode.Tests.graph;
//
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
//import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
//import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
//
//@Autonomous
//public class NewAuto extends LinearOpMode {
//
//    private Robot robot;
//    private skyStoneDetector detector;
//    private Graph graph;
//
//    @Override
//    public void runOpMode() {
//
//        // initialize skystone detector
//        detector = new skyStoneDetector(this);
//        detector.initializeCamera();
//        detector.start();
//        detector.setActive(true);
//        detector.isAllianceRed(true);
//
//        // initialize robot
//        robot = new Robot(this, 9, 111, 0);
//        robot.logger.startLogging();
//        robot.grabber.releaseFoundation();
//        robot.intake.setControls(0);
//        robot.stacker.unClampStone();
//        robot.stacker.goHome();
//        graph = new Graph(19, robot);
//
//        // after start
//        waitForStart();
//
//        // skystone stuff
//        double skystonePos = detector.getPosition(); detector.setActive(false);
//        double skystoneY = robot.drivetrain.y;
//        if (skystonePos == 1) skystoneY = 129;
//        else if (skystonePos == 2) skystoneY = 121;
//        else if (skystonePos == 3) skystoneY = 112; //*/ double skystoneY = 129; graph = new Graph(19, robot);
//
//        // time variables (might not be needed in future)
//        double skystone1Time = 2.5;
//        double backToCenterTime = 1;
//        double foundationTurnTime = 1.75;
//        double toQuarryTime = 3;
//        double skystone2Time = 2;
//
//        // add nodes
//        Node skystone1 = new Node(9, 111, 45, skystoneY, 0, Math.PI / 4,
//                0, 0, 20, 0, 0, 0, skystone1Time, "ss1");
//        skystone1.addForcedTheta(Math.PI / 4 + 0.15);
//
//        Node adjustPosSS1 = new Node(1, 0.8 , 0.07, true, "ap1");
//
//        Node backToCenter1 = new Node(45, skystoneY, 33, skystoneY - 12, Math.PI / 4, Math.PI / 2,
//                0, -70, -20, -50, 0, 0, backToCenterTime, "bc1");
//        backToCenter1.addThetaSpline(Math.PI / 4, 0, 0, Math.PI / 2, 0, 0);
//
//        Node toFoundation1 = new Node(36, 55, Math.PI / 2, "tf1");
//
//        Node foundationTurn = new Node(36, 55, 31, 36, Math.PI / 2, Math.PI,
//                -70, 0, -50, 0, 0, 0, foundationTurnTime, "ftn");
//        foundationTurn.addThetaSpline(Math.PI / 2, 0, 0, Math.PI, 0, 0);
//
//        Node approachFoundation = new Node(44, 25, Math.PI, "apf");
//
//        Node grabFoundation = new Node(Node.robotAction.GrabFoundation, "gbf"); grabFoundation.noTimeReset();
//
//        Node depositStone1 = new Node(Node.robotAction.DepositStone, "ds1");
//
//        Node pullFoundation = new Node(26, 35, Math.PI, 0.6, 0.6, 0.4, "plf");
//
//        Node turnFoundation = new Node(35, 35, Math.PI / 2, 0, 0, 3, "tnf");
//
//        Node pushFoundation = new Node(35, 29, Math.PI / 2, 0.1, 0.4, 0.8, "phf");
//
//        Node releaseFoundation = new Node(Node.robotAction.ReleaseFoundation, "ref");
//
//        Node toQuarry = new Node(35, 29, 24, skystoneY - 30, Math.PI / 2, Math.PI / 4,
//                0, 0, 20, 0, 0, 0, toQuarryTime, "tqy");
//        toQuarry.addThetaSpline(Math.PI / 2, 0, 0, Math.PI / 4, 0, 0);
//
//        Node skystone2 = new Node(24, skystoneY - 30, 45, skystoneY - 26, Math.PI / 2, Math.PI / 4,
//                30, 0, 20, 0, 0, 0, skystone2Time, "ss2");
//        skystone2.addForcedTheta(Math.PI / 4 + 0.15);
//
//        Node adjustPosSS2 = new Node(1, 0, 0.1, "ap2");
//
//        Node backToCenter2 = new Node(33, 91, Math.PI / 2, "bc2");
//
//        Node toFoundation2 = new Node(33, 33, Math.PI / 2, "tf2");
//
//        Node depositStone2 = new Node(Node.robotAction.DepositStone, "ds2");
//
//        Node toTape = new Node(30, 72, Math.PI / 2, 0.14, 0.07, 0.8, "ttp");
//
//        // add edges--------------------------------------------------------------------------------------------------
//        Edge ss1_ap1 = new Edge(skystone1, adjustPosSS1);
//        ss1_ap1.setTimeStartCondition(skystone1Time);
//
//        Edge ap1_bc1 = new Edge(adjustPosSS1, backToCenter1);
//        ap1_bc1.setStateStartCondition(Node.robotState.StoneClamped);
//        ap1_bc1.setTimeStartCondition(3);
//
//        Edge bc1_tf1 = new Edge(backToCenter1, toFoundation1);
//        bc1_tf1.setTimeStartCondition(backToCenterTime);
//
//        Edge tf1_ftn = new Edge(toFoundation1, foundationTurn);
//        tf1_ftn.setPosStartCondition(36, 58, Math.PI / 2, 144, 3, 1);
//
//        Edge ftn_apf = new Edge(foundationTurn, approachFoundation);
//        ftn_apf.setTimeStartCondition(foundationTurnTime);
//
//        Edge apf_gbf = new Edge(approachFoundation, grabFoundation);
//        apf_gbf.setTimeStartCondition(0.3);
//
//        Edge gbf_ds1 = new Edge(grabFoundation, depositStone1);
//        gbf_ds1.setPosStartCondition(44, 25, Math.PI, 1, 1, 0.1);
//        gbf_ds1.setTimeStartCondition(2);
//        gbf_ds1.setStateStartCondition(Node.robotState.StoneClamped);
//
//        Edge gbf_plf = new Edge(grabFoundation, pullFoundation);
//        gbf_plf.setPosStartCondition(44, 25, Math.PI, 1, 1, 0.1);
//        gbf_plf.setTimeStartCondition(2);
//        gbf_plf.setStateStartCondition(Node.robotState.StoneNotClamped);
//
//        Edge ds1_plf = new Edge(depositStone1, pullFoundation);
//
//        Edge plf_tnf = new Edge(pullFoundation, turnFoundation);
//        plf_tnf.setPosStartCondition(26, 35, Math.PI, 1, 1, 0.1);
//        plf_tnf.setTimeStartCondition(1.5);
//
//        Edge tnf_phf = new Edge(turnFoundation, pushFoundation);
//        tnf_phf.setPosStartCondition(35, 35, Math.PI / 2, 144, 144, 0.1);
//        tnf_phf.setTimeStartCondition(1);
//
//        Edge phf_ref = new Edge(pushFoundation, releaseFoundation);
//        phf_ref.setStateStartCondition(Node.robotState.ArmHome);
//        phf_ref.setTimeStartCondition(1.5);
//
//        Edge ref_tqy = new Edge(releaseFoundation, toQuarry);
//
//        Edge tqy_ss2 = new Edge(toQuarry, skystone2);
//        tqy_ss2.setTimeStartCondition(toQuarryTime);
//
//        Edge ss2_ap2 = new Edge(skystone2, adjustPosSS2);
//        ss2_ap2.setTimeStartCondition(skystone2Time);
//
//        Edge ap2_bc2 = new Edge(adjustPosSS2, backToCenter2);
//        ap2_bc2.setStateStartCondition(Node.robotState.StoneClamped);
//        ap2_bc2.setTimeStartCondition(3);
//
//        Edge ap2_ttp = new Edge(adjustPosSS2, toTape);
//        ap2_ttp.setStateStartCondition(Node.robotState.StoneNotClamped);
//        ap2_ttp.setTimeStartCondition(3);
//
//        Edge bc2_tf2 = new Edge(backToCenter2, toFoundation2);
//        bc2_tf2.setPosStartCondition(33, 85, Math.PI / 2, 144, 3, 1);
//
//        Edge tf2_ds1 = new Edge(toFoundation2, depositStone2);
//        tf2_ds1.setPosStartCondition(33, 60, Math.PI / 2, 144, 1, 1);
//
//        Edge ds2_ttp = new Edge(depositStone2, toTape);
//        ds2_ttp.setStateStartCondition(Node.robotState.ArmHome);
//
//        graph.addNode(skystone1, adjustPosSS1, backToCenter1, toFoundation1, foundationTurn, approachFoundation,
//                grabFoundation, depositStone1, pullFoundation, turnFoundation, pushFoundation, releaseFoundation,
//                toQuarry, skystone2, adjustPosSS2, backToCenter2, toFoundation2, depositStone2, toTape);
//        graph.addEdge(ss1_ap1, ap1_bc1, bc1_tf1, tf1_ftn, ftn_apf, apf_gbf, gbf_ds1, gbf_plf, ds1_plf, plf_tnf,
//                tnf_phf, phf_ref, ref_tqy, tqy_ss2, ss2_ap2, ap2_bc2, ap2_ttp, bc2_tf2, tf2_ds1, ds2_ttp);
//        graph.showRelationships();
//
//        while (opModeIsActive()) {
//            robot.update(); graph.update();
//
//            telemetry.addData("skystone position", skystonePos);
//            telemetry.addData("x", robot.drivetrain.x);
//            telemetry.addData("y", robot.drivetrain.y);
//            telemetry.addData("theta", robot.drivetrain.currentheading);
//            telemetry.update();
//        }
//
//        robot.update();
//        robot.logger.flush(); robot.logger.stopLogging(); //*/
//    }
//}