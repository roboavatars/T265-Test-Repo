//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.ejml.simple.SimpleMatrix;
//import org.firstinspires.ftc.teamcode.RobotClasses.SixWheelDrivetrain;
//import org.firstinspires.ftc.teamcode.Splines.Spline;
//import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;
//
//@Autonomous
//@Disabled
//public class SplineTest extends LinearOpMode {
//
//    SixWheelDrivetrain drivetrain;
//
//    SimpleMatrix K = new SimpleMatrix(new double[][]{{0.1,0,0.0447,0},{0,0.1,0,0.0447}});
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        drivetrain = new SixWheelDrivetrain(hardwareMap,this,0,0,0);
//        waitForStart();
//
//        SplineGenerator splineGenerator = new SplineGenerator();
//        Spline[] splines = splineGenerator.SplineBetweenTwoPoints(0,0,0,
//                10,0,0,0.1,0.1,
//                0,0,0,0, 7);
//        Spline xspline = splines[0];
//        Spline yspline = splines[1];
//        ElapsedTime time = new ElapsedTime();
//        double currenttime,lasttime, currentx, currenty, currenttheta, currentxdot, currentydot, lastx, lasty;
//        lastx = 0;
//        lasty = 0;
//        lasttime = 0;
//        currentydot = 1;
//        drivetrain.setControls(currentydot,0);
//        sleep(20);
//        double robotvelo = 0;
//        while(opModeIsActive()){
//            //computing all important values
//            currenttime = time.seconds();
//            currentx = drivetrain.x;
//            currenty = drivetrain.y;
//            currenttheta = drivetrain.currentheading;
//            currentxdot = (currentx-lastx)/(currenttime-lasttime);
//            if(currentxdot>0){
//                currentxdot = Math.max(currentxdot,0.1);
//            }
//            else{
//                currentxdot = Math.min(currentxdot,-0.1);
//            }
//            currentydot = (currenty-lasty)/(currenttime-lasttime);
//
//
//            //linear controls
//            SimpleMatrix diff = new SimpleMatrix(new double[][]{{-currentx+0,
//                    -currenty+18,0,0}});
//
//            SimpleMatrix v = K.mult(diff.transpose());
//
//            //transformation
//            double velocity = Math.sqrt(Math.pow(currentxdot,2)+ Math.pow(currentydot,2));
//            SimpleMatrix u = gInv(currenttheta, velocity).mult(v);
//
//            double acceleration = u.get(0,0);
//            double w = u.get(1,0);
//            robotvelo += acceleration*(currenttime-lasttime);
//            if(currenttime>=6){
//                drivetrain.setControls(0,0);
//            }
//            else{
//                drivetrain.setControls(robotvelo,w);
//            }
//            drivetrain.updatePose();
////            telemetry.addData("raw acc", v.get(0,0));
////            telemetry.addData("raw w", v.get(1,0));
//            telemetry.addData("velo" , robotvelo);
//            telemetry.addData("w", w);
//            telemetry.addData("x: ", currentx);
//            telemetry.addData("y: ", currenty);
////            telemetry.addData("xdot: ", currentxdot);
////            telemetry.addData("ydot: ", currentydot);
//            telemetry.addData("theta", currenttheta);
//            telemetry.update();
//
//            lastx = currentx;
//            lasty = currenty;
//            lasttime = currenttime;
//        }
//
//    }
//    public SimpleMatrix gInv(double theta, double velocity){
//        SimpleMatrix GInv = new SimpleMatrix(new double[][]{{Math.cos(theta),Math.sin(theta)},
//                {-Math.sin(theta)/velocity, Math.cos(theta)/velocity}});
//        return GInv;
//    }
//
//}
